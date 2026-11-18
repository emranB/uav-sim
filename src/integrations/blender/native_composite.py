"""Compose recorded native UI pixels with synchronized offline aircraft cameras."""

import datetime
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Callable

from common.logging.storage import write_json_atomic


def load_native_recording(
    directory: Path, check: Callable[[], None], timeout_seconds: float = 20
) -> dict:
    """Wait for a matching sealed screen recording; never substitute a previous run."""
    export = json.loads((directory / "export-manifest.json").read_text())
    capture_id = export["capture_id"]
    package = directory.parent.parent / "native-screen" / capture_id
    deadline = time.monotonic() + timeout_seconds
    while True:
        check()
        manifest = package / "recording.json"
        if manifest.is_file():
            recording = json.loads(manifest.read_text())
            if recording.get("state") == "failed":
                raise RuntimeError(recording.get("error", "Native screen recording failed"))
            if recording.get("state") == "completed":
                break
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Native screen recording was not sealed for {capture_id}")
        time.sleep(0.1)
    if recording["capture_id"] != capture_id or recording["dimensions"] != [1600, 1000]:
        raise ValueError("Native screen identity or unscaled dimensions do not match")
    movie = package / "native-screen.mkv"
    if movie.is_symlink() or not movie.is_file():
        raise ValueError("Native recording movie is unavailable")
    if hashlib.sha256(movie.read_bytes()).hexdigest() != recording["sha256"]:
        raise ValueError("Native recording checksum differs from its sealed manifest")
    first = recording["first_input_wall_time"]
    stamps = recording["input_frame_wall_times"]
    if len(stamps) < 2 or first != stamps[0] or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("Native input timestamps must be strictly increasing")
    if max(b - a for a, b in zip(stamps, stamps[1:])) > 0.25:
        raise ValueError("Native screen input gap exceeded 0.25 seconds")
    capture_path = directory.parent.parent / "replay" / capture_id / "manifest.json"
    capture = json.loads(capture_path.read_text())
    started = (
        capture["started_wall_time"] + export["start_monotonic"] - capture["started_monotonic"]
    )
    start = max(0, first - started)
    end = min(export["end_monotonic"] - export["start_monotonic"], stamps[-1] - started)
    if not 0 <= start < end or end - start < 0.2:
        raise ValueError("Native recording has no usable overlap with the selected capture")
    return {
        **recording,
        "movie": str(movie),
        "capture_started_wall_time": started,
        "start_seconds": start,
        "end_seconds": end,
        "native_end_seconds": min(end + 5, stamps[-1] - started),
        "native_start_seconds": started + start - first,
        "source_manifest": str(manifest),
    }


def composite_annotations(path: Path, title: str, recording: dict, fps: int = 10) -> None:
    """Write labels outside source panels; timestamps use the capture UTC clock."""
    title = title.replace("\\", "/").replace("{", "(").replace("}", ")")
    header = """[Script Info]
ScriptType: v4.00+
PlayResX: 2464
PlayResY: 1180
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Text,DejaVu Sans,20,&H00F6EDE6,&H00F6EDE6,&H001E120A,&H001E120A,0,0,0,0,100,100,0,0,1,0,0,7,0,0,0,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
    duration_seconds = recording.get("native_end_seconds", recording["end_seconds"])
    count = math.ceil((duration_seconds - recording["start_seconds"]) * fps - 1e-8)
    replay_frames = math.ceil((recording["end_seconds"] - recording["start_seconds"]) * fps - 1e-8)
    last_replay_offset = recording["start_seconds"] + (replay_frames - 1) / fps

    def stamp(seconds: float) -> str:
        ticks = round(seconds * 100)
        return f"{ticks // 360000}:{ticks // 6000 % 60:02}:{ticks // 100 % 60:02}.{ticks % 100:02}"

    events = []

    def event(begin: float, end: float, body: str) -> None:
        events.append(f"Dialogue: 0,{stamp(begin)},{stamp(end)},Text,,0,0,0,,{body}\n")

    duration = count / fps
    for body in (
        r"{\pos(24,18)\fs28\b1}" + title,
        r"{\pos(24,62)\fs20}ACTUAL NATIVE APPLICATION RECORDING | QGC / GAZEBO / ORB / CONTROLS",
        r"{\pos(1640,18)\fs22\b1}SYNCHRONIZED RECORDED-POSE REPLAY",
        r"{\pos(1656,120)\fs22\b1\1c&HB1D939&}ATLAS",
        r"{\pos(1656,626)\fs22\b1\1c&HFFA562&}BOREAL",
        r"{\pos(1796,124)\fs18}Blender | postprocessed replay",
        r"{\pos(1796,630)\fs18}Blender | postprocessed replay",
        r"{\pos(24,1130)\fs18}Native UI pixels preserved at 1600 x 1000 | fault and health states are recorded evidence",
        r"{\pos(1640,1130)\fs17}Separate follow cameras | recorded simulator poses",
        r"{\pos(24,1157)\fs14}Blender views generated after capture; not simultaneous live views. Rotor RPM was not recorded.",
    ):
        event(0, duration, body)
    for frame in range(count):
        offset = recording["start_seconds"] + frame / fps
        utc = (
            datetime.datetime.fromtimestamp(
                recording["capture_started_wall_time"] + offset, datetime.timezone.utc
            )
            .isoformat(timespec="milliseconds")
            .replace("+00:00", "Z")
        )
        sealed = offset >= recording["end_seconds"]
        caption = (
            f" | SEALED at +{last_replay_offset:.3f} s" if sealed else f" | capture +{offset:.3f} s"
        )
        event(frame / fps, (frame + 1) / fps, r"{\pos(1640,60)\fs18}" + utc + caption)
        if sealed:
            for y in (580, 1086):
                event(
                    frame / fps,
                    (frame + 1) / fps,
                    rf"{{\pos(1656,{y})\fs18\bord2}}CAPTURE ENDED | LAST RECORDED POSE",
                )
    path.write_text(header + "".join(events))


def composite_command(directory: Path, recording: dict, title: str) -> list[str]:
    """Keep the native pane's aspect ratio and give both aircraft equal viewports."""
    duration = (
        recording.get("native_end_seconds", recording["end_seconds"]) - recording["start_seconds"]
    )
    tail = max(0, duration - recording["end_seconds"] + recording["start_seconds"])
    composite_annotations(directory / "composite-labels.ass", title, recording)
    graph = (
        f"[1:v]trim=start={recording['native_start_seconds']}:duration={duration},"
        "setpts=PTS-STARTPTS,fps=10[ui];"
        f"[2:v]tpad=stop_mode=clone:stop_duration={tail + 0.1}[atlas];"
        f"[3:v]tpad=stop_mode=clone:stop_duration={tail + 0.1}[boreal];"
        "[0:v][ui]overlay=24:112:eof_action=endall[a];"
        "[a]drawbox=x=1640:y=112:w=800:h=494:color=0x111f31:t=fill,"
        "drawbox=x=1640:y=618:w=800:h=494:color=0x111f31:t=fill[b];"
        "[b][atlas]overlay=1640:156:eof_action=endall[c];"
        "[c][boreal]overlay=1640:662:eof_action=endall,ass=composite-labels.ass[out]"
    )
    write_json_atomic(
        directory / "native-synchronization.json",
        {
            **recording,
            "maximum_video_quantization_seconds": 0.1,
            "pose_selection": "Latest recorded observation at or before each capture offset",
            "layout": {
                "canvas": [2464, 1180],
                "native": [24, 112, 1600, 1000],
                "atlas": [1640, 156, 800, 450],
                "boreal": [1640, 662, 800, 450],
            },
        },
    )
    return [
        "ffmpeg",
        "-nostdin",
        "-v",
        "error",
        "-n",
        "-f",
        "lavfi",
        "-i",
        f"color=c=0x0a121e:s=2464x1180:r=10:d={duration}",
        "-i",
        recording["movie"],
        "-i",
        str(directory / "atlas/mission-replay.mp4"),
        "-i",
        str(directory / "boreal/mission-replay.mp4"),
        "-filter_complex_threads",
        "2",
        "-filter_complex",
        graph,
        "-map",
        "[out]",
        "-t",
        str(duration),
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-crf",
        "18",
        "-threads",
        "2",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(directory / "mission-replay.mp4"),
    ]
