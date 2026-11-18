"""Retain revision-matched model assets and generate measured scene annotations."""

import base64
import hashlib
import json
import shutil
from collections.abc import Callable
from pathlib import Path

from common.logging.replay_timeline import RecordedReplay
from common.logging.storage import write_json_atomic
from integrations.blender.overlay import fresh_received


def retain_aircraft_assets(directory: Path, model_root: Path, revision_file: Path) -> Path:
    """Copy the pinned x500 visual inputs; reject a different native model revision."""
    context = json.loads((directory / "context.json").read_text())
    revisions = {
        row.get("native_revisions", {}).get("models-revision")
        for row in context.get("vehicles", {}).values()
    }
    revision = revision_file.read_text().strip()
    if revisions != {revision}:
        raise ValueError("Offline aircraft assets do not match the recorded model revision")
    if any(
        row.get("configuration", {}).get("model") != "x500" for row in context["vehicles"].values()
    ):
        raise ValueError("This offline scene supports recorded x500 aircraft only")
    source = model_root / "x500_base"
    files = [
        p
        for p in source.rglob("*")
        if p.is_file() and not p.is_symlink() and "thumbnails" not in p.parts
    ]
    if not files or sum(p.stat().st_size for p in files) > 64 * 1024**2:
        raise ValueError("Recorded aircraft asset package is missing or exceeds 64 MiB")
    destination = directory / "scene-assets"
    destination.mkdir()
    hashes = {}
    for name, retained in context.get("retained_geographic_images", {}).items():
        if name != "texture.png" or len(retained["base64"]) > 6 * 1024**2:
            raise ValueError("Retained terrain image has an invalid name or exceeds budget")
        encoded = base64.b64decode(retained["base64"], validate=True)
        if hashlib.sha256(encoded).hexdigest() != retained["sha256"]:
            raise ValueError("Retained terrain image checksum differs")
        (destination / name).write_bytes(encoded)
        hashes[name] = retained["sha256"]
    if (
        sum(
            len(row["contents"].encode())
            for row in context.get("retained_geographic_assets", {}).values()
        )
        > 32 * 1024**2
    ):
        raise ValueError("Retained geographic meshes exceed 32MiB")
    for name, retained in context.get("retained_geographic_assets", {}).items():
        if Path(name).name != name or not name.endswith(".obj"):
            raise ValueError("Retained geographic mesh requires a plain OBJ filename")
        encoded = retained["contents"].encode("utf-8")
        if hashlib.sha256(encoded).hexdigest() != retained["sha256"]:
            raise ValueError(f"Retained geographic mesh checksum differs: {name}")
        (destination / name).write_bytes(encoded)
        hashes[name] = retained["sha256"]
    for path in files:
        target = destination / "x500_base" / path.relative_to(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
        hashes[str(target.relative_to(destination))] = hashlib.sha256(
            target.read_bytes()
        ).hexdigest()
    write_json_atomic(
        destination / "asset-manifest.json",
        {
            "models_revision": revision,
            "files_sha256": hashes,
            "source": "Revision-matched offline runtime model assets; no live simulator access",
        },
    )
    return destination


def write_scene_annotations(
    replay: RecordedReplay,
    path: Path,
    start: float,
    frame_count: int,
    fps: int,
    title: str,
    project: Callable[[tuple], tuple[float, float]] | None = None,
) -> None:
    """Write only screen text; every animated aircraft and trail is rendered in Blender."""

    def clean(value: object) -> str:
        return str(value).replace("\\", "/").replace("{", "(").replace("}", ")").replace("\n", " ")

    def stamp(value: float) -> str:
        ticks = round(value * 100)
        return f"{ticks // 360000}:{ticks // 6000 % 60:02}:{ticks // 100 % 60:02}.{ticks % 100:02}"

    header = """[Script Info]
ScriptType: v4.00+
PlayResX: 1280
PlayResY: 720
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Text,DejaVu Sans,18,&H00FFFFFF,&H00FFFFFF,&H00141D2B,&H00141D2B,0,0,0,0,100,100,0,0,1,1,0,7,0,0,0,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
    events = []
    duration = frame_count / fps

    def event(layer: int, start_time: float, end_time: float, body: str) -> None:
        events.append(
            f"Dialogue: {layer},{stamp(start_time)},{stamp(end_time)},Text,,0,0,0,,{body}\n"
        )

    event(
        0,
        0,
        duration,
        r"{\p1\bord0\1c&H2B1D14&}m 0 0 l 1280 0 1280 112 0 112 m 0 590 l 1280 590 1280 720 0 720",
    )
    event(1, 0, duration, r"{\pos(22,16)\fs29}" + clean(title))
    event(
        1,
        0,
        duration,
        r"{\pos(22,58)\fs16}3D BLENDER RECONSTRUCTION | RECORDED SIMULATOR POSES | NOT CAMERA FOOTAGE",
    )
    event(
        1,
        0,
        duration,
        r"{\pos(22,696)\fs13}World ENU metres | 1x capture wall time | latest observed poses; gaps hidden | rotor RPM not recorded",
    )
    if project is not None:
        for identity in replay.samples:
            markers = replay.planned_markers(identity, scene_frame=True)
            if markers is None:
                continue
            for name, point, offset in (
                ("HOME", markers[0], (42, 20)),
                ("WP1 planned", markers[1], (12, 24)),
            ):
                x, y = project((point[0], point[1], 0.08))
                if 0 <= x < 1200 and 115 <= y < 560:
                    event(
                        2,
                        0,
                        duration,
                        f"{{\\pos({x + offset[0]:.1f},{y + offset[1]:.1f})\\fs17\\bord2\\shad0}}"
                        + clean(name + " " + replay.label(identity)),
                    )
    for frame in range(frame_count):
        elapsed = start + frame / fps
        begin, end = frame / fps, (frame + 1) / fps
        event(
            1,
            begin,
            end,
            r"{\pos(22,86)\fs17}" + f"Capture {elapsed:.1f} / {replay.duration_seconds:.1f} s",
        )
        for index, identity in enumerate(replay.samples):
            sample = replay.at(identity, elapsed)
            observation = sample.observation if sample else {}
            if (
                project is not None
                and sample is not None
                and elapsed - sample.elapsed_wall_seconds <= 0.3
            ):
                point = replay.scene_position(observation)
                if point is not None and fresh_received(
                    observation.get("evaluation_truth_received_monotonic"),
                    replay.started_monotonic + elapsed,
                ):
                    x, y = project((point[0], point[1], point[2] + 0.2))
                    if 0 <= x < 1190 and 130 < y < 580:
                        event(
                            3,
                            begin,
                            end,
                            f"{{\\pos({x + 26:.1f},{y - 22:.1f})\\fs20\\bord2\\shad0}}"
                            + clean(replay.label(identity)),
                        )
            aiding = observation.get("aiding") or {}
            mission = observation.get("mission") or {}
            truth = observation.get("evaluation_truth") or {}

            def flag(value: object) -> str:
                return "ON" if value is True else "OFF" if value is False else "UNKNOWN"

            lines = [
                f"{replay.label(identity)} | {mission.get('state', 'unknown')} / {mission.get('leg', '-')}",
                f"GPS request {flag(aiding.get('gps_requested_enabled'))} | injected loss {flag(aiding.get('gps_loss_injected'))} | fix {aiding.get('gps_fix_type', '?')}",
                f"Measured GNSS XY {flag(aiding.get('gnss_xy_active'))} | vision XY {flag(aiding.get('vision_xy_active'))}",
                f"Native sim {truth.get('simulation_time_seconds', '?')} s",
            ]
            colour = "A1D12E" if index == 0 else "FF9E4D"
            for row, text in enumerate(lines):
                event(
                    1,
                    begin,
                    end,
                    f"{{\\pos({22 + index * 640},{603 + row * 21})\\fs17\\1c&H{colour}&}}"
                    + clean(text),
                )
    path.write_text(header + "".join(events))
