"""Bounded X11 window recording bound to an acknowledged replay capture."""

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import uuid
from pathlib import Path

from common.logging.storage import write_json_atomic


def native_input_timestamps(log_text: str) -> tuple[list[float], int]:
    """Read actual input PTS, reporting post-mux-close read-ahead separately.

    FFmpeg 6 can finish its demux thread after printing the closed muxer's summary.
    Those packets never enter the recorded video. Keep them in the log and report
    their count; every input before mux close must still match an output frame.
    """
    pattern = r"demuxer -> [^\n]*?pkt_pts:(\d+)"
    closed = re.search(r"(?m)^.*\bvideo:\S+.*\baudio:\S+.*muxing overhead:.*$", log_text)
    after_close = []
    if closed is not None:
        after_close = [int(value) for value in re.findall(pattern, log_text[closed.end() :])]
        log_text = log_text[: closed.start()]
    recorded = [int(value) for value in re.findall(pattern, log_text)]
    if after_close and (
        not recorded or any(b <= a for a, b in zip([recorded[-1], *after_close], after_close))
    ):
        raise ValueError("Post-close demux input overlaps the recorded native interval")
    return [value / 1e6 for value in recorded], len(after_close)


class NativeScreenRecording:
    """Own one recorder process and bind its actual timestamps to one acknowledged capture."""

    def __init__(
        self,
        outputs: Path,
        window_id: int,
        display: str = ":0",
        title: str = "Recorded mission",
        dimensions: tuple[int, int] = (1600, 1000),
    ) -> None:
        if type(window_id) is not int or window_id <= 0:
            raise ValueError("Choose one positive native window ID")
        if len(title) > 120 or any(ord(char) < 32 for char in title):
            raise ValueError("Recording title must contain at most 120 printable characters")
        if not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
            raise RuntimeError("Native capture requires FFmpeg; build the native desktop image")
        self.outputs = outputs
        self.window_id = window_id
        self.display = display
        self.title = title
        if dimensions not in {(1600, 1000), (1920, 1200)}:
            raise ValueError("Native capture requires a declared 1600x1000 or 1920x1200 layout")
        self.dimensions = dimensions
        self.directory = outputs / "native-screen" / ("pending-" + uuid.uuid4().hex)
        self.directory.mkdir(parents=True)
        self.capture_id: str | None = None
        self.process: subprocess.Popen | None = None
        self.log = None

    def start(self) -> None:
        """Start before clicking capture/mission controls; source is the real native window."""
        self.log = (self.directory / "input-timestamps.log").open("w")
        command = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "info",
            "-debug_ts",
            "-f",
            "x11grab",
            "-use_wallclock_as_timestamps",
            "1",
            "-framerate",
            "10",
            "-video_size",
            f"{self.dimensions[0]}x{self.dimensions[1]}",
            "-window_id",
            str(self.window_id),
            "-i",
            self.display,
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "18",
            "-bf",
            "0",
            "-threads",
            "2",
            "-pix_fmt",
            "yuv420p",
            "-vsync",
            "0",
            "-t",
            "1800",
            str(self.directory / "native-screen.mkv"),
        ]
        write_json_atomic(
            self.directory / "command.json", {"command": command, "started_wall_time": time.time()}
        )
        guarded = [
            sys.executable,
            "-m",
            "integrations.blender.guarded_child",
            str(os.getpid()),
            "--",
            *command,
        ]
        self.process = subprocess.Popen(
            guarded,
            stdin=subprocess.PIPE,
            stdout=self.log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError("Native encoder failed before its first frame")
            if "demuxer ->" in (self.directory / "input-timestamps.log").read_text():
                return
            time.sleep(0.05)
        raise TimeoutError("Native encoder did not acknowledge its first frame within 5 seconds")

    def bind(self, capture_id: str) -> None:
        """Use the acknowledged capture ID; never infer identity from current UI focus."""
        if self.capture_id or not re.fullmatch(r"capture-[a-f0-9]{32}", capture_id):
            raise ValueError("Native screen requires one valid capture identity")
        self.capture_id = capture_id
        destination = self.directory.with_name(capture_id)
        if destination.exists():
            raise FileExistsError("A native recording already exists for this capture")
        self.directory.rename(destination)
        self.directory = destination
        write_json_atomic(
            self.directory / "recording.json",
            {
                "state": "recording",
                "capture_id": capture_id,
                "dimensions": list(self.dimensions),
            },
        )

    def finish(self) -> dict:
        """Flush the owned encoder, verify frame timestamps, then publish the sealed derivative."""
        try:
            if self.process is None or self.process.poll() is not None:
                raise RuntimeError("Native recorder ended before requested stop")
            write_json_atomic(
                self.directory / "recording.json",
                {
                    "state": "finalizing",
                    "capture_id": self.capture_id,
                    "finalization_deadline_wall_time": time.time() + 60,
                },
            )
            self.process.stdin.write(b"q\n")
            self.process.stdin.flush()
            if self.process.wait(timeout=8):
                raise RuntimeError("Native screen encoder failed; see input-timestamps.log")
            self.log.close()
            stamps, shutdown_read_ahead = native_input_timestamps(
                (self.directory / "input-timestamps.log").read_text()
            )
            probe = json.loads(
                subprocess.check_output(
                    [
                        "ffprobe",
                        "-v",
                        "error",
                        "-select_streams",
                        "v:0",
                        "-show_frames",
                        "-show_entries",
                        "frame=best_effort_timestamp_time,width,height",
                        "-of",
                        "json",
                        str(self.directory / "native-screen.mkv"),
                    ],
                    text=True,
                    timeout=45,
                )
            )
            frames = probe["frames"]
            if len(frames) < 2 or len(frames) != len(stamps):
                raise ValueError("Native input/output frame counts differ")
            errors = [
                abs(stamp - stamps[0] - float(frame["best_effort_timestamp_time"]))
                for stamp, frame in zip(stamps, frames)
            ]
            if max(b - a for a, b in zip(stamps, stamps[1:])) > 0.25:
                raise ValueError("Native input gap exceeded 0.25 seconds")
            if max(errors) > 0.1 or any(
                (f["width"], f["height"]) != self.dimensions for f in frames
            ):
                raise ValueError("Native frame dimensions or timestamp error exceeded its bound")
            result = {
                "state": "completed",
                "capture_id": self.capture_id,
                "dimensions": list(self.dimensions),
                "title": self.title,
                "first_input_wall_time": stamps[0],
                "input_frame_wall_times": stamps,
                "frame_count": len(frames),
                "maximum_timestamp_quantization_seconds": max(errors),
                "post_mux_close_read_ahead_packets": shutdown_read_ahead,
                "maximum_input_gap_seconds": max(b - a for a, b in zip(stamps, stamps[1:])),
                "sha256": hashlib.sha256(
                    (self.directory / "native-screen.mkv").read_bytes()
                ).hexdigest(),
                "source": "Actual X11 native application window; no reconstructed widgets",
            }
            write_json_atomic(self.directory / "recording.json", result)
            return result
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            if self.log is not None and not self.log.closed:
                self.log.close()
            write_json_atomic(
                self.directory / "recording.json",
                {
                    "state": "failed",
                    "capture_id": self.capture_id,
                    "error": str(error),
                },
            )
            raise
