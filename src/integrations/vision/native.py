"""Own the native stereo estimator process and its single-request exchange."""

import json
import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from types import TracebackType


class NativeStereoEstimator:
    """Keep file exchange, cancellation and process cleanup out of map orchestration."""

    def __init__(self, work: Path, log_path: Path) -> None:
        self.work = work
        self.log_path = log_path
        self.started = False
        self.log = log_path.open("w", encoding="utf-8")
        try:
            self.process = subprocess.Popen(
                [
                    "orb-recording",
                    "/opt/orb-slam3/Vocabulary/ORBvoc.txt",
                    str(work / "camera.yaml"),
                    "-",
                    str(work),
                    "--live",
                ],
                stdin=subprocess.PIPE,
                stdout=self.log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except OSError:
            self.log.close()
            raise

    def __enter__(self) -> "NativeStereoEstimator":
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def estimate(
        self,
        images: list[bytes],
        timestamp_ns: int,
        stopping: threading.Event,
        *,
        timeout_seconds: float | None = None,
    ) -> dict | None:
        """Return a completed snapshot, or None when the caller cancels."""
        if self.process.poll() is not None:
            raise RuntimeError(f"Estimator exited; inspect {self.log_path}")
        if len(images) != 2 or any(len(image) > 1024 * 1024 for image in images):
            raise ValueError("Estimator requires two bounded stereo images")
        for side, contents in zip(("left", "right"), images):
            (self.work / f"{side}.ppm").write_bytes(contents)
        started = time.monotonic()
        self.process.stdin.write(
            json.dumps(
                {
                    "timestamp_seconds": timestamp_ns / 1e9,
                    "left": str(self.work / "left.ppm"),
                    "right": str(self.work / "right.ppm"),
                }
            )
            + "\n"
        )
        self.process.stdin.flush()
        native_path = self.work / "snapshot.json"
        last_revision = None
        if timeout_seconds is None:
            timeout_seconds = 30 if self.started else 120
        while not stopping.wait(0.005):
            if self.process.poll() is not None:
                raise RuntimeError(f"Estimator exited; inspect {self.log_path}")
            if time.monotonic() - started > timeout_seconds:
                raise TimeoutError(
                    f"Estimator frame exceeded its {timeout_seconds} second deadline"
                )
            try:
                status = native_path.stat()
            except FileNotFoundError:
                continue
            revision = (status.st_ino, status.st_mtime_ns, status.st_size)
            # The native writer atomically replaces immutable snapshots. Poll
            # promptly without repeatedly decoding the previous frame's map.
            if revision != last_revision:
                last_revision = revision
                with native_path.open("rb") as stream:
                    contents = stream.read(3 * 1024 * 1024 + 1)
                if len(contents) > 3 * 1024 * 1024:
                    raise ValueError("Native snapshot exceeds its point budget")
                candidate = json.loads(contents)
                if abs(candidate["timestamp_seconds"] - timestamp_ns / 1e9) < 1e-7:
                    self.started = True
                    return candidate
        return None

    def close(self) -> None:
        """Try orderly EOF shutdown, then bound termination of the owned process group."""
        try:
            if self.process.stdin is not None:
                try:
                    self.process.stdin.close()
                except BrokenPipeError:
                    logging.warning("Native estimator closed its input early")
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=2)
        finally:
            self.log.close()
