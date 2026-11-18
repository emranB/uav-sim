"""Single-frame in-memory exchange with the native stereo pose process."""

import json
import math
import os
import select
import signal
import struct
import subprocess
import threading
import time
from pathlib import Path
from types import TracebackType

from common.logging.async_recording import AsyncRecording

REQUEST = struct.Struct("<QII")
RESPONSE = struct.Struct("<I")
MAX_IMAGE_BYTES = 1 << 20
MAX_POSE_BYTES = 64 << 10


class NativePosePipe:
    """Send one stereo capture and receive its pose without touching storage."""

    def __init__(
        self,
        configuration: Path,
        diagnostic_path: Path,
        executable: str = "orb-recording",
    ) -> None:
        self.process = subprocess.Popen(
            [
                executable,
                "/opt/orb-slam3/Vocabulary/ORBvoc.txt",
                str(configuration),
                "-",
                ".",
                "--pipe",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            start_new_session=True,
        )
        self.diagnostics = AsyncRecording(diagnostic_path)
        self.drain = threading.Thread(target=self._drain_diagnostics, daemon=True)
        self.drain.start()
        self.lock = threading.Lock()
        self.started = False
        self.failed = False
        assert self.process.stdin is not None and self.process.stdout is not None
        self.input_fd = self.process.stdin.fileno()
        self.output_fd = self.process.stdout.fileno()
        os.set_blocking(self.input_fd, False)
        os.set_blocking(self.output_fd, False)

    def __enter__(self) -> "NativePosePipe":
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def _drain_diagnostics(self) -> None:
        assert self.process.stderr is not None
        while chunk := self.process.stderr.read(4096):
            self.diagnostics.offer(chunk)

    def _wait(self, fd: int, writing: bool, deadline: float, stopping: threading.Event) -> bool:
        while not stopping.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Native pose pipe exceeded its frame deadline")
            readable, writable, _ = select.select(
                [] if writing else [fd],
                [fd] if writing else [],
                [],
                min(remaining, 0.02),
            )
            if readable or writable:
                return True
        return False

    def _write(self, payload: bytes, deadline: float, stopping: threading.Event) -> bool:
        remaining = memoryview(payload)
        while remaining:
            if not self._wait(self.input_fd, True, deadline, stopping):
                return False
            try:
                count = os.write(self.input_fd, remaining)
            except BlockingIOError:
                continue
            except BrokenPipeError as error:
                raise RuntimeError("Native estimator closed its request pipe") from error
            remaining = remaining[count:]
        return True

    def _read(self, size: int, deadline: float, stopping: threading.Event) -> bytes | None:
        received = bytearray()
        while len(received) < size:
            if not self._wait(self.output_fd, False, deadline, stopping):
                return None
            try:
                chunk = os.read(self.output_fd, size - len(received))
            except BlockingIOError:
                continue
            if not chunk:
                raise RuntimeError("Native estimator returned EOF or a truncated pose response")
            received.extend(chunk)
        return bytes(received)

    def estimate(
        self,
        images: list[bytes],
        timestamp_ns: int,
        stopping: threading.Event,
        *,
        timeout_seconds: float = 10,
    ) -> dict | None:
        """Exchange one matched frame or invalidate the stream on interruption."""
        if (
            len(images) != 2
            or any(not 0 < len(image) <= MAX_IMAGE_BYTES for image in images)
            or not 0 <= timestamp_ns < 1 << 64
            or not math.isfinite(timeout_seconds)
            or timeout_seconds <= 0
        ):
            raise ValueError("Invalid bounded stereo pose request")
        if not self.lock.acquire(blocking=False):
            raise RuntimeError("Native pose pipe already has a frame in flight")
        try:
            if self.failed:
                raise RuntimeError("Native pose pipe must restart after a failed exchange")
            deadline = time.monotonic() + timeout_seconds
            # An interrupted request may have left either pipe partially consumed.
            self.failed = True
            header = REQUEST.pack(timestamp_ns, len(images[0]), len(images[1]))
            for part in (header, *images):
                if not self._write(part, deadline, stopping):
                    return None
            response = self._read(RESPONSE.size, deadline, stopping)
            if response is None:
                return None
            (length,) = RESPONSE.unpack(response)
            if not 0 < length <= MAX_POSE_BYTES:
                raise RuntimeError("Native estimator returned an invalid pose length")
            payload = self._read(length, deadline, stopping)
            if payload is None:
                return None
            pose = json.loads(payload)
            stamp = pose.get("timestamp_seconds") if isinstance(pose, dict) else None
            if type(stamp) not in {int, float} or not math.isclose(
                stamp, timestamp_ns / 1e9, rel_tol=0, abs_tol=1e-7
            ):
                raise RuntimeError("Native pose response timestamp does not match the capture")
            self.failed = False
            self.started = True
            return pose
        finally:
            self.lock.release()

    def close(self) -> dict:
        """Close input for native EOF, escalating only if the process fails to exit."""
        assert self.process.stdin and self.process.stdout and self.process.stderr
        self.process.stdin.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=1)
        self.drain.join(timeout=0.5)
        self.process.stdout.close()
        self.process.stderr.close()
        return self.diagnostics.close()
