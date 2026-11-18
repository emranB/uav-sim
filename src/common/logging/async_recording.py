"""Bounded optional recording separated from producers that must stay live."""

import json
import queue
import threading
from pathlib import Path


class AsyncRecording:
    """Accept records without waiting for serialization, storage or available queue space."""

    def __init__(self, path: Path, *, pending_limit: int = 64, byte_limit: int = 16 << 20) -> None:
        if pending_limit < 1 or byte_limit < 1:
            raise ValueError("Recording limits must be positive")
        self.path = path
        self.pending: queue.Queue[bytes | dict] = queue.Queue(maxsize=pending_limit)
        self.byte_limit = byte_limit
        self.stopping = threading.Event()
        self.dropped = 0
        self.written = 0
        self.error: str | None = None
        self.thread = threading.Thread(target=self._record, name="optional-recording", daemon=True)
        self.thread.start()

    def offer(self, record: bytes | dict) -> bool:
        """Drop an incoming record on overflow or permanent sink failure."""
        if self.error is None and not self.stopping.is_set():
            try:
                self.pending.put_nowait(record)
                return True
            except queue.Full:
                pass
        self.dropped += 1
        return False

    def _record(self) -> None:
        total = 0
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("xb") as stream:
                while not self.stopping.is_set() or not self.pending.empty():
                    try:
                        record = self.pending.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    encoded = (
                        record
                        if isinstance(record, bytes)
                        else (
                            json.dumps(record, allow_nan=False, separators=(",", ":")).encode()
                            + b"\n"
                        )
                    )
                    total += len(encoded)
                    if total > self.byte_limit:
                        raise OSError("Optional recording reached its byte limit")
                    stream.write(encoded)
                    self.written += 1
        except (OSError, ValueError) as error:
            self.error = str(error)

    def close(self, timeout_seconds: float = 0.5) -> dict:
        """Request orderly drain and bound how long the owner waits for the sink."""
        self.stopping.set()
        self.thread.join(timeout_seconds)
        return {
            "written": self.written,
            "dropped": self.dropped,
            "pending": self.pending.qsize(),
            "error": self.error,
            "writer_finished": not self.thread.is_alive(),
        }
