"""Bound idle telemetry without deleting flight or fault evidence."""

import os
import time
from collections.abc import Callable
from pathlib import Path

from common.logging.storage import BoundedRecording, write_json_atomic


class TelemetryRecording:
    """Two recent idle segments, protected flight prefixes and latched write failures."""

    def __init__(
        self,
        directory: Path,
        vehicle_id: str,
        controller_run_id: str,
        *,
        idle_segment_bytes: int | None = None,
        idle_segments: int | None = None,
        writer_factory: Callable[..., BoundedRecording] = BoundedRecording,
    ) -> None:
        self.directory = directory
        self.vehicle_id = vehicle_id
        self.controller_run_id = controller_run_id
        self.idle_segment_bytes = (
            idle_segment_bytes
            if idle_segment_bytes is not None
            else int(os.environ.get("UAV_TELEMETRY_IDLE_SEGMENT_BYTES", 16 * 1024**2))
        )
        self.idle_segments = (
            idle_segments
            if idle_segments is not None
            else int(os.environ.get("UAV_TELEMETRY_IDLE_SEGMENTS", 2))
        )
        if not 1 <= self.idle_segments <= 8 or not 1 <= self.idle_segment_bytes <= 256 * 1024**2:
            raise ValueError("Telemetry idle retention requires positive segments and bytes")
        self._writer_factory = writer_factory
        self.generation = time.time_ns()
        self.manifest_path = directory / f"telemetry-{self.generation}.manifest.json"
        self.segments: list[dict] = []
        self.counter = 0
        self.armed = False
        self._error: str | None = None
        self._writer: BoundedRecording | None = None
        self.closed = False
        self.idle_bytes_removed = 0
        self._writer = self._new_writer("idle")

    @property
    def error(self) -> str | None:
        """Return a latched policy/storage failure, including an in-progress writer failure."""
        return self._error or (self._writer.error if self._writer is not None else None)

    def _new_writer(self, kind: str) -> BoundedRecording:
        self.counter += 1
        path = self.directory / f"telemetry-{self.generation}-{self.counter:04d}-{kind}.jsonl"
        maximum = self.idle_segment_bytes if kind == "idle" else 256 * 1024**2
        self.segments.append({"path": path.name, "kind": kind, "bytes": 0})
        self._publish()
        return self._writer_factory(path, maximum_bytes=maximum)

    def _publish(self) -> None:
        try:
            write_json_atomic(
                self.manifest_path,
                {
                    "schema_version": 1,
                    "vehicle_id": self.vehicle_id,
                    "controller_run_id": self.controller_run_id,
                    "idle_segment_bytes": self.idle_segment_bytes,
                    "idle_segments": self.idle_segments,
                    "flight_and_prefix_segments_retained": True,
                    "idle_bytes_removed": self.idle_bytes_removed,
                    "closed": self.closed,
                    "segments": self.segments,
                    "error": self.error,
                },
            )
        except OSError as error:
            self._error = self.error or f"{self.vehicle_id}: telemetry manifest failed: {error}"

    def _finish(self) -> None:
        self._writer.close()
        self._error = self.error or self._writer.error
        self.segments[-1]["bytes"] = self._writer.bytes_written

    def observe_armed(self, armed: bool) -> None:
        """Protect pre-arm context and record each flight in an unpruned segment."""
        if armed == self.armed or self.error is not None:
            return
        self._finish()
        if armed:
            for segment in self.segments:
                if segment["kind"] == "idle":
                    segment["kind"] = "flight-prefix"
        self.armed = armed
        self._writer = self._new_writer("flight" if armed else "idle")

    def write(self, text: str) -> bool:
        """Rotate idle history intentionally; storage errors stay latched across rotation."""
        if self.error is not None:
            return False
        size = len(text.encode("utf-8"))
        if not self.armed and self._writer.bytes_written + size > self.idle_segment_bytes:
            self._finish()
            if self.error is not None:
                return False
            idle = [segment for segment in self.segments if segment["kind"] == "idle"]
            while len(idle) >= self.idle_segments:
                oldest = idle.pop(0)
                try:
                    path = self.directory / oldest["path"]
                    path.unlink(missing_ok=True)
                except OSError as error:
                    self._error = f"{self.vehicle_id}: idle retention failed: {error}"
                    self._publish()
                    return False
                self.idle_bytes_removed += oldest["bytes"]
                self.segments.remove(oldest)
            self._writer = self._new_writer("idle")
        if len(self.segments) > 128:
            self._error = (
                f"{self.vehicle_id}: telemetry checkpoint required after 128 protected segments"
            )
            self._publish()
            return False
        accepted = self._writer.write(text)
        self._error = self.error or self._writer.error
        return accepted

    def snapshot(self) -> dict:
        """Expose bounded idle policy separately from recording health."""
        result = self._writer.snapshot()
        return {
            **result,
            "available": self.error is None and result["available"],
            "error": self.error or result["error"],
            "manifest": str(self.manifest_path),
            "idle_segment_bytes": self.idle_segment_bytes,
            "idle_segments": self.idle_segments,
            "idle_bytes_removed": self.idle_bytes_removed,
        }

    def close(self) -> None:
        self._finish()
        self.closed = True
        self._publish()
