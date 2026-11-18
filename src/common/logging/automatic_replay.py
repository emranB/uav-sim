"""Idempotent automatic rendering of finalized immutable replay captures."""

import json
import threading
import time
from typing import TYPE_CHECKING

from common.logging.storage import write_json_atomic

if TYPE_CHECKING:
    from common.logging.replay_export import ReplayExportJobs
    from common.logging.replay_store import ReplayCaptureStore


class AutomaticReplayRenders:
    """Serialize a bounded render queue without accessing any flight component.

    A capture ID owns one automatic attempt, including failed and cancelled attempts.
    Explicit interval renders remain available for deliberate retries. A restart marks
    unfinished attempts interrupted rather than silently duplicating a render.
    """

    maximum_pending = 4
    maximum_records = 4096

    def __init__(self, store: "ReplayCaptureStore", jobs: "ReplayExportJobs") -> None:
        self.store = store
        self.jobs = jobs
        self.path = jobs.directory / "automatic-renders.json"
        self._lock = threading.RLock()
        self._stopping = threading.Event()
        if self.path.is_symlink() or (
            self.path.exists() and self.path.stat().st_size > 4 * 1024**2
        ):
            raise ValueError("Automatic replay metadata exceeds its bounded contract")
        self._records = json.loads(self.path.read_text()) if self.path.exists() else {}
        if not isinstance(self._records, dict) or len(self._records) > self.maximum_records:
            raise ValueError("Automatic replay queue metadata exceeds its bounded contract")
        for record in self._records.values():
            if record["state"] in {"queued", "rendering"}:
                record.update(state="interrupted", error="Render queue owner restarted")
        self._current: str | None = None
        self._publish()
        self._worker = threading.Thread(target=self._run, name="automatic-replay", daemon=True)
        self._worker.start()

    def enqueue(self, capture_id: str, title: str = "Recorded 3D mission replay") -> dict:
        """Pin one sealed capture and acknowledge an existing attempt on repeat calls."""
        if not isinstance(title, str) or len(title) > 120 or any(ord(c) < 32 for c in title):
            raise ValueError("Replay title must contain at most 120 printable characters")
        with self._lock:
            if capture_id in self._records:
                return {"accepted": True, "duplicate": True, **self._records[capture_id]}
            if self._stopping.is_set():
                raise RuntimeError("Automatic replay queue is shutting down")
            if len(self._records) >= self.maximum_records:
                raise RuntimeError("Automatic replay history is full; existing records retained")
            if (
                sum(r["state"] in {"queued", "rendering"} for r in self._records.values())
                >= self.maximum_pending
            ):
                raise RuntimeError("Automatic replay queue is full (four captures)")
            manifest = self.store.manifest(capture_id)
            if manifest["state"] != "stopped" or manifest.get("duration_wall_seconds", 0) <= 0:
                raise ValueError("Automatic render requires a finalized positive capture interval")
            self.store.keep(capture_id)
            record = {
                "capture_id": capture_id,
                "state": "queued",
                "progress": 0.0,
                "created_wall_time": time.time(),
                "title": title,
                "error": None,
            }
            self._records[capture_id] = record
            self._publish()
            return {"accepted": True, **record}

    def snapshot(self) -> dict:
        """Expose actual queue and worker acknowledgements, including terminal failures."""
        with self._lock:
            record = self._records.get(self._current) if self._current else None
            if record is None and self._records:
                record = next(reversed(self._records.values()))
            return {
                **(record or {"state": "idle"}),
                "queued_count": sum(r["state"] == "queued" for r in self._records.values()),
            }

    def cancel(self) -> dict:
        """Cancel queued work or the owned active render, preserving the capture."""
        with self._lock:
            record = self._records.get(self._current) if self._current else None
            queued = [r for r in self._records.values() if r["state"] == "queued"]
            if record is None and not queued:
                raise RuntimeError("No automatic replay job is active")
            for item in queued:
                item.update(state="cancelled", error="Queued render cancelled")
            if record and record["state"] == "rendering":
                try:
                    self.jobs.cancel()
                except RuntimeError:
                    result = self.jobs.snapshot()
                    if result.get("state") in {"queued", "running", "rendering"}:
                        raise
                    record.update(result)
            self._publish()
            return {"accepted": True}

    def close(self) -> None:
        """Stop queue admission and cancel its reader without changing flight state."""
        self._stopping.set()
        with self._lock:
            if self._current or any(r["state"] == "queued" for r in self._records.values()):
                self.cancel()
        self._worker.join(10)
        if self._worker.is_alive():
            raise TimeoutError("Automatic replay queue did not acknowledge shutdown")

    def _publish(self) -> None:
        write_json_atomic(self.path, self._records)

    def _run(self) -> None:
        while not self._stopping.wait(0.1):
            with self._lock:
                if self.jobs.snapshot().get("state") in {"queued", "running", "rendering"}:
                    continue
                pending = next((r for r in self._records.values() if r["state"] == "queued"), None)
                if pending is None:
                    continue
                available = self.jobs.availability()
                if (
                    not available.get("available")
                    and available.get("reason") == "Offline Blender worker is busy"
                ):
                    if time.time() - pending["created_wall_time"] < 7200:
                        continue
                self._current = pending["capture_id"]
                try:
                    admitted = self.jobs.start(
                        {"capture_id": self._current, "title": pending["title"]},
                        render=True,
                    )
                    pending.update(
                        state="rendering", job_id=admitted["job_id"], path=admitted["path"]
                    )
                except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
                    if self.jobs.snapshot().get("state") in {"queued", "running", "rendering"}:
                        pending.update(state="queued", error=None)
                    else:
                        pending.update(state="failed", error=str(error))
                    self._current = None
                self._publish()
            if self._current is None:
                continue
            while True:
                result = self.jobs.snapshot()
                with self._lock:
                    if result["state"] not in {"queued", "running", "rendering"}:
                        pending.update(result)
                        if pending["state"] == "error":
                            pending["state"] = "failed"
                        self._current = None
                        self._publish()
                        break
                    pending["progress"] = result.get("progress", 0)
                self._stopping.wait(0.1)
