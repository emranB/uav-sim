"""Filesystem-only offline render worker; separate memory/CPU budget from live control."""

import json
import re
import threading
import time
import uuid
from collections.abc import Callable
from pathlib import Path

from common.logging.replay_export import verify_replay_export
from common.logging.storage import write_json_atomic


class FileReplayRenderer:
    """Submit one opaque export job to a network-free worker, with a renewable lease."""

    def __init__(self, jobs: Path, runtime: Path = Path("/runtime/replay-renderer")) -> None:
        self.jobs, self.runtime = jobs, runtime

    def availability(self) -> dict:
        try:
            status = self.read(self.runtime / "owner.json")
            age = time.time() - status["received_wall_time"]
            if not 0 <= age < 3:
                raise RuntimeError("Offline Blender worker heartbeat is stale")
            if status.get("state") == "running":
                return {"available": False, "reason": "Offline Blender worker is busy"}
            return status["renderer"]
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
            return {"available": False, "reason": f"Offline Blender worker unavailable: {error}"}

    def render(
        self,
        directory: Path,
        cancelled: threading.Event,
        check: Callable[[], None],
        progress: Callable[[float], None],
    ) -> None:
        if directory.resolve().parent != self.jobs.resolve() or not re.fullmatch(
            r"job-[a-f0-9]{32}", directory.name
        ):
            raise ValueError("Offline render job is outside the declared export store")
        available = self.availability()
        if not available["available"]:
            raise RuntimeError(available["reason"])
        verify_replay_export(directory)
        token = uuid.uuid4().hex
        request = {"job_id": directory.name, "token": token}
        write_json_atomic(
            self.runtime / "lease.json", {**request, "received_wall_time": time.time()}
        )
        write_json_atomic(self.runtime / "request.json", request)
        status_path = directory / "renderer-status.json"
        terminal = False
        try:
            while True:
                check()
                write_json_atomic(
                    self.runtime / "lease.json",
                    {
                        **request,
                        "received_wall_time": time.time(),
                    },
                )
                owner = self.read(self.runtime / "owner.json")
                if not 0 <= time.time() - owner["received_wall_time"] < 3:
                    raise RuntimeError("Offline Blender worker stopped reporting")
                if status_path.exists():
                    status = self.read(status_path)
                    if status.get("token") == token:
                        progress(status.get("progress", 0))
                        if status["state"] in {"completed", "error", "cancelled"}:
                            terminal = True
                            if status["state"] != "completed":
                                raise RuntimeError(status.get("error", "Offline worker failed"))
                            return
                cancelled.wait(0.1)
        finally:
            if not terminal:
                write_json_atomic(self.runtime / "cancel.json", request)
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    if status_path.exists():
                        status = self.read(status_path)
                        if status.get("token") == token and status["state"] in {
                            "completed",
                            "error",
                            "cancelled",
                        }:
                            break
                    time.sleep(0.05)
                else:
                    raise TimeoutError("Offline worker did not acknowledge cancellation")

    @staticmethod
    def read(path: Path) -> dict:
        if path.is_symlink() or path.stat().st_size > 4096:
            raise ValueError("Offline worker metadata exceeds its bounded contract")
        payload = json.loads(path.read_text())
        if not isinstance(payload, dict):
            raise ValueError("Offline worker metadata must be an object")
        return payload
