"""Acknowledge PX4 process generations independently of cached MAVLink data."""

import json
import time
from pathlib import Path


class PX4Runtime:
    def __init__(self, directory: Path = Path("/runtime")) -> None:
        self.directory = directory

    def generation(self) -> str:
        path = self.directory / "px4.run"
        return path.read_text(encoding="utf-8").strip() if path.exists() else ""

    def restart(self) -> None:
        (self.directory / "px4.restart").touch()

    def wait_stopped(self, timeout_seconds: float = 25) -> None:
        path = self.directory / "process.json"
        deadline = time.monotonic() + timeout_seconds
        while True:
            if path.exists() and not json.loads(path.read_text(encoding="utf-8"))["running"]:
                return
            if time.monotonic() >= deadline:
                raise TimeoutError("PX4 supervisor did not confirm process shutdown")
            time.sleep(0.2)
