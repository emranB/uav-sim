"""Named mission and experiment persistence without arbitrary filesystem access."""

import hashlib
import json
import re
import time
import uuid
from pathlib import Path

from common.configuration.missions import parse_plan


class MissionStore:
    def __init__(self, directory: Path) -> None:
        self.directory = directory

    def path(self, name: str, kind: str = "missions") -> Path:
        if not isinstance(name, str) or not re.fullmatch(r"[\w][\w -]{0,63}", name, re.ASCII):
            raise ValueError("Use 1..64 letters, digits, spaces, underscores or hyphens for a name")
        folder = self.directory / kind / "named"
        folder.mkdir(parents=True, exist_ok=True)
        return folder / f"{name}.json"

    def list(self, kind: str = "missions") -> list[dict]:
        folder = self.directory / kind / "named"
        return [
            {"name": path.stem, "modified_utc_seconds": path.stat().st_mtime}
            for path in sorted(folder.glob("*.json"))[:200]
        ]

    def save(self, name: str, mission: dict) -> None:
        parse_plan(mission)
        destination = self.path(name)
        temporary = destination.with_suffix(".tmp")
        temporary.write_text(json.dumps(mission, indent=2) + "\n", encoding="utf-8")
        if destination.exists():
            self._preserve_version(destination)
        temporary.replace(destination)

    def load(self, name: str) -> dict:
        mission, _ = self.load_with_digest(name)
        return mission

    def load_with_digest(self, name: str) -> tuple[dict, str]:
        """Read one immutable file version and identify those exact validated bytes."""
        encoded = self.path(name).read_bytes()
        mission = json.loads(encoded)
        parse_plan(mission)
        return mission, hashlib.sha256(encoded).hexdigest()

    def archive(self, name: str, kind: str = "missions") -> None:
        path = self.path(name, kind)
        self._preserve_version(path)

    @staticmethod
    def _preserve_version(path: Path) -> None:
        # Wall-clock resolution varies by host; timestamps alone can collide.
        suffix = f".archived-{time.time_ns()}-{uuid.uuid4().hex}"
        path.rename(path.with_suffix(suffix))

    def save_experiment(self, name: str, configuration: dict) -> None:
        parse_plan(configuration["mission"])
        configuration = {"schema_version": 1, "saved_utc_seconds": time.time(), **configuration}
        destination = self.path(name, "experiments")
        temporary = destination.with_suffix(".tmp")
        temporary.write_text(json.dumps(configuration, indent=2) + "\n", encoding="utf-8")
        if destination.exists():
            self._preserve_version(destination)
        temporary.replace(destination)

    def load_experiment(self, name: str) -> dict:
        experiment = json.loads(self.path(name, "experiments").read_text(encoding="utf-8"))
        if experiment.get("schema_version") != 1:
            raise ValueError("Unsupported experiment schema")
        parse_plan(experiment["mission"])
        return experiment
