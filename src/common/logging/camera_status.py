"""Read camera-service evidence without trusting arbitrary filesystem paths."""

import json
import time
from pathlib import Path


class CameraStatus:
    def __init__(self, root: Path, vehicle_id: str, runtime_root: Path = Path("/runtime")) -> None:
        self.root = root.resolve()
        self.vehicle_id = vehicle_id
        self.recording_root = (
            self.root / "perception"
            if vehicle_id == "vehicle_1"
            else self.root / "vehicles" / vehicle_id / "perception"
        )
        directory = runtime_root if vehicle_id == "vehicle_1" else runtime_root / vehicle_id
        self.path = directory / "stereo.json"
        self._directory_cache: tuple[tuple[str, str | None], Path] | None = None

    def snapshot(self) -> dict:
        """Readiness needs fresh frame reception, not merely a running recorder."""
        try:
            with self.path.open("rb") as stream:
                encoded = stream.read(8193)
            if len(encoded) > 8192:
                raise ValueError("Camera status exceeds its size limit")
            status = json.loads(encoded)
            if status.get("vehicle_id") != self.vehicle_id or status.get("schema_version") != 1:
                raise ValueError("Camera status identity or schema mismatch")
            generation_file = self.path.parent / "px4.run"
            if generation_file.exists() and status.get("controller_run_id") != (
                generation_file.read_text(encoding="utf-8").strip()
            ):
                raise ValueError("Camera images belong to a previous controller generation")
            directory = self._directory(status)
            age = time.time() - float(status["received_wall_time"])
            status["ready"] = 0 <= age < 3 and not status.get("error")
            status["age_seconds"] = max(0, age)
            status["recording"] = (directory / "report.json").relative_to(self.root).as_posix()
            return status
        except (OSError, ValueError, KeyError, TypeError) as error:
            return {"ready": False, "vehicle_id": self.vehicle_id, "error": str(error)}

    def _directory(self, status: dict) -> Path:
        name = status.get("directory")
        if not isinstance(name, str):
            raise ValueError("Camera status has no recording directory")
        key = (name, status.get("controller_run_id"))
        cached = self._directory_cache
        if cached is not None and cached[0] == key:
            return cached[1]
        directory = (self.root / name).resolve()
        if not directory.is_relative_to(self.recording_root):
            raise ValueError("Camera recording is outside the selected vehicle's directory")
        # One immutable cache entry avoids repeated host-mount stat calls in the live poller.
        self._directory_cache = (key, directory)
        return directory

    def image(self, side: str) -> bytes:
        if side not in {"left", "right"}:
            raise ValueError("Camera side must be left or right")
        status = self.snapshot()
        if not status["ready"]:
            raise RuntimeError(f"{self.vehicle_id}: camera stream is stale or unavailable")
        directory = (
            self.path.parent / "stereo-preview"
            if status.get("runtime_previews")
            else self._directory(status)
        )
        path = (directory / f"latest-{side}.ppm").resolve()
        allowed_root = self.path.parent.resolve() if status.get("runtime_previews") else self.root
        if not path.is_relative_to(allowed_root):
            raise ValueError("Camera image is outside the output directory")
        with path.open("rb") as stream:
            image = stream.read(1024 * 1024 + 1)
        if len(image) > 1024 * 1024:
            raise ValueError("Camera preview exceeds 1 MiB")
        return image
