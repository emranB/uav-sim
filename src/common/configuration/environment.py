"""Acknowledged simulated-input controls and their experiment-visible state."""

import json
import threading
import time
from pathlib import Path

from common.interfaces.vehicles import VehicleAdapter


class EnvironmentControls:
    def __init__(
        self,
        vehicle: VehicleAdapter,
        directory: Path,
        runtime_directory: Path = Path("/runtime"),
        vehicle_id: str = "vehicle_1",
    ) -> None:
        self.runtime_directory = runtime_directory
        self.vehicle_id = vehicle_id
        self.vehicle = vehicle
        self.lock = threading.RLock()
        self.directory = directory
        self.gnss_enabled = True
        self.satellites = 10
        self.run_id = self._current_run()
        self.state_path = directory.parent / "environment-state.json"
        if self.state_path.exists():
            state = json.loads(self.state_path.read_text(encoding="utf-8"))
            if state.get("run_id") == self.run_id:
                self.gnss_enabled = state["gnss_enabled"]
                self.satellites = state["satellites"]
                link = state["link"]
                self.vehicle.configure_link(link["latency_ms"], link["loss_percent"], link["seed"])

    def _current_run(self) -> str:
        path = self.runtime_directory / "px4.run"
        return path.read_text(encoding="utf-8") if path.exists() else "unknown"

    def _synchronize_run(self) -> None:
        current = self._current_run()
        if current != self.run_id:
            self.run_id = current
            self.clear()

    def _save(self) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        state = {
            "run_id": self.run_id,
            "gnss_enabled": self.gnss_enabled,
            "satellites": self.satellites,
            "link": self.vehicle.snapshot().get("communication", {}),
        }
        temporary = self.state_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(state) + "\n", encoding="utf-8")
        temporary.replace(self.state_path)

    def snapshot(self) -> dict:
        with self.lock:
            return self._snapshot()

    def _snapshot(self) -> dict:
        self._synchronize_run()
        return {
            "gnss_enabled": self.gnss_enabled,
            "satellites": self.satellites,
            "link": self.vehicle.snapshot().get("communication", {}),
            "sensor_capabilities": {
                "gnss": "Live simulated GNSS fix control",
                "imu": "Measurements and health; runtime switching is unavailable",
                "magnetometer": "Measurements and health; runtime switching is unavailable",
                "barometer": "Measurements and health; runtime switching is unavailable",
                "camera": "The configured X500 has no camera pipeline",
                "lidar": "The configured X500 has no LiDAR pipeline",
                "visual_odometry": "Requires a camera, estimator and PX4 odometry bridge",
            },
        }

    def execute(self, operation: str, body: dict) -> dict:
        with self.lock:
            return self._execute(operation, body)

    def _execute(self, operation: str, body: dict) -> dict:
        self._synchronize_run()
        if operation == "sensors/gnss":
            enabled = body.get("enabled")
            if type(enabled) is not bool:
                raise ValueError("GNSS enabled must be true or false")
            self.vehicle.set_gnss_satellites(self.satellites if enabled else 0)
            self.gnss_enabled = enabled
        elif operation == "interference/gnss":
            satellites = body.get("satellites")
            if type(satellites) is not int or not 0 <= satellites <= 20:
                raise ValueError("Simulated satellites must be an integer in 0..20")
            self.vehicle.set_gnss_satellites(satellites if self.gnss_enabled else 0)
            self.satellites = satellites
        elif operation == "interference/link":
            self.vehicle.configure_link(
                body.get("latency_ms", 0), body.get("loss_percent", 0), body.get("seed", 0)
            )
        elif operation == "interference/clear":
            self.vehicle.configure_link(0, 0, 0)
            self.vehicle.set_gnss_satellites(10)
            self.gnss_enabled = True
            self.satellites = 10
        else:
            raise ValueError(f"Unsupported environment operation: {operation}")
        self._save()
        self.directory.mkdir(parents=True, exist_ok=True)
        with (self.directory / "environment.jsonl").open("a", encoding="utf-8") as recording:
            recording.write(
                json.dumps(
                    {
                        "timestamp": time.time(),
                        "vehicle_id": self.vehicle_id,
                        "operation": operation,
                        "settings": self.snapshot(),
                    }
                )
                + "\n"
            )
        return {"accepted": True, **self.snapshot()}

    def clear(self) -> None:
        """Restore the API link immediately; the new PX4 run restores sensor defaults."""
        with self.lock:
            self.vehicle.configure_link(0, 0, 0)
            self.gnss_enabled = True
            self.satellites = 10
            self._save()
