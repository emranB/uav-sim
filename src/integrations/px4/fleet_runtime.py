"""Supervise configured SITL processes without access to the container engine."""

import json
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path

from common.configuration.fleet import VehicleDefinition, load_fleet


def publish_process(
    vehicle: VehicleDefinition, running: bool, exit_code: int | None = None
) -> None:
    vehicle.runtime_directory.mkdir(parents=True, exist_ok=True)
    path = vehicle.runtime_directory / "process.json"
    temporary = path.with_suffix(".tmp")
    temporary.write_text(
        json.dumps({"running": running, "exit_code": exit_code}) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def stop_process(vehicle: VehicleDefinition, process: subprocess.Popen) -> None:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    publish_process(vehicle, False, process.returncode)


def main() -> int:
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda signum, frame: stopping.set())
    signal.signal(signal.SIGINT, lambda signum, frame: stopping.set())
    primary = os.environ.get("UAV_INSTANCE_GROUP", "secondary") == "primary"
    processes: dict[str, tuple[VehicleDefinition, subprocess.Popen]] = {}
    generation_file = Path("/runtime/gazebo.epoch")
    generation = generation_file.read_text() if generation_file.exists() else ""
    try:
        while not stopping.is_set():
            current_generation = generation_file.read_text() if generation_file.exists() else ""
            if current_generation != generation:
                for vehicle, process in processes.values():
                    stop_process(vehicle, process)
                processes.clear()
                generation = current_generation
            desired = {
                vehicle.vehicle_id: vehicle
                for vehicle in load_fleet()
                if vehicle.enabled and (vehicle.instance == 0) == primary
            }
            for identifier in list(processes):
                vehicle, process = processes[identifier]
                if identifier not in desired or process.poll() is not None:
                    stop_process(vehicle, process)
                    del processes[identifier]
            for identifier, vehicle in desired.items():
                if identifier not in processes:
                    environment = {**os.environ, "UAV_VEHICLE_ID": identifier}
                    process = subprocess.Popen(
                        [sys.executable, "-m", "integrations.px4.launcher"],
                        env=environment,
                        start_new_session=True,
                    )
                    processes[identifier] = (vehicle, process)
                    publish_process(vehicle, True)
            stopping.wait(1)
        return 0
    finally:
        for vehicle, process in processes.values():
            stop_process(vehicle, process)


if __name__ == "__main__":
    raise SystemExit(main())
