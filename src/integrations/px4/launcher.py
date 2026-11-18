"""Launch PX4 SITL against an independently running Gazebo server."""

import os
import re
import subprocess
import time
from pathlib import Path

from common.configuration.fleet import load_fleet
from common.configuration.world import WorldLocation
from common.lifecycle.process import run_process
from integrations.px4.startup import prepare_startup_script


def model_exists(model_name: str) -> bool:
    """Query actual scene entities; camera subscribers can advertise nonexistent topics."""
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/world/default/scene/info",
            "--reqtype",
            "gz.msgs.Empty",
            "--reptype",
            "gz.msgs.Scene",
            "--timeout",
            "2000",
            "--req",
            "",
        ],
        capture_output=True,
        text=True,
        timeout=4,
        check=False,
    )
    return (
        result.returncode == 0
        and re.search(rf'^\s*name:\s*"{re.escape(model_name)}"\s*$', result.stdout, re.MULTILINE)
        is not None
    )


def main() -> int:
    vehicle_id = os.environ.get("UAV_VEHICLE_ID", "vehicle_1")
    os.environ["UAV_VEHICLE_ID"] = vehicle_id
    vehicle = next(vehicle for vehicle in load_fleet() if vehicle.vehicle_id == vehicle_id)
    root = vehicle.output_directory(Path("/workspace/outputs"))
    os.environ["OUTPUT_DIR"] = str(root)
    log_mode = os.environ.get("UAV_PX4_LOG_MODE", "armed")
    if log_mode not in {"armed", "boot"}:
        raise ValueError(f"{vehicle_id}: PX4 log mode must be armed or boot")
    directory = root / "px4"
    directory.mkdir(parents=True, exist_ok=True)
    location = WorldLocation.configured()
    os.environ.update(
        {
            "PX4_GZ_STANDALONE": "1",
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_SIM_MODEL": "gz_x500",
            "PX4_GZ_WORLD": "default",
            "PX4_GZ_MODELS": "/opt/px4/Tools/simulation/gz/models",
            "PX4_GZ_WORLDS": "/opt/px4/Tools/simulation/gz/worlds",
            "PX4_UXRCE_DDS_NS": vehicle.vehicle_id,
            "PX4_GZ_MODEL_POSE": ",".join(str(value) for value in vehicle.spawn_enu_m),
            "PX4_HOME_LAT": str(location.latitude_deg),
            "PX4_HOME_LON": str(location.longitude_deg),
            "PX4_HOME_ALT": str(location.elevation_m),
        }
    )
    if os.environ.get("UAV_STEREO", "false") == "true":
        os.environ["PX4_GZ_MODELS"] = "/runtime/stereo-models"
    vehicle.runtime_directory.mkdir(parents=True, exist_ok=True)
    restart_file = vehicle.runtime_directory / "px4.restart"
    restart_file.unlink(missing_ok=True)
    if model_exists(vehicle.model_name):
        os.environ["PX4_GZ_MODEL_NAME"] = vehicle.model_name
    while True:
        run_directory = directory / f"run-{time.time_ns()}"
        run_directory.mkdir()
        marker = vehicle.runtime_directory / "px4.run"
        temporary = marker.with_suffix(".tmp")
        temporary.write_text(run_directory.name, encoding="utf-8")
        temporary.replace(marker)
        os.chdir(run_directory)
        startup_script = prepare_startup_script(
            Path("/opt/px4/build/px4_sitl_default/etc/init.d-posix/rcS"),
            run_directory,
            log_mode,
            dds_time_source=os.environ.get("UAV_DDS_TIME_SOURCE", "simulation"),
        )
        code = run_process(
            [
                "/opt/px4/build/px4_sitl_default/bin/px4",
                "-i",
                str(vehicle.instance),
                "-d",
                "-s",
                str(startup_script),
                "/opt/px4/build/px4_sitl_default/etc",
            ],
            restart_file=restart_file,
            log_source="px4",
            own_process_group=False,
        )
        if code != 75:
            return code
        os.environ["PX4_GZ_MODEL_NAME"] = vehicle.model_name


if __name__ == "__main__":
    raise SystemExit(main())
