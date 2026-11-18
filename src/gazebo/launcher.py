"""Independent Gazebo server and viewer entry points."""

import argparse
import logging
import os
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from common.configuration.fleet import load_fleet
from common.configuration.runtime import RuntimeSettings
from common.configuration.world import WorldLocation
from common.lifecycle.process import run_process
from gazebo.navigation_reference import add_launch_platforms, add_navigation_references
from gazebo.navigation_scene import add_navigation_obstacles
from gazebo.stereo_model import prepare_stereo_model
from gazebo.world import prepare_world


def follow_vehicle() -> None:
    """Keep the simulated drone in view while leaving camera control available."""
    try:
        _request_camera_follow()
    except subprocess.TimeoutExpired:
        logging.warning("Gazebo camera follow timed out; use manual camera controls")


def _request_camera_follow() -> None:
    for attempt in range(8):
        result = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                "/gui/follow",
                "--reqtype",
                "gz.msgs.StringMsg",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "1000",
                "--req",
                'data: "x500_0"',
            ],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        if result.returncode == 0 and "data: true" in result.stdout:
            subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    "/gui/follow/offset",
                    "--reqtype",
                    "gz.msgs.Vector3d",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "1000",
                    "--req",
                    "x: -2.5 y: -2.5 z: 1.8",
                ],
                capture_output=True,
                timeout=3,
                check=False,
            )
            return
        time.sleep(1)
    logging.warning("Gazebo camera follow was not acknowledged; use manual camera controls")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=["server", "viewer"])
    arguments = parser.parse_args()
    settings = RuntimeSettings.from_environment(os.environ)
    os.environ.update(settings.rendering_environment(os.environ))
    if arguments.role == "server":
        stereo_enabled = os.environ.get("UAV_STEREO", "false") == "true"
        if stereo_enabled:
            prepare_stereo_model(
                Path("/opt/px4/Tools/simulation/gz/models/x500/model.sdf"),
                Path("/runtime/stereo-models"),
            )
            os.environ["GZ_SIM_RESOURCE_PATH"] = "/runtime/stereo-models:" + os.environ.get(
                "GZ_SIM_RESOURCE_PATH", ""
            )
        location = WorldLocation.configured()
        polish = os.environ.get("UAV_VISUAL_POLISH", "false")
        if polish not in {"true", "false"}:
            raise ValueError("UAV_VISUAL_POLISH must be true or false")
        world = prepare_world(
            Path("/opt/px4/Tools/simulation/gz/worlds/default.sdf"),
            settings.output_directory / "worlds/default.sdf",
            location,
            scene_name=os.environ.get("UAV_SCENE", "default"),
            stereo_enabled=stereo_enabled,
            terrain_directory=Path(os.environ["UAV_TERRAIN_DIR"])
            if "UAV_TERRAIN_DIR" in os.environ
            else None,
            shadows_enabled=settings.render_mode == "gpu",
            real_time_factor=settings.real_time_factor,
            visual_polish=polish == "true",
            buildings_directory=Path(os.environ["UAV_WATERFRONT_BUILDINGS_DIR"])
            if os.environ.get("UAV_WATERFRONT_BUILDINGS_DIR")
            else None,
        )
        if os.environ.get("UAV_NAVIGATION", "false") == "true":
            tree = ET.parse(world)
            add_navigation_obstacles(tree.find("world"), Path(os.environ["UAV_AVOIDANCE_SCENARIO"]))
            add_navigation_references(tree.find("world"), load_fleet())
            platforms = os.environ.get("UAV_LAUNCH_PLATFORMS", "false")
            if platforms not in {"true", "false"}:
                raise ValueError("Launch platforms must be explicitly true or false")
            if platforms == "true":
                add_launch_platforms(tree.find("world"), load_fleet())
            tree.write(world, encoding="utf-8", xml_declaration=True)
        os.environ.setdefault(
            "GZ_SIM_SERVER_CONFIG_PATH", "/workspace/configs/worlds/server.config"
        )
        command = [
            "gz",
            "sim",
            "-s",
            "-r",
            "-v",
            "2",
            str(world),
        ]
        if stereo_enabled:
            command.insert(3, "--headless-rendering")
        generation = Path("/runtime/gazebo.epoch")
        temporary = generation.with_suffix(".tmp")
        temporary.write_text(uuid.uuid4().hex, encoding="utf-8")
        temporary.replace(generation)
        return run_process(command, log_source="gazebo")
    if settings.headless:
        return 0
    # Match the Qt 6 host's device-independent sizing when Gazebo uses Qt 5.
    os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"
    os.environ["QT_SCALE_FACTOR_ROUNDING_POLICY"] = "PassThrough"
    command = [
        "gz",
        "sim",
        "-g",
        "-v",
        "2",
        "--gui-config",
        "/workspace/configs/worlds/viewer.config",
    ]
    while True:
        result = run_process(
            command,
            window_name="Gazebo",
            ready_callback=follow_vehicle,
            log_source="gazebo-viewer",
            generation_file=Path("/runtime/gazebo.epoch"),
        )
        if result != 75:
            return result


if __name__ == "__main__":
    raise SystemExit(main())
