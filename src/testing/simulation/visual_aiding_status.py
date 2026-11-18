"""Record PX4 source flags and publish bounded readiness for the isolated SITL demo."""

import argparse
import json
import os
import re
import socket
import subprocess
import time
from pathlib import Path

from common.configuration.fleet import load_fleet
from common.configuration.visual_navigation import VisualEndpoints
from common.logging.async_recording import AsyncRecording
from integrations.px4.visual_aiding import receive_status, send_status

LISTENER = "/opt/px4/build/px4_sitl_default/bin/px4-listener"


def read_topic(topic: str, instance: int = 0) -> str:
    """Read a single sample through the local SITL uORB listener."""
    return subprocess.run(
        [LISTENER, "--instance", str(instance), topic, "-n", "1"],
        capture_output=True,
        text=True,
        timeout=1,
        check=True,
    ).stdout


def field(sample: str, name: str, default: str = "") -> str:
    match = re.search(rf"^\s+{re.escape(name)}:\s*(\S+)", sample, re.MULTILINE)
    return match.group(1) if match else default


def configure_controller(instance: int, endpoints: VisualEndpoints, gps_enabled: bool) -> None:
    """Configure the selected disarmed test controller without weakening health checks."""
    if field(read_topic("vehicle_status", instance), "arming_state") != "1":
        raise RuntimeError(f"PX4 instance {instance}: visual setup requires confirmed disarm")
    parameters = {
        "SIM_GPS_USED": 10 if gps_enabled else 0,
        "EKF2_GPS_CTRL": 1 if gps_enabled else 0,
        "EKF2_EV_CTRL": 3,
        "EKF2_HGT_REF": 3,
        "EKF2_EV_DELAY": 0,
        "EKF2_EV_NOISE_MD": 1,
        "COM_OBL_RC_ACT": 4,
    }
    for name, value in parameters.items():
        subprocess.run(
            [
                LISTENER.replace("px4-listener", "px4-param"),
                "--instance",
                str(instance),
                "set",
                name,
                str(value),
            ],
            check=True,
            timeout=3,
        )
    mavlink = LISTENER.replace("px4-listener", "px4-mavlink")
    commands = [
        [
            "start",
            "-u",
            str(endpoints.controller_port),
            "-o",
            str(endpoints.publisher_port),
            "-m",
            "extvision",
            "-r",
            "100000",
        ],
        ["stream", "-u", str(endpoints.controller_port), "-s", "ATTITUDE_QUATERNION", "-r", "40"],
        ["stream", "-u", str(endpoints.controller_port), "-s", "TIMESYNC", "-r", "80"],
    ]
    for command in commands:
        subprocess.run([mavlink, "--instance", str(instance), *command], check=True, timeout=3)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vehicle", default="vehicle_1")
    parser.add_argument("--configure-gps", choices=("enabled", "disabled"))
    parser.add_argument("--observe-native-gnss-commands", action="store_true")
    parser.add_argument("--duration-seconds", type=int, default=450)
    args = parser.parse_args()
    if not 60 <= args.duration_seconds <= 2400:
        parser.error("Observer duration must lie within 60..2400 wall seconds")
    if os.environ.get("GZ_PARTITION") != "uav-visual-odometry":
        parser.error("Use the isolated uav-visual-odometry project")
    selected = [v for v in load_fleet() if v.enabled and v.vehicle_id == args.vehicle]
    if len(selected) != 1:
        parser.error("Choose one configured, enabled visual vehicle")
    vehicle = selected[0]
    endpoints = VisualEndpoints.for_vehicle(vehicle)
    if args.configure_gps is not None:
        configure_controller(vehicle.instance, endpoints, args.configure_gps == "enabled")
    runtime = vehicle.runtime_directory
    run_marker = runtime / "px4.run"
    controller_run_id = run_marker.read_text().strip()
    started = time.monotonic()
    recorder = AsyncRecording(args.output)
    with (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as channel,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as faults,
    ):
        channel.setblocking(False)
        faults.bind(("127.0.0.1", endpoints.gps_fault_port))
        faults.setblocking(False)
        fault_applied = False
        while (
            time.monotonic() - started < args.duration_seconds
            and run_marker.read_text().strip() == controller_run_id
        ):
            try:
                fault = receive_status(faults)
                if (
                    not fault_applied
                    and fault
                    and fault.get("controller_run_id") == controller_run_id
                    and fault.get("vehicle_id", "vehicle_1") == vehicle.vehicle_id
                ):
                    subprocess.run(
                        [
                            LISTENER.replace("px4-listener", "px4-param"),
                            "--instance",
                            str(vehicle.instance),
                            "set",
                            "SIM_GPS_USED",
                            "0",
                        ],
                        capture_output=True,
                        text=True,
                        timeout=1,
                        check=True,
                    )
                    fault_applied = True
                local = read_topic("vehicle_local_position", vehicle.instance)
                flags = read_topic("estimator_status_flags", vehicle.instance)
                gps = read_topic("sensor_gps", vehicle.instance)
                vision = read_topic("vehicle_visual_odometry", vehicle.instance)
                status = read_topic("vehicle_status", vehicle.instance)
                vision_stamp = int(field(vision, "timestamp", "0"))
                capture_stamp = int(field(vision, "timestamp_sample", "0"))
                local_stamp = int(field(local, "timestamp", "0"))
                flag_stamp = int(field(flags, "timestamp", "0"))
                gps_stamp = int(field(gps, "timestamp", "0"))
                requested_enabled = args.configure_gps != "disabled" and not fault_applied
                if args.observe_native_gnss_commands:
                    command_state = (
                        vehicle.output_directory(Path(os.environ["OUTPUT_DIR"]))
                        / "environment-state.json"
                    )
                    if command_state.exists():
                        applied = json.loads(command_state.read_text())
                        if applied.get("run_id", "").strip() == controller_run_id:
                            requested_enabled = applied["gnss_enabled"]
                record = {
                    "controller_run_id": controller_run_id,
                    "monotonic": time.monotonic(),
                    "simulation_timestamp_us": local_stamp,
                    "vision_receipt_timestamp_us": vision_stamp,
                    "vision_capture_timestamp_us": capture_stamp,
                    "capture_stamp_valid": 0 < vision_stamp - capture_stamp <= 150_000
                    and abs(local_stamp - vision_stamp) <= 300_000,
                    "vision_xy_active": field(flags, "cs_ev_pos") == "True",
                    "vision_height_active": field(flags, "cs_ev_hgt") == "True",
                    "barometric_height_active": field(flags, "cs_baro_hgt") == "True",
                    "source_samples_fresh": abs(local_stamp - flag_stamp) <= 1_200_000
                    and abs(local_stamp - gps_stamp) <= 1_200_000,
                    "gnss_xy_active": field(flags, "cs_gnss_pos") == "True",
                    "gnss_velocity_active": field(flags, "cs_gnss_vel") == "True",
                    "gnss_height_active": field(flags, "cs_gps_hgt") == "True",
                    "gps_fix_type": int(field(gps, "fix_type", "-1")),
                    "satellites_used": int(field(gps, "satellites_used", "-1")),
                    "xy_valid": field(local, "xy_valid") == "True",
                    "z_valid": field(local, "z_valid") == "True",
                    "xy_reset_counter": int(field(local, "xy_reset_counter", "-1")),
                    "z_reset_counter": int(field(local, "z_reset_counter", "-1")),
                    "mag_aligned_in_flight": field(flags, "cs_mag_aligned_in_flight") == "True",
                    "heading_reset_counter": int(field(local, "heading_reset_counter", "-1")),
                    "delta_heading_rad": float(field(local, "delta_heading", "0")),
                    "arming_state": int(field(status, "arming_state", "-1")),
                    "gps_loss_injected": fault_applied
                    or (args.configure_gps == "enabled" and not requested_enabled),
                    "gps_requested_enabled": requested_enabled,
                }
            except (ValueError, subprocess.SubprocessError) as error:
                record = {
                    "controller_run_id": controller_run_id,
                    "monotonic": time.monotonic(),
                    "error": str(error),
                }
            record["kind"] = "aiding"
            record["vehicle_id"] = vehicle.vehicle_id
            send_status(channel, endpoints.status_port, record)
            recorder.offer(record)
            time.sleep(0.15)
    print(json.dumps(recorder.close()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
