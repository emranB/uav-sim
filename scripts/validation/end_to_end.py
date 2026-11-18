"""Validate a short waypoint flight against the local simulation API only."""

import argparse
import json
import math
import subprocess
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

from common.logging.storage import write_json_atomic


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--vehicle-id", default="vehicle_1")
    parser.add_argument("--timeout", type=float, default=240)
    parser.add_argument("--viewer-follow", action="store_true")
    parser.add_argument("--north-offset-m", type=float, default=16.698)
    parser.add_argument("--east-offset-m", type=float, default=11.88)
    parser.add_argument("--waypoint-acceptance-radius-m", type=float, default=0.5)
    parser.add_argument(
        "--reset-from-pause",
        action="store_true",
        help="Reset directly from a seven-second disarmed pause after landing.",
    )
    parser.add_argument(
        "--restart-control-in-flight",
        action="store_true",
        help="Restart the control API while the vehicle continues its mission.",
    )
    parser.add_argument(
        "--pause-in-flight",
        action="store_true",
        help="Pause for seven wall-clock seconds in flight before resuming or stopping.",
    )
    parser.add_argument(
        "--stop-early",
        action="store_true",
        help="Test Stop/Land during flight instead of completing all waypoints.",
    )
    arguments = parser.parse_args()
    if (
        not all(math.isfinite(v) for v in [arguments.north_offset_m, arguments.east_offset_m])
        or not 8 < math.hypot(arguments.north_offset_m, arguments.east_offset_m) <= 30
    ):
        parser.error(
            "Waypoint offsets must be finite ENU metres, total distance greater than 8 and at most 30m"
        )
    if (
        not math.isfinite(arguments.waypoint_acceptance_radius_m)
        or not 0.2 <= arguments.waypoint_acceptance_radius_m <= 2
    ):
        parser.error("Waypoint acceptance radius must be finite, in 0.2..2 metres")
    base_url = f"http://127.0.0.1:{arguments.port}"
    report = {
        "schema_version": 2,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "checks": [],
        "vehicle_id": arguments.vehicle_id,
        "waypoint_offset_enu_m": [arguments.east_offset_m, arguments.north_offset_m],
        "waypoint_acceptance_radius_m": arguments.waypoint_acceptance_radius_m,
        "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "scope": "Authored PX4 waypoint mission and lifecycle, not obstacle avoidance or camera-only control",
    }
    destination = Path("outputs/reports") / f"flight-{time.time_ns()}.json"
    destination.parent.mkdir(parents=True, exist_ok=True)

    def request(path: str, body: dict | None = None) -> dict:
        if body is not None:
            body = {**body, "vehicle_id": arguments.vehicle_id}
        else:
            path += "?vehicle_id=" + arguments.vehicle_id
        encoded = json.dumps(body).encode() if body is not None else None
        query = urllib.request.Request(
            base_url + path,
            data=encoded,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(query, timeout=40) as response:
            payload = json.load(response)
            if path.startswith("/scene"):
                payload["poses"] = [
                    pose for pose in payload["poses"] if pose["vehicle_id"] == arguments.vehicle_id
                ]
            return payload

    def record(name: str, evidence: dict) -> None:
        report["checks"].append({"name": name, "passed": True, "evidence": evidence})
        write_json_atomic(destination, report)
        print(json.dumps({"check": name, "passed": True}), flush=True)

    def expect_conflict(operation: str, body: dict) -> None:
        try:
            request("/" + operation, body)
        except urllib.error.HTTPError as error:
            detail = json.load(error)
            if error.code != 409:
                raise RuntimeError(f"Expected rejection for {operation}: {detail}") from error
            record(f"armed-{operation}-rejected", detail)
        else:
            raise RuntimeError(f"{operation} was incorrectly accepted while armed")

    def settled_paused_clock() -> float:
        """Wait for pose delivery to catch up with the acknowledged physics pause."""
        previous = request("/scene")["poses"][0]["simulation_time_seconds"]
        deadline = time.monotonic() + 3
        stable_samples = 0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            current = request("/scene")["poses"][0]["simulation_time_seconds"]
            stable_samples = stable_samples + 1 if abs(current - previous) < 0.001 else 0
            if stable_samples >= 2:
                return current
            previous = current
        raise RuntimeError("Physics clock did not settle within three seconds of pausing")

    armed = False
    try:
        deadline = time.monotonic() + 60
        while True:
            try:
                health = request("/health")
                break
            except (OSError, TimeoutError):
                if time.monotonic() > deadline:
                    raise TimeoutError("Simulation readiness did not complete within 60 seconds")
                time.sleep(1)
        if health.get("mode") != "simulation" or health.get("real_simulator") is not True:
            raise RuntimeError("End-to-end flight requires the real local simulation backend")
        record("simulation-readiness", health)
        camera_model_name = (
            request("/workspace")["vehicle"]["model_name"] if arguments.viewer_follow else None
        )
        deadline = time.monotonic() + 60
        while True:
            initial = request("/telemetry")
            gps = initial["messages"].get("GPS_RAW_INT", {})
            if gps.get("fix_type", 0) >= 3 and abs(initial["latitude_deg"]) > 0.1:
                break
            if time.monotonic() > deadline:
                raise TimeoutError("No valid simulated GPS position")
            time.sleep(1)
        latitude, longitude = initial["latitude_deg"], initial["longitude_deg"]
        if initial["armed"]:
            raise RuntimeError("Refusing to replace a mission while the simulated vehicle is armed")
        north_latitude = latitude + arguments.north_offset_m / 111320
        east_longitude = longitude + arguments.east_offset_m / (
            111320 * math.cos(math.radians(latitude))
        )
        items = []
        for command, lat, lon, altitude in (
            (22, latitude, longitude, 8),
            (16, north_latitude, longitude, 8),
            (16, north_latitude, east_longitude, 8),
            (21, latitude, longitude, 0),
        ):
            acceptance_radius_m = arguments.waypoint_acceptance_radius_m if command == 16 else 0.0
            items.append(
                {
                    "type": "SimpleItem",
                    "command": command,
                    "frame": 3,
                    "autoContinue": True,
                    "doJumpId": len(items) + 1,
                    "params": [0, acceptance_radius_m, 0, None, lat, lon, altitude],
                }
            )
        mission = {
            "fileType": "Plan",
            "version": 1,
            "groundStation": "QGroundControl",
            "mission": {
                "version": 2,
                "firmwareType": 12,
                "vehicleType": 2,
                "cruiseSpeed": 5,
                "hoverSpeed": 3,
                "plannedHomePosition": [latitude, longitude, 0],
                "items": items,
            },
            "geoFence": {"version": 2, "circles": [], "polygons": []},
            "rallyPoints": {"version": 2, "points": []},
        }
        record("mission-upload", request("/mission", mission))
        downloaded = request("/mission")
        if len(downloaded["mission"]["items"]) != len(items):
            raise RuntimeError("Downloaded mission differs from the uploaded item count")
        for index, (expected, actual) in enumerate(zip(items, downloaded["mission"]["items"])):
            if expected["command"] != actual["command"] or expected["frame"] != actual["frame"]:
                raise RuntimeError(f"Mission command or frame changed at item {index}")
            for desired, received in zip(expected["params"], actual["params"]):
                if desired is None and received is None:
                    continue
                if (
                    desired is None
                    or received is None
                    or not math.isclose(desired, received, rel_tol=1e-6, abs_tol=1e-7)
                ):
                    raise RuntimeError(f"Mission parameters changed at item {index}")
        record("mission-download", {"item_count": len(downloaded["mission"]["items"])})
        armed = True
        if arguments.viewer_follow:
            record(
                "camera-follow-request",
                request(
                    "/viewer/camera",
                    {
                        "vehicle_id": arguments.vehicle_id,
                        "preset": "chase",
                        "follow": True,
                    },
                ),
            )
        record("mission-start", request("/start", {}))
        maximum_altitude, maximum_distance = 0.0, 0.0
        stop_sent = False
        flight_pause_checked = False
        control_restarted = False
        camera_samples = 0
        reached = -1
        deadline = time.monotonic() + arguments.timeout
        while time.monotonic() < deadline:
            snapshot = request("/telemetry")
            if not snapshot["connected"]:
                raise RuntimeError("Telemetry disconnected during flight")
            maximum_altitude = max(maximum_altitude, snapshot["relative_altitude_m"])
            north = (snapshot["latitude_deg"] - latitude) * 111320
            east = (
                (snapshot["longitude_deg"] - longitude) * 111320 * math.cos(math.radians(latitude))
            )
            maximum_distance = max(maximum_distance, math.hypot(north, east))
            if arguments.viewer_follow and maximum_altitude > 5 and snapshot["armed"]:
                scene = request("/scene")
                viewer = scene["viewer"]
                if (
                    not viewer["available"]
                    or viewer["target"] != camera_model_name
                    or math.dist(
                        viewer["camera_position_enu_m"], scene["poses"][0]["position_enu_m"]
                    )
                    > 25
                ):
                    raise RuntimeError("Viewer lost its vehicle follow target during flight")
                camera_samples += 1
            reached = max(
                reached, snapshot["messages"].get("MISSION_ITEM_REACHED", {}).get("seq", -1)
            )
            if (
                arguments.restart_control_in_flight
                and maximum_altitude > 5
                and not control_restarted
            ):
                root = Path(__file__).resolve().parents[2]
                physics_query = ["docker", "compose", "ps", "-q", "gazebo"]
                physics_before = subprocess.check_output(physics_query, cwd=root, timeout=10)
                subprocess.run(
                    ["docker", "compose", "restart", "control-panel"],
                    cwd=root,
                    timeout=45,
                    check=True,
                )
                recovery_deadline = time.monotonic() + 60
                while True:
                    try:
                        if request("/health").get("real_simulator"):
                            break
                    except OSError:
                        pass
                    if time.monotonic() >= recovery_deadline:
                        raise TimeoutError("Control API did not reconnect during the mission")
                    time.sleep(1)
                physics_after = subprocess.check_output(physics_query, cwd=root, timeout=10)
                if not physics_before.strip() or physics_before != physics_after:
                    raise RuntimeError("Control API restart recreated the physics container")
                record("control-restart-during-flight", {"physics_container_unchanged": True})
                control_restarted = True
            if arguments.pause_in_flight and maximum_altitude > 5 and not flight_pause_checked:
                expect_conflict("reset", {})
                expect_conflict("mission", mission)
                record("pause-during-flight", request("/pause", {}))
                paused_time = settled_paused_clock()
                time.sleep(7)
                later_time = request("/scene")["poses"][0]["simulation_time_seconds"]
                if abs(later_time - paused_time) > 0.02:
                    raise RuntimeError(
                        f"Physics advanced {later_time - paused_time:.6f}s during the in-flight pause"
                    )
                record("in-flight-paused-clock", {"advance_seconds": later_time - paused_time})
                flight_pause_checked = True
                if not arguments.stop_early:
                    record("resume-during-flight", request("/resume", {}))
            if arguments.stop_early and maximum_altitude > 5 and not stop_sent:
                record("stop-during-flight", request("/stop", {}))
                stop_sent = True
            travelled = maximum_distance > 8 or (arguments.stop_early and stop_sent)
            if maximum_altitude > 5 and reached >= 2 and not snapshot["armed"] and not travelled:
                armed = False
                raise RuntimeError(
                    f"Completed mission travelled only {maximum_distance:.3f}m; minimum is greater than 8m"
                )
            if maximum_altitude > 5 and travelled and not snapshot["armed"]:
                armed = False
                break
            time.sleep(1)
        else:
            raise TimeoutError(
                f"Flight did not complete: altitude={maximum_altitude}, distance={maximum_distance}, reached={reached}"
            )
        if reached < 2 and not arguments.stop_early:
            raise RuntimeError(f"No confirmation of intermediate waypoint completion: {reached}")
        record(
            "waypoint-flight-and-landing",
            {
                "maximum_altitude_m": maximum_altitude,
                "maximum_distance_m": maximum_distance,
                "last_waypoint_reached": reached,
            },
        )
        if arguments.viewer_follow:
            if camera_samples < 3:
                raise RuntimeError("Insufficient airborne follow-camera samples")
            record("camera-follow-during-flight", {"airborne_samples": camera_samples})
        record("pause", request("/pause", {}))
        paused_time = settled_paused_clock()
        time.sleep(7 if arguments.reset_from_pause else 4)
        later_time = request("/scene")["poses"][0]["simulation_time_seconds"]
        if abs(later_time - paused_time) > 0.02:
            raise RuntimeError("Physics continued advancing while paused")
        record("paused-clock", {"advance_seconds": later_time - paused_time})
        if not arguments.reset_from_pause:
            record("resume", request("/resume", {}))
            record("stop-on-ground", request("/stop", {}))
        record("reset-request", request("/reset", {}))
        deadline = time.monotonic() + 60
        while True:
            final = request("/telemetry")
            if final.get("ready") and not final["armed"] and final["relative_altitude_m"] < 1:
                break
            if time.monotonic() > deadline:
                raise RuntimeError("Reset did not restore a ready, disarmed vehicle on the ground")
            time.sleep(1)
        record(
            "reset-state",
            {key: final[key] for key in ("connected", "armed", "relative_altitude_m")},
        )
        if request("/mission")["mission"]["items"]:
            raise RuntimeError("Reset retained the previous active vehicle mission")
        record("reset-cleared-active-mission", {"item_count": 0})
        report["passed"] = True
        return 0
    except (
        OSError,
        ValueError,
        RuntimeError,
        TimeoutError,
        KeyError,
        subprocess.SubprocessError,
    ) as error:
        report["passed"] = False
        report["error"] = str(error)
        if isinstance(error, urllib.error.HTTPError):
            try:
                report["error_response"] = json.load(error)
            except ValueError:
                report["error_response"] = "Response was not JSON"
        print(json.dumps({"passed": False, "error": str(error)}), flush=True)
        if armed:
            recovery_deadline = time.monotonic() + 15
            while True:
                try:
                    request("/stop", {})
                    break
                except (OSError, ValueError) as stop_error:
                    if time.monotonic() >= recovery_deadline:
                        report["cleanup_error"] = str(stop_error)
                        break
                    time.sleep(1)
            if "cleanup_error" not in report:
                landing_deadline = time.monotonic() + 60
                while True:
                    try:
                        if not request("/telemetry")["armed"]:
                            report["cleanup_landed"] = True
                            break
                    except (OSError, ValueError, KeyError):
                        # Preserve the original failure while bounded recovery continues.
                        pass
                    if time.monotonic() >= landing_deadline:
                        report["cleanup_error"] = "Landing was not confirmed within 60 seconds"
                        break
                    time.sleep(1)
        return 1
    finally:
        write_json_atomic(destination, report)
        print(f"Report: {destination.resolve()}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
