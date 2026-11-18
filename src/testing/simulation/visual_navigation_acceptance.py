"""Fly and score one bounded visual navigation case in the isolated Gazebo project."""

import argparse
import hashlib
import json
import math
import os
import socket
import time
import urllib.error
import urllib.request
from functools import partial
from pathlib import Path

from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node, SubscribeOptions

from common.configuration.fleet import load_fleet
from common.configuration.mission_store import MissionStore
from common.configuration.visual_navigation import VisualEndpoints
from common.logging.storage import write_json_atomic
from common.navigation.mission import VisualMission
from common.navigation.session import OUT_AND_BACK_MAXIMUM_WALL_SECONDS
from integrations.px4.visual_aiding import send_status
from testing.simulation.arrival_truth import ArrivalTruthHistory
from testing.simulation.navigation import clearance

API = "http://127.0.0.1:8080/"
# Pinned PX4 EKF2 waits seven simulated seconds after GNSS quality checks
# stop passing. The sampled status observer adds at most 0.5 simulated seconds.
GNSS_FUSION_TRANSITION_US = 7_500_000


def api_request(route: str, body: dict | None = None, *, vehicle_id: str) -> dict:
    data = json.dumps({"vehicle_id": vehicle_id, **body}).encode() if body is not None else None
    operation = urllib.request.Request(
        API + route,
        data=data,
        headers={"Content-Type": "application/json", "X-Request-Source": "visual-sitl-test"},
    )
    try:
        with urllib.request.urlopen(operation, timeout=3) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"{route}: {error.read(2048).decode()}") from error


def yaw(orientation: list) -> float:
    """Convert the Gazebo XYZW orientation to ENU heading in radians."""
    x, y, z, w = orientation
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def motion(samples: list[dict]) -> dict:
    start = samples[0]["truth"]
    end = samples[-1]["truth"]
    initial = start["position_enu_m"]
    final = end["position_enu_m"]
    heading_change = sum(
        math.remainder(
            yaw(second["truth"]["orientation_xyzw"]) - yaw(first["truth"]["orientation_xyzw"]),
            2 * math.pi,
        )
        for first, second in zip(samples, samples[1:])
    )
    return {
        "horizontal_displacement_enu_m": [final[i] - initial[i] for i in (0, 1)],
        "horizontal_displacement_m": math.dist(initial[:2], final[:2]),
        "accumulated_horizontal_distance_m": sum(
            math.dist(first["truth"]["position_enu_m"][:2], second["truth"]["position_enu_m"][:2])
            for first, second in zip(samples, samples[1:])
        ),
        "heading_change_deg": math.degrees(heading_change),
        "altitude_change_m": final[2] - initial[2],
        "maximum_altitude_change_m": max(
            sample["truth"]["position_enu_m"][2] - initial[2] for sample in samples
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vehicle", default="vehicle_1")
    parser.add_argument(
        "--case", choices=("short", "gps-on", "gps-off", "gps-loss", "vision-loss"), required=True
    )
    parser.add_argument("--mission-name")
    parser.add_argument("--mission-sha256")
    parser.add_argument("--external-start", action="store_true")
    parser.add_argument("--external-gps-loss", action="store_true")
    args = parser.parse_args()
    if os.environ.get("GZ_PARTITION") != "uav-visual-odometry":
        parser.error("Use the isolated uav-visual-odometry project")
    selected = [v for v in load_fleet() if v.enabled and v.vehicle_id == args.vehicle]
    if len(selected) != 1:
        parser.error("Choose one configured, enabled visual vehicle")
    vehicle = selected[0]
    endpoints = VisualEndpoints.for_vehicle(vehicle)
    request = partial(api_request, vehicle_id=vehicle.vehicle_id)
    scenario_path = Path("configs/navigation/stereo/visual-odometry-probe.json")
    scenario = json.loads(scenario_path.read_text())
    goal_offset = {"vision-loss": [2, 0], "short": [2.5, 1.5]}.get(
        args.case, scenario["goal_offset_enu_m"]
    )
    mission = None
    if args.mission_name:
        store = MissionStore(vehicle.output_directory(Path("/workspace/outputs")))
        mission = VisualMission.from_plan(
            args.mission_name, args.mission_sha256, store.load(args.mission_name)
        )
        goal_offset = list(mission.goal_offset_enu_m)
    node = Node()
    arrival_truth = ArrivalTruthHistory(vehicle.vehicle_id) if mission is not None else None
    topics = []
    contacts: list[dict] = []
    rows = []
    errors = []
    final_navigation = {}
    trigger = None
    started = False

    def receive_truth(message: Pose_V) -> None:
        timestamp = message.header.stamp.sec + message.header.stamp.nsec / 1e9
        for pose in message.pose:
            if pose.name == vehicle.model_name:
                arrival_truth.append(
                    vehicle.vehicle_id,
                    timestamp,
                    (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                    ),
                )
                break

    def receive_contact(index: int, message: Contacts) -> None:
        if message.contact and len(contacts) < 100:
            contacts.append({"obstacle": index, "monotonic": time.monotonic()})

    def sample() -> dict:
        now = time.monotonic()
        nav = request(f"navigation?vehicle_id={vehicle.vehicle_id}")
        scene = request(f"scene?vehicle_id={vehicle.vehicle_id}")
        telemetry = request(f"telemetry?vehicle_id={vehicle.vehicle_id}")
        if (
            scene["stale"]
            or sum(p["vehicle_id"] == vehicle.vehicle_id for p in scene["poses"]) != 1
        ):
            raise RuntimeError("Evaluation requires the selected fresh Gazebo truth pose")
        pose = next(pose for pose in scene["poses"] if pose["vehicle_id"] == vehicle.vehicle_id)
        local = telemetry["messages"].get("LOCAL_POSITION_NED")
        return {
            "monotonic": now,
            "truth": pose,
            "estimated_position_enu_m": [local["y"], local["x"], -local["z"]] if local else None,
            "nav_state": nav["state"],
            "waypoint_events": nav.get("waypoint_events", []),
            "leg": nav.get("leg"),
            "armed": telemetry["armed"],
            "connected": telemetry["connected"],
            "aiding": nav["visual_localization"]["aiding"],
            "visual_stream": nav["visual_localization"]["pose"],
            "clearance_m": min(
                clearance(pose["position_enu_m"], box) for box in scenario["obstacles"]
            ),
        }

    try:
        if arrival_truth is not None:
            options = SubscribeOptions()
            options.msgs_per_sec = 30
            topic = "/world/default/pose/info"
            if not node.subscribe(Pose_V, topic, receive_truth, options):
                raise RuntimeError("Cannot subscribe to independent arrival truth")
            topics.append(topic)
        for index in range(len(scenario["obstacles"])):
            topic = f"/avoidance/contacts/{index}"
            if not node.subscribe(Contacts, topic, partial(receive_contact, index)):
                raise RuntimeError(f"Cannot subscribe to {topic}")
            topics.append(topic)
        deadline = time.monotonic() + 10
        while not all(node.topic_info(topic)[0] for topic in topics):
            if time.monotonic() > deadline:
                raise TimeoutError("Physical obstacle contact publishers unavailable")
            time.sleep(0.1)
        initial = sample()
        rows.append(initial)
        if (
            not initial["connected"]
            or initial["armed"]
            or not initial["aiding"]["vision_xy_active"]
        ):
            raise RuntimeError("Require fresh disarmed telemetry and active horizontal vision")
        initial_aiding = initial["aiding"]
        expected_fix = (
            initial_aiding["gps_fix_type"] >= 3 and initial_aiding["gnss_xy_active"]
            if args.case in {"gps-loss", "gps-on"}
            else initial_aiding["gps_fix_type"] == 0 and initial_aiding["satellites_used"] == 0
        )
        if not expected_fix:
            raise RuntimeError("Initial GPS fix does not match the selected case")
        if args.external_start:
            if mission is None:
                raise ValueError("External start requires an identified saved mission")
            write_json_atomic(
                args.output.with_suffix(".ready.json"),
                {
                    "vehicle_id": vehicle.vehicle_id,
                    "mission_sha256": mission.sha256,
                },
            )
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                navigation = request(f"navigation?vehicle_id={vehicle.vehicle_id}")
                if navigation["active"]:
                    if navigation.get("saved_mission", {}).get("sha256") != mission.sha256:
                        raise RuntimeError("External command started a different mission")
                    break
                time.sleep(0.1)
            else:
                raise TimeoutError("The explicit external mission start was not observed")
        elif mission is not None:
            request(
                "navigation/mission",
                {
                    "name": mission.name,
                    "expected_sha256": mission.sha256,
                    "stereo_horizontal_reference": args.case == "gps-loss",
                },
            )
        else:
            request(
                "navigation/start",
                {
                    "obstacle_mode": "camera",
                    "goal_offset_enu_m": goal_offset,
                    "return_to_start": True,
                    "stereo_horizontal_reference": args.case == "gps-loss",
                },
            )
        started = True
        deadline = time.monotonic() + OUT_AND_BACK_MAXIMUM_WALL_SECONDS + 50
        with args.output.with_suffix(".jsonl").open("x", encoding="utf-8") as evidence:
            while time.monotonic() < deadline:
                row = sample()
                rows.append(row)
                evidence.write(json.dumps(row) + "\n")
                evidence.flush()
                if trigger is None and row["armed"] and row["nav_state"] == "navigating":
                    if args.case == "vision-loss":
                        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as channel:
                            send_status(
                                channel,
                                endpoints.vision_fault_port,
                                {
                                    "controller_run_id": initial["aiding"]["controller_run_id"],
                                    "vehicle_id": vehicle.vehicle_id,
                                },
                            )
                        trigger = row["monotonic"]
                    elif args.case == "gps-loss" and row["estimated_position_enu_m"] is not None:
                        estimate = row["estimated_position_enu_m"]
                        start_estimate = initial["estimated_position_enu_m"]
                        arrived = (
                            any(
                                event["waypoint_id"] == "outbound-1"
                                for event in row["waypoint_events"]
                            )
                            if mission is not None
                            else (
                                start_estimate and math.dist(estimate[:2], start_estimate[:2]) >= 2
                            )
                        )
                        if arrived and args.external_gps_loss:
                            trigger = row["monotonic"]
                        elif arrived:
                            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as channel:
                                send_status(
                                    channel,
                                    endpoints.gps_fault_port,
                                    {
                                        "controller_run_id": initial["aiding"]["controller_run_id"],
                                        "vehicle_id": vehicle.vehicle_id,
                                    },
                                )
                            trigger = row["monotonic"]
                final_navigation = request(f"navigation?vehicle_id={vehicle.vehicle_id}")
                if not final_navigation["active"]:
                    break
                time.sleep(0.15)
            else:
                errors.append("Navigation exceeded the route and landing time budget")
    except (OSError, ValueError, RuntimeError, KeyError, StopIteration) as error:
        errors.append(str(error))
    finally:
        if started:
            try:
                final_navigation = request(f"navigation?vehicle_id={vehicle.vehicle_id}")
                if final_navigation["active"]:
                    request("navigation/cancel", {})
                deadline = time.monotonic() + 50
                while time.monotonic() < deadline:
                    final_navigation = request(f"navigation?vehicle_id={vehicle.vehicle_id}")
                    row = sample()
                    rows.append(row)
                    if not final_navigation["active"] and row["connected"] and not row["armed"]:
                        break
                    time.sleep(0.2)
            except (OSError, ValueError, RuntimeError, KeyError, StopIteration) as error:
                errors.append(f"Landing verification: {error}")
        for topic in topics:
            node.unsubscribe(topic)

    results = {}
    arrival_samples = arrival_truth.snapshot() if arrival_truth is not None else []
    if arrival_truth is not None:
        truth_path = args.output.with_suffix(".arrival-truth.json")
        write_json_atomic(
            truth_path,
            {
                "source": "gazebo-world-pose-direct",
                "evaluation_only": True,
                "vehicle_id": vehicle.vehicle_id,
                "dropped_samples": arrival_truth.dropped_samples,
                "samples": arrival_samples,
            },
        )
        results["arrival_truth_report"] = str(truth_path)
        if not arrival_samples or arrival_truth.dropped_samples:
            errors.append("Independent arrival truth unavailable or truncated")
    if rows:
        results["motion"] = motion(rows)
        results["minimum_clearance_m"] = min(row["clearance_m"] for row in rows)
        results["confirmed_disarm"] = (
            any(row["armed"] for row in rows) and rows[-1]["connected"] and not rows[-1]["armed"]
        )
        initial_position = rows[0]["truth"]["position_enu_m"]
        final_position = rows[-1]["truth"]["position_enu_m"]
        results["touchdown_error_m"] = math.dist(initial_position[:2], final_position[:2])
        results["touchdown_altitude_error_m"] = abs(final_position[2] - initial_position[2])
        if args.case == "gps-loss" and trigger is not None:
            switched = next(
                (
                    row
                    for row in rows
                    if row["monotonic"] >= trigger and row["aiding"].get("gps_fix_type") == 0
                ),
                None,
            )
            if switched is not None:
                results["gps_fix_loss_simulation_us"] = switched["aiding"][
                    "simulation_timestamp_us"
                ]
                results["gps_fix_loss_wall_seconds"] = switched["monotonic"] - trigger
                cleared = next(
                    (
                        row
                        for row in rows
                        if row["monotonic"] >= switched["monotonic"]
                        and not any(
                            row["aiding"].get(flag, True)
                            for flag in (
                                "gnss_xy_active",
                                "gnss_velocity_active",
                                "gnss_height_active",
                            )
                        )
                    ),
                    None,
                )
                if cleared is not None:
                    results["gnss_fusion_clear_simulation_seconds"] = (
                        cleared["aiding"]["simulation_timestamp_us"]
                        - results["gps_fix_loss_simulation_us"]
                    ) / 1e6
        estimates = [row for row in rows if row.get("estimated_position_enu_m") is not None]
        if estimates:
            initial_estimate = estimates[0]["estimated_position_enu_m"]
            drifts = [
                math.dist(
                    [row["estimated_position_enu_m"][i] - initial_estimate[i] for i in (0, 1)],
                    [row["truth"]["position_enu_m"][i] - initial_position[i] for i in (0, 1)],
                )
                for row in estimates
            ]
            results["maximum_horizontal_estimate_drift_m"] = max(drifts)
            results["final_horizontal_estimate_drift_m"] = drifts[-1]
        active_rows = [row for row in rows if row["armed"] and row["nav_state"] == "navigating"]
        if active_rows:
            results["maximum_pose_age_wall_seconds"] = max(
                row["monotonic"] - row["visual_stream"]["monotonic"] for row in active_rows
            )
            results["maximum_aiding_age_wall_seconds"] = max(
                row["monotonic"] - row["aiding"]["monotonic"] for row in active_rows
            )
        for leg in ("outbound", "return"):
            leg_rows = [
                row
                for row in rows
                if row["armed"] and row["leg"] == leg and row["nav_state"] == "navigating"
            ]
            no_fix_rows = [
                row
                for row in leg_rows
                if row["aiding"].get("gps_fix_type") == 0
                and row["aiding"].get("satellites_used") == 0
            ]
            source_deadline = (
                results.get(
                    "gps_fix_loss_simulation_us",
                    -GNSS_FUSION_TRANSITION_US,
                )
                + GNSS_FUSION_TRANSITION_US
            )
            stable_no_fix_rows = [
                row
                for row in no_fix_rows
                if row["aiding"]["simulation_timestamp_us"] >= source_deadline
            ]
            results[f"{leg}_aiding"] = {
                "samples": len(leg_rows),
                "no_fix_samples": len(no_fix_rows),
                "settled_no_fix_samples": len(stable_no_fix_rows),
                "active_synced_vision_samples": sum(
                    row["aiding"].get("vision_xy_active", False)
                    and row["aiding"].get("source_samples_fresh", False)
                    and row["aiding"].get("capture_stamp_valid", False)
                    for row in leg_rows
                ),
                "gnss_fusion_samples": sum(
                    row["aiding"].get("gnss_xy_active", False)
                    or row["aiding"].get("gnss_velocity_active", False)
                    or row["aiding"].get("gnss_height_active", False)
                    for row in stable_no_fix_rows
                ),
                "gnss_position_active_samples": sum(
                    row["aiding"].get("gnss_xy_active", False) for row in leg_rows
                ),
                "vision_height_samples": sum(
                    row["aiding"].get("vision_height_active", False) for row in leg_rows
                ),
            }
            stream_rows = [
                row["visual_stream"]
                for row in leg_rows
                if "published_count" in row["visual_stream"]
            ]
            if len(stream_rows) > 1:
                first, last = stream_rows[0], stream_rows[-1]
                interval = (last["timestamp_ns"] - first["timestamp_ns"]) / 1e9
                if interval > 0:
                    results[f"{leg}_aiding"]["published_visual_rate_sim_hz"] = (
                        last["published_count"] - first["published_count"]
                    ) / interval
            stamp = final_navigation.get(f"{leg}_arrival_simulation_seconds")
            if stamp is None:
                continue
            nearest = min(
                rows, key=lambda row: abs(row["truth"]["simulation_time_seconds"] - stamp)
            )
            position = nearest["truth"]["position_enu_m"]
            arrival_pose = nearest["truth"]
            if arrival_truth is not None:
                if not arrival_samples:
                    continue
                arrival_pose = min(
                    arrival_samples, key=lambda pose: abs(pose["simulation_time_seconds"] - stamp)
                )
                position = arrival_pose["position_enu_m"]
            target = (
                initial_position[:2]
                if leg == "return"
                else [initial_position[i] + goal_offset[i] for i in (0, 1)]
            )
            results[f"{leg}_arrival"] = {
                "timestamp_error_seconds": abs(arrival_pose["simulation_time_seconds"] - stamp),
                "horizontal_error_m": math.dist(position[:2], target),
                "altitude_error_m": abs(position[2] - initial_position[2] - 4),
            }
            departure_index = (
                0
                if leg == "outbound"
                else next(index for index, row in enumerate(rows) if row["leg"] == "return")
            )
            arrival_index = rows.index(nearest)
            if arrival_index >= departure_index:
                results[f"{leg}_motion"] = motion(rows[departure_index : arrival_index + 1])
    if contacts or not results.get("confirmed_disarm"):
        errors.append("Require zero obstacle contacts and confirmed disarm after arming")
    if results.get("touchdown_altitude_error_m", math.inf) > 0.25:
        errors.append("Touchdown height was not confirmed")
    if args.case != "vision-loss":
        if args.case == "gps-loss" and results.get("gps_fix_loss_wall_seconds", math.inf) > 3:
            errors.append("GPS loss was not confirmed within 3 wall seconds of injection")
        if (
            args.case == "gps-loss"
            and results.get("gnss_fusion_clear_simulation_seconds", math.inf)
            > GNSS_FUSION_TRANSITION_US / 1e6
        ):
            errors.append("GNSS fusion flags exceeded the pinned PX4 transition budget")
        if (
            final_navigation.get("state") != "reached"
            or results.get("touchdown_error_m", math.inf) > 0.75
        ):
            errors.append("Out-and-back did not end in a verified landing within 0.75 m of start")
        for leg in ("outbound", "return"):
            arrival = results.get(f"{leg}_arrival", {})
            if (
                arrival.get("timestamp_error_seconds", math.inf) > 0.2
                or arrival.get("horizontal_error_m", math.inf) > 0.75
                or arrival.get("altitude_error_m", math.inf) > 0.5
            ):
                errors.append(f"{leg}: truth arrival exceeded the time or position tolerance")
            aiding = results.get(f"{leg}_aiding", {})
            gps_on = args.case == "gps-on"
            if (
                (not gps_on and not aiding.get("settled_no_fix_samples"))
                or aiding["active_synced_vision_samples"] != aiding["samples"]
                or (not gps_on and aiding["gnss_fusion_samples"])
                or (gps_on and aiding["gnss_position_active_samples"] != aiding["samples"])
                or (
                    not gps_on
                    and (args.case in {"gps-off", "short"} or leg == "return")
                    and aiding["no_fix_samples"] != aiding["samples"]
                )
            ):
                errors.append(f"{leg}: expected no-fix and active visual aiding evidence missing")
            if aiding.get("published_visual_rate_sim_hz", 0) < 30:
                errors.append(f"{leg}: published visual cadence below 30 simulation Hz")
    else:
        abort_reason = final_navigation.get("error") or ""
        if trigger is None or not any(
            phrase in abort_reason
            for phrase in (
                "Visual localization evidence",
                "Visual pose capture is stale",
                "horizontal visual aiding",
            )
        ):
            errors.append("Vision-loss abort was not observed")
        elif rows:
            fault_rows = [row for row in rows if row["monotonic"] >= trigger]
            results["post_fault_motion"] = motion(fault_rows)
            if results["post_fault_motion"]["horizontal_displacement_m"] > 1.0:
                errors.append("Vision-loss landing moved more than 1.0 m horizontally")

    report = {
        "schema_version": 1,
        "case": args.case,
        "scenario": str(scenario_path),
        "vehicle_id": vehicle.vehicle_id,
        "saved_mission": vars(mission) if mission is not None else None,
        "scenario_sha256": hashlib.sha256(scenario_path.read_bytes()).hexdigest(),
        "gps_loss_trigger": (
            "outbound-1 waypoint arrival"
            if mission is not None
            else "estimated outbound displacement of two metres"
        )
        if args.case == "gps-loss"
        else None,
        "passed": not errors,
        "errors": errors,
        "fault_injected_monotonic": trigger,
        "contacts": contacts,
        "results": results,
        "navigation": final_navigation,
        "samples": rows,
    }
    write_json_atomic(args.output, report)
    print(
        json.dumps(
            {key: value for key, value in report.items() if key not in {"navigation", "samples"}}
        )
    )
    return int(bool(errors))


if __name__ == "__main__":
    raise SystemExit(main())
