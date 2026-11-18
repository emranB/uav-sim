"""Replay recorded navigation-boundary timing and estimated poses into an opt-in ROS adapter."""

import argparse
import hashlib
import json
import math
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy

from common.interfaces.navigation import NavigationPose
from common.navigation.pose_history import EstimatedPoseHistory
from integrations.ros2.observation_sync import EstimatedObservationSynchronizer, ObservationStamp


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flight", type=Path, required=True)
    parser.add_argument("--telemetry", type=Path, required=True)
    arguments = parser.parse_args()
    flight = json.loads(arguments.flight.read_text())
    result = flight["navigation_result"]
    record_path = Path(result["report"].replace("/workspace/", ""))
    record = json.loads(record_path.read_text())
    samples = record["samples"]
    origin = flight["started_wall_time"]
    finish = result["finished_wall_time"]
    events = []
    attitude, heartbeat = None, {}
    pose_count = 0
    for line in arguments.telemetry.open():
        if not any(kind in line for kind in ['"ATTITUDE"', '"LOCAL_POSITION_NED"', '"HEARTBEAT"']):
            continue
        entry = json.loads(line)
        received = datetime.fromisoformat(entry["timestamp_utc"]).timestamp()
        if received > finish:
            break
        message = entry["message"]
        kind = message.get("mavpackettype")
        if kind == "ATTITUDE":
            attitude = message
        elif kind == "HEARTBEAT":
            heartbeat = message
        elif kind == "LOCAL_POSITION_NED" and attitude is not None and received >= origin - 2:
            pose = NavigationPose(
                (message["y"], message["x"], -message["z"]),
                math.pi / 2 - attitude["yaw"],
                received - origin,
                result["controller_run_id"],
                bool(heartbeat.get("base_mode", 0) & 128),
                f"recorded PX4 custom_mode={heartbeat.get('custom_mode', 'unavailable')}",
                message["time_boot_ms"] / 1000,
                attitude["roll"],
                -attitude["pitch"],
            )
            events.append((received - origin, "pose", pose))
            pose_count += 1
    seen = set()
    for sample in samples:
        sensing = sample["sensing"]
        if sensing is None:
            continue
        if sensing.get("source_skew_sim_seconds") not in {None, 0, 0.0}:
            raise ValueError("Cannot infer exact camera stamp from a skewed fused map record")
        stamp = sample["scan_simulation_seconds"]
        key = (sensing["source_session_id"], stamp)
        if key in seen:
            continue
        seen.add(key)
        available = sample["wall_time"] - origin
        events.append(
            (
                available,
                "observation",
                ObservationStamp(
                    flight["vehicle_id"],
                    result["controller_run_id"],
                    sensing["source_session_id"],
                    sensing["camera_calibration_id"],
                    stamp,
                    available,
                ),
            )
        )
    events.sort(key=lambda event: event[0])
    if not seen or not pose_count:
        raise ValueError("Recorded observation/estimated-pose timing prefix unavailable")
    rclpy.init(domain_id=84)
    report = {
        "schema_version": 1,
        "passed": False,
        "scope": __doc__,
        "runtime_python": sys.version,
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "source_sha256": {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [Path(__file__), Path("src/integrations/ros2/observation_sync.py")]
        },
        "flight": str(arguments.flight),
        "telemetry": str(arguments.telemetry),
        "navigation_record": str(record_path),
        "pose_events": pose_count,
        "unique_observation_events": len(seen),
        "clock_source": "Original recorded UTC callback/record times, translated to an origin-relative event clock; not original kernel-monotonic ingress times. Discrete-event replay adds no jitter or fabricated arrivals.",
        "observations_available_at": "Original post-mapping navigation-record time, not unknown producer ingress",
        "raw_horizontal_images_and_failed_frame_ingress_available": False,
        "results": [],
    }
    prototype = EstimatedObservationSynchronizer(
        flight["vehicle_id"], result["controller_run_id"], result["camera_calibration_id"]
    )
    baseline = EstimatedPoseHistory()
    current = None
    try:
        for received, kind, event in events:
            if kind == "pose":
                baseline.append(event)
                current = event
                prototype.add_pose(event, received)
                continue
            baseline_error = None
            baseline_pose = None
            baseline_started = time.monotonic()
            try:
                if (
                    current is None
                    or not -0.08
                    <= (current.simulation_timestamp_seconds - event.simulation_seconds)
                    <= 0.9
                ):
                    raise RuntimeError("sensor-age-outside-original-bounds")
                baseline_pose = baseline.at(event.simulation_seconds)
            except RuntimeError as error:
                baseline_error = str(error)
            baseline_processing = time.monotonic() - baseline_started
            started = time.monotonic()
            prototype.offer(event, received)
            selected = prototype.latest(received)
            processing = time.monotonic() - started
            accepted = selected is not None and selected.observation == event
            report["results"].append(
                {
                    "available_event_seconds": received,
                    "sensor_simulation_seconds": event.simulation_seconds,
                    "baseline_accepted": baseline_pose is not None,
                    "baseline_error": baseline_error,
                    "prototype_accepted": accepted,
                    "prototype_state": prototype.snapshot()["state"],
                    "adapter_processing_wall_seconds": processing,
                    "baseline_processing_wall_seconds": baseline_processing,
                    "position_difference_m": math.dist(
                        baseline_pose.position_enu_m, selected.pose.position_enu_m
                    )
                    if baseline_pose is not None and accepted
                    else None,
                }
            )
        error = result.get("error", "")
        match = re.search(
            r"age=([\d.-]+) simulation seconds.*frame_time=([\d.-]+), estimated_time=([\d.-]+)",
            error,
        )
        report["terminal_failure"] = {
            "original_error": error,
            "full_ingress_replay_possible": False,
            "reason": "The latest overwritten horizontal frame and its producer arrival/publication events were not archived. No arrival time was invented for this terminal frame.",
        }
        if match:
            age, sensor_time, estimated_time = map(float, match.groups())
            report["terminal_failure"].update(
                {
                    "recorded_sensor_age_sim_seconds": age,
                    "recorded_sensor_time": sensor_time,
                    "recorded_estimated_time": estimated_time,
                    "original_criterion_requires_rejection": age > 0.9,
                    "synchronization_can_make_this_frame_fresh": False,
                }
            )
        report["prototype_counters"] = prototype.snapshot()
        report["baseline_accepts"] = sum(row["baseline_accepted"] for row in report["results"])
        report["prototype_accepts"] = sum(row["prototype_accepted"] for row in report["results"])
        report["observed_prefix_order_alignment_parity"] = all(
            row["baseline_accepted"] == row["prototype_accepted"] for row in report["results"]
        )
        report["passed"] = report["observed_prefix_order_alignment_parity"]
        report["producer_age_fix_demonstrated"] = False
        report["live_integration_authorized_by_evidence"] = False
        report["input_sha256"] = {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [arguments.flight, record_path]
        }
    finally:
        rclpy.shutdown()
    destination = Path(f"outputs/reports/ros2-observation-sync-replay-{time.time_ns()}.json")
    destination.write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {
                "passed": report["passed"],
                "report": str(destination),
                "baseline_accepts": report["baseline_accepts"],
                "prototype_accepts": report["prototype_accepts"],
                "producer_age_fix_demonstrated": False,
            }
        ),
        flush=True,
    )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
