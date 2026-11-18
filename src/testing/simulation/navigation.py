"""Independent physical clearance/contact evaluation of opt-in measured-obstacle navigation."""

import argparse
import hashlib
import json
import math
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path

from common.logging.storage import write_json_atomic


def clearance(position: list[float], obstacle: dict) -> float:
    """Signed distance from a 0.5m vehicle envelope to an axis-aligned physical box."""
    delta = [
        abs(position[i] - obstacle["center_enu_m"][i]) - obstacle["size_m"][i] / 2 for i in range(3)
    ]
    return math.sqrt(sum(max(value, 0) ** 2 for value in delta)) + min(max(delta), 0) - 0.5


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--vehicle-id", default="vehicle_1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--expected", choices=["reached", "blocked", "failed", "cancelled"], default="reached"
    )
    parser.add_argument("--fault", choices=["none", "pause", "cancel", "ranges"], default="none")
    parser.add_argument("--label", default="run")
    parser.add_argument("--obstacle-mode", choices=["range", "camera", "fused"], default="range")
    parser.add_argument(
        "--camera-fault",
        default="none",
        choices=[
            "none",
            "missing_left",
            "invalid_depth",
            "stale",
            "calibration",
            "depth_scale",
            "dark",
        ],
    )
    parser.add_argument("--input-quality", type=json.loads, default={})
    parser.add_argument(
        "--observation-recovery-check",
        action="store_true",
        help="One brief stereo delivery gap, recovery, then sustained loss and landing",
    )
    arguments = parser.parse_args()
    if arguments.observation_recovery_check and (
        arguments.obstacle_mode == "range" or arguments.expected != "failed"
    ):
        parser.error(
            "Observation recovery check requires stereo sensing and expected failed outcome"
        )
    scenario = json.loads(arguments.scenario.read_text())
    report = {
        "schema_version": 1,
        "started_wall_time": time.time(),
        "passed": False,
        "label": arguments.label,
        "vehicle_id": arguments.vehicle_id,
        "scenario": scenario,
        "expected": arguments.expected,
        "fault": arguments.fault,
        "obstacle_mode": arguments.obstacle_mode,
        "camera_fault": arguments.camera_fault,
        "samples": [],
        "contact_messages": 0,
        "peer_maximum_displacement_m": 0.0,
    }
    report["source_sha256"] = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [
            Path("src/common/navigation/session.py"),
            Path("src/common/navigation/freshness.py"),
            Path("src/common/navigation/sensing.py"),
            Path("src/common/interfaces/obstacles.py"),
            Path("src/integrations/vision/navigation_source.py"),
            Path("src/common/navigation/occupancy.py"),
            Path("src/common/navigation/progress.py"),
            Path("src/common/interfaces/navigation.py"),
            Path("src/common/simulation_manager/server.py"),
            Path("src/common/simulation_manager/fleet.py"),
            Path("src/integrations/runtime.py"),
            Path("src/gazebo/stereo_model.py"),
            Path("src/integrations/mavlink/connection.py"),
            Path("src/gazebo/ranges.py"),
            Path("src/gazebo/magnetometer.py"),
            Path("src/gazebo/navigation_scene.py"),
            arguments.scenario,
        ]
    }
    destination = (
        Path("/workspace/outputs/reports") / f"avoidance-{arguments.label}-{time.time_ns()}.json"
    )

    def request(route: str, body: dict | None = None) -> dict:
        operation = urllib.request.Request(
            f"http://127.0.0.1:{arguments.port}/" + route,
            data=json.dumps(body).encode() if body is not None else None,
            headers={
                "Content-Type": "application/json",
                "X-Request-Source": "avoidance-evaluation",
            },
        )
        with urllib.request.urlopen(operation, timeout=15) as response:
            return json.load(response)

    def fleet() -> dict:
        return {v["vehicle_id"]: v for v in request("vehicles")["items"]}

    def state() -> dict:
        return request("navigation?vehicle_id=" + arguments.vehicle_id)

    peer_start_positions = {}

    def scene_pose() -> dict:
        poses = request("scene")["poses"]
        for pose in poses:
            if pose["vehicle_id"] != arguments.vehicle_id:
                initial_position = peer_start_positions.setdefault(
                    pose["vehicle_id"], pose["position_enu_m"]
                )
                report["peer_maximum_displacement_m"] = max(
                    report["peer_maximum_displacement_m"],
                    math.dist(initial_position, pose["position_enu_m"]),
                )
        return next(pose for pose in poses if pose["vehicle_id"] == arguments.vehicle_id)

    from gz.msgs10.contacts_pb2 import Contacts
    from gz.transport13 import Node

    node = Node()
    topics = []

    def contacts(message: Contacts) -> None:
        if message.contact:
            report["contact_messages"] += 1

    for index in range(len(scenario["obstacles"])):
        topic = f"/avoidance/contacts/{index}"
        if not node.subscribe(Contacts, topic, contacts):
            raise RuntimeError("Cannot subscribe to physical contact evaluation")
        topics.append(topic)
    fault_worker = None
    try:
        deadline = time.monotonic() + 45
        while not all(node.topic_info(topic)[0] for topic in topics):
            if time.monotonic() >= deadline:
                raise RuntimeError("Physical contact sensor publisher unavailable")
            time.sleep(0.2)
        report["contact_publishers_available"] = True
        deadline = time.monotonic() + 60
        while True:
            initial = fleet()
            if initial[arguments.vehicle_id]["ready"]:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Vehicle readiness did not recover within 60 seconds")
            time.sleep(0.5)
        if any(v["armed"] for v in initial.values()) or not initial[arguments.vehicle_id]["ready"]:
            raise RuntimeError("Requires ready disarmed vehicles")
        start = scene_pose()["position_enu_m"]
        goal = [start[i] + scenario["goal_offset_enu_m"][i] for i in (0, 1)]
        if arguments.obstacle_mode != "range":
            request(
                "navigation/cameras",
                {
                    "vehicle_id": arguments.vehicle_id,
                    "settings": arguments.input_quality or {"fault": "none"},
                },
            )
            time.sleep(2)
        request(
            "navigation/start",
            {
                "vehicle_id": arguments.vehicle_id,
                "goal_offset_enu_m": scenario["goal_offset_enu_m"],
                "obstacle_mode": arguments.obstacle_mode,
            },
        )
        start_time = time.monotonic()
        fault_injected = False
        gap_phase = None
        recovered_monotonic = None
        evaluation_timeout_seconds = 210 if arguments.obstacle_mode == "range" else 310
        while time.monotonic() - start_time < evaluation_timeout_seconds:
            snapshot = state()
            pose = scene_pose()
            position = pose["position_enu_m"]
            report["samples"].append(
                {
                    "wall_time": time.time(),
                    "simulation_seconds": pose["simulation_time_seconds"],
                    "position_enu_m": position,
                    "clearance_m": min(clearance(position, box) for box in scenario["obstacles"]),
                    "state": snapshot["state"],
                    "recovery_test_phase": gap_phase,
                    "freshness": {
                        key: value
                        for key, value in snapshot.items()
                        if key.startswith("freshness_") and key != "freshness_wait_events"
                    },
                    "sensing": {
                        key: value
                        for key, value in snapshot.items()
                        if key.startswith(
                            ("camera_", "range_frames", "source_", "obstacle_", "pose_time")
                        )
                    },
                }
            )
            if snapshot["state"] == "navigating" and not fault_injected:
                # A backend mission must not compete with the leased Offboard authority.
                try:
                    request("start", {"vehicle_id": arguments.vehicle_id})
                    raise RuntimeError("Conflicting mission start was accepted")
                except urllib.error.HTTPError as error:
                    if error.code != 409:
                        raise
                    report["conflicting_command_rejected"] = True
                fault_injected = True
                report["fault_injected_at_wall_time"] = time.time()
                if arguments.camera_fault != "none":
                    request(
                        "navigation/cameras",
                        {
                            "vehicle_id": arguments.vehicle_id,
                            "settings": {"fault": arguments.camera_fault},
                        },
                    )
                if arguments.fault == "cancel":
                    request("navigation/cancel", {"vehicle_id": arguments.vehicle_id})
                elif arguments.fault == "ranges":
                    request(
                        "navigation/ranges", {"vehicle_id": arguments.vehicle_id, "enabled": False}
                    )
                elif arguments.fault == "pause":
                    # Independent fault injection: shared sensor/position clock pauses.
                    code = (
                        "import time; from gazebo.transport import GazeboControlConnection; "
                        "c=GazeboControlConnection(); c.request('pause'); time.sleep(4); "
                        "c.request('resume'); c.close()"
                    )
                    fault_worker = subprocess.Popen(["python", "-c", code])
            if arguments.observation_recovery_check and fault_injected and snapshot.get("active"):
                settings = None
                if gap_phase is None and time.time() - report["fault_injected_at_wall_time"] > 3:
                    gap_phase = "brief_gap"
                    settings = {**arguments.input_quality, "fault": "stale"}
                elif gap_phase == "brief_gap" and snapshot.get("freshness_waiting"):
                    gap_phase = "awaiting_fresh_replan"
                    settings = {**arguments.input_quality, "fault": "none"}
                elif gap_phase == "awaiting_fresh_replan" and snapshot.get(
                    "freshness_recovery_count", 0
                ):
                    gap_phase = "recovered"
                    recovered_monotonic = time.monotonic()
                elif gap_phase == "recovered" and time.monotonic() - recovered_monotonic > 3:
                    gap_phase = "sustained_gap"
                    settings = {**arguments.input_quality, "fault": "stale"}
                if settings is not None:
                    request(
                        "navigation/cameras",
                        {
                            "vehicle_id": arguments.vehicle_id,
                            "settings": settings,
                        },
                    )
                    report.setdefault("observation_gap_injections", []).append(
                        {
                            "phase": gap_phase,
                            "wall_time": time.time(),
                            "simulation_seconds": pose["simulation_time_seconds"],
                            "position_enu_m": position,
                        }
                    )
            if snapshot["state"] in {"reached", "blocked", "failed", "cancelled"} and snapshot.get(
                "finished_wall_time"
            ):
                report["navigation_result"] = snapshot
                break
            time.sleep(0.1)
        else:
            raise TimeoutError(
                f"End-to-end navigation did not finish within {evaluation_timeout_seconds} seconds"
            )
        final = fleet()
        report["peer_generations_preserved"] = all(
            final[key]["run_id"] == value["run_id"]
            for key, value in initial.items()
            if key != arguments.vehicle_id
        )
        report["minimum_clearance_m"] = min(v["clearance_m"] for v in report["samples"])
        report["final_goal_error_m"] = math.dist(report["samples"][-1]["position_enu_m"][:2], goal)
        report["independent_goal_error_m"] = min(
            math.dist(sample["position_enu_m"][:2], goal) for sample in report["samples"]
        )
        report["final_disarmed"] = not final[arguments.vehicle_id]["armed"]
        metrics = [
            sample["sensing"]
            for sample in report["samples"]
            if sample["state"] == "navigating" and sample["sensing"].get("camera_frames_used")
        ]
        if arguments.obstacle_mode != "range":
            latencies = sorted(item["camera_delivery_to_result_seconds"] for item in metrics)
            report["perception_metrics"] = {
                "samples": len(metrics),
                "delivery_to_result_p95_seconds": latencies[math.ceil(len(latencies) * 0.95) - 1]
                if latencies
                else None,
                "maximum_sensor_age_sim_seconds": max(
                    (item["camera_sensor_age_sim_seconds"] for item in metrics), default=None
                ),
                "maximum_delivery_age_wall_seconds": max(
                    (item["camera_age_wall_seconds"] for item in metrics), default=None
                ),
                "maximum_control_sensor_age_sim_seconds": report["navigation_result"].get(
                    "maximum_control_sensor_age_sim_seconds"
                ),
                "maximum_control_delivery_age_wall_seconds": report["navigation_result"].get(
                    "maximum_control_delivery_age_wall_seconds"
                ),
                "mapped_control_updates": report["navigation_result"].get(
                    "mapped_control_updates", 0
                ),
                "sources_used": sorted(
                    {source for item in metrics for source in item["obstacle_sources_used"]}
                ),
            }
        perception_passed = True
        if arguments.obstacle_mode != "range" and arguments.expected == "reached":
            measured = report["perception_metrics"]
            expected_sources = (
                ["stereo"]
                if arguments.obstacle_mode == "camera"
                else ["horizontal_range", "stereo"]
            )
            backend = metrics[-1].get("camera_backend") if metrics else None
            perception_passed = bool(metrics) and (
                measured["sources_used"] == expected_sources
                and measured["mapped_control_updates"] > 0
                and measured["maximum_control_sensor_age_sim_seconds"] <= 0.9
                and measured["maximum_control_delivery_age_wall_seconds"] <= 1.8
                and measured["maximum_sensor_age_sim_seconds"] <= 0.9
                and measured["maximum_delivery_age_wall_seconds"] <= 1.8
                and measured["delivery_to_result_p95_seconds"]
                <= (0.9 if backend == "vpi-cuda" else 1.6)
            )
        if arguments.observation_recovery_check:
            result = report["navigation_result"]
            events = result.get("freshness_wait_events", [])
            stops = [event for event in events if event["kind"] == "zero_velocity_command"]
            wait_samples = [
                sample
                for sample in report["samples"]
                if sample["freshness"].get("freshness_waiting")
            ]
            report["observation_recovery_check"] = {
                "passed": gap_phase == "sustained_gap"
                and result.get("freshness_recovery_count", 0) >= 1
                and result.get("freshness_wait_count", 0) >= 2
                and result.get("freshness_abort_count", 0) == 1
                and "deadline" in (result.get("error") or "")
                and bool(stops),
                "zero_velocity_requests": len(stops),
                "sampled_physical_wait_positions": [
                    {
                        key: sample[key]
                        for key in ("simulation_seconds", "position_enu_m", "clearance_m")
                    }
                    for sample in wait_samples
                ],
                "braking_evidence_limit": "Sampled real simulator motion after stop requests; no instantaneous physical stop claim",
            }
            perception_passed = perception_passed and report["observation_recovery_check"]["passed"]
        report["fault_injected"] = fault_injected
        if arguments.fault != "none" or arguments.camera_fault != "none":
            perception_passed = perception_passed and fault_injected
        report["perception_criteria_passed"] = perception_passed
        report["passed"] = (
            perception_passed
            and report["navigation_result"]["outcome"] == arguments.expected
            and report["final_disarmed"]
            and report["peer_generations_preserved"]
            and report["peer_maximum_displacement_m"] <= 0.1
            and all(
                not value["armed"] for key, value in final.items() if key != arguments.vehicle_id
            )
            and report["contact_messages"] == 0
            and report["minimum_clearance_m"]
            >= (0.2 if arguments.obstacle_mode == "range" else 0.25)
            and (arguments.expected != "reached" or report["final_goal_error_m"] <= 1.0)
        )
    except (OSError, ValueError, RuntimeError, KeyError, StopIteration) as error:
        report["error"] = str(error)
    finally:
        try:
            if state().get("active"):
                request("navigation/cancel", {"vehicle_id": arguments.vehicle_id})
                deadline = time.monotonic() + 60
                while fleet()[arguments.vehicle_id]["armed"]:
                    if time.monotonic() >= deadline:
                        raise TimeoutError("Validation cleanup could not confirm disarm")
                    time.sleep(0.2)
        except (OSError, ValueError, KeyError) as cleanup_error:
            report["cleanup_error"] = str(cleanup_error)
        if arguments.fault == "ranges":
            try:
                request("navigation/ranges", {"vehicle_id": arguments.vehicle_id, "enabled": True})
            except OSError as error:
                report["cleanup_error"] = str(error)
        if arguments.obstacle_mode != "range":
            try:
                request(
                    "navigation/cameras",
                    {"vehicle_id": arguments.vehicle_id, "settings": {"fault": "none"}},
                )
            except OSError as error:
                report["cleanup_error"] = str(error)
        if fault_worker:
            fault_worker.wait(timeout=15)
        for topic in topics:
            node.unsubscribe(topic)
        destination.parent.mkdir(parents=True, exist_ok=True)
        write_json_atomic(destination, report)
        print(
            json.dumps(
                {
                    "passed": report["passed"],
                    "report": str(destination),
                    "error": report.get("error"),
                    "result": report.get("navigation_result", {}).get("outcome"),
                    "minimum_clearance_m": report.get("minimum_clearance_m"),
                    "independent_goal_error_m": report.get("independent_goal_error_m"),
                }
            ),
            flush=True,
        )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
