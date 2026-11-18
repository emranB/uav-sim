"""Two-mode fixed-scene clock/load benchmark; never mixes profiles in a nominal matrix."""

import argparse
import hashlib
import json
import os
import subprocess
import time
import urllib.request
from pathlib import Path

from common.logging.storage import write_json_atomic
from testing.simulation.navigation_acceptance import (
    fleet,
    request,
    reset_vehicle,
    select_scene,
    wait_ready,
)

MAP_FIELDS = {
    "ready",
    "stale",
    "state",
    "position_valid",
    "tracking_state",
    "session_id",
    "map_id",
    "map_generation",
    "processed_frames",
    "tracked_frames",
    "tracking_loss_events",
    "input_gap_events",
    "input_gap_seconds",
    "input_age_seconds",
    "processing_seconds",
    "surface_seconds",
    "surface_backend",
    "age_seconds",
    "simulation_timestamp_ns",
}


def sample() -> dict:
    """Observe clocks, original observation and resources; no command path consumes this."""
    started = time.monotonic()
    scene = request("scene")
    captured = time.monotonic()
    row = {
        "wall_time": time.time(),
        "clock_monotonic": (started + captured) / 2,
        "scene_query_seconds": captured - started,
        "simulation_seconds": scene["poses"][0]["simulation_time_seconds"],
        "altitudes_m": {pose["vehicle_id"]: pose["position_enu_m"][2] for pose in scene["poses"]},
        "navigation": request("navigation?vehicle_id=vehicle_1"),
        "observation": {},
        "capture": {},
    }
    row["navigation"] = {
        key: value
        for key, value in row["navigation"].items()
        if key.startswith(("camera_", "range_", "source_"))
        or key
        in {"state", "active", "error", "obstacle_mode", "mapping_seconds", "planning_seconds"}
    }
    for vehicle_id in ["vehicle_1", "vehicle_2"]:
        mapping = request("perception/map?vehicle_id=" + vehicle_id)
        row["observation"][vehicle_id] = {
            key: value for key, value in mapping.items() if key in MAP_FIELDS
        }
        camera = request("perception?vehicle_id=" + vehicle_id)["camera"]
        row["capture"][vehicle_id] = {
            key: value for key, value in camera.items() if not isinstance(value, (list, dict))
        }
    try:
        started = time.monotonic()
        with urllib.request.urlopen("http://127.0.0.1:8090", timeout=4) as response:
            row["space_apps_http_status"] = response.status
        row["space_apps_get_seconds"] = time.monotonic() - started
        services = (
            "gazebo",
            "control-panel",
            "navigation-stereo",
            "live-mapping",
            "camera-streams",
        )
        container_ids = subprocess.check_output(
            ["docker", "compose", "ps", "-q", *services],
            text=True,
            timeout=5,
        ).split()
        if len(container_ids) != len(services):
            raise RuntimeError("Acceptance project lacks a running measured service")
        row["docker_stats"] = subprocess.check_output(
            [
                "docker",
                "stats",
                "--no-stream",
                "--format",
                "{{json .}}",
                *container_ids,
            ],
            text=True,
            timeout=8,
        ).splitlines()
        row["mavlink_udp"] = json.loads(
            subprocess.check_output(
                [
                    "docker",
                    "compose",
                    "exec",
                    "-T",
                    "control-panel",
                    "python",
                    "-c",
                    "import json; from pathlib import Path; "
                    "rows=[line.split() for line in Path('/proc/net/udp').read_text().splitlines()[1:]]; "
                    "print(json.dumps([{'port':int(r[1].split(':')[1],16),'kernel_drops':int(r[-1]),"
                    "'inode':r[9]} for r in rows if int(r[1].split(':')[1],16) "
                    "in [14540,14541,14640,14641]]))",
                ],
                text=True,
                timeout=4,
            )
        )
        row["stereo_worker"] = json.loads(
            subprocess.check_output(
                [
                    "docker",
                    "compose",
                    "exec",
                    "-T",
                    "navigation-stereo",
                    "python",
                    "-c",
                    "import json; from pathlib import Path; "
                    "p=json.loads(Path('/runtime/navigation-depth.json').read_text()); "
                    "print(json.dumps({k:p.get(k) for k in "
                    "['dropped_batches','timestamp_seconds','processing_seconds','source_session_id']}))",
                ],
                text=True,
                timeout=4,
            )
        )
        row["gpu"] = subprocess.check_output(
            [
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used,utilization.memory",
                "--format=csv,noheader,nounits",
            ],
            text=True,
            timeout=4,
        ).strip()
        row["available_memory_kib"] = int(
            next(
                line.split()[1]
                for line in Path("/proc/meminfo").read_text().splitlines()
                if line.startswith("MemAvailable:")
            )
        )
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        row["resource_error"] = str(error)
    return row


def summarize_samples(rows: list[dict]) -> dict:
    """Keep independent observer continuity separate from navigation success."""
    first, last = rows[0], rows[-1]
    elapsed = last["clock_monotonic"] - first["clock_monotonic"]
    airborne = [row for row in rows if row["altitudes_m"]["vehicle_1"] > 3]
    observations = [row["observation"]["vehicle_1"] for row in airborne]
    sessions = {
        (p.get("session_id"), p.get("map_id"), p.get("map_generation")) for p in observations
    }
    observer_passed = (
        len(observations) >= 3
        and len(sessions) == 1
        and all(
            p.get("ready") and p.get("position_valid") and not p.get("stale") for p in observations
        )
        and len({p.get("tracking_loss_events") for p in observations}) == 1
        and len({p.get("input_gap_events") for p in observations}) == 1
    )
    cpu_peaks = {}
    for row in rows:
        for value in row.get("docker_stats", []):
            stats = json.loads(value)
            cpu_peaks[stats["Name"]] = max(
                cpu_peaks.get(stats["Name"], 0), float(stats["CPUPerc"].rstrip("%"))
            )
    gpu_samples = [list(map(float, row["gpu"].split(","))) for row in rows if row.get("gpu")]
    dropped = [
        r["stereo_worker"]["dropped_batches"]
        for r in rows
        if r.get("stereo_worker", {}).get("dropped_batches") is not None
    ]
    return {
        "airborne_observer_continuity_passed": observer_passed,
        "maximum_container_cpu_percent": cpu_peaks,
        "mavlink_kernel_drop_counts": {
            str(port): {
                "first": next(
                    (
                        entry["kernel_drops"]
                        for row in rows
                        for entry in row.get("mavlink_udp", [])
                        if entry["port"] == port
                    ),
                    None,
                ),
                "last": next(
                    (
                        entry["kernel_drops"]
                        for row in reversed(rows)
                        for entry in row.get("mavlink_udp", [])
                        if entry["port"] == port
                    ),
                    None,
                ),
            }
            for port in [14540, 14541, 14640, 14641]
        },
        "maximum_gpu_utilization_percent": max((r[0] for r in gpu_samples), default=None),
        "maximum_gpu_memory_mib": max((r[1] for r in gpu_samples), default=None),
        "stereo_worker_dropped_batches_delta": max(dropped) - min(dropped) if dropped else None,
        "capture_dropped_pairs_delta": {
            vehicle_id: max(r["capture"][vehicle_id].get("dropped_pairs", 0) for r in rows)
            - min(r["capture"][vehicle_id].get("dropped_pairs", 0) for r in rows)
            for vehicle_id in ["vehicle_1", "vehicle_2"]
        },
        "achieved_real_time_factor": (last["simulation_seconds"] - first["simulation_seconds"])
        / elapsed
        if elapsed
        else None,
        "clock_samples": len(rows),
        "airborne_samples": len(airborne),
        "airborne_observer_ready_samples": sum(p.get("ready", False) for p in observations),
        "airborne_observer_invalid_samples": sum(
            not p.get("position_valid", False) for p in observations
        ),
        "airborne_observer_stale_samples": sum(p.get("stale", True) for p in observations),
        "airborne_map_identity_count": len(sessions),
        "airborne_loss_counter_delta": max(
            (p.get("tracking_loss_events", 0) for p in observations), default=0
        )
        - min((p.get("tracking_loss_events", 0) for p in observations), default=0),
        "airborne_input_gap_counter_delta": max(
            (p.get("input_gap_events", 0) for p in observations), default=0
        )
        - min((p.get("input_gap_events", 0) for p in observations), default=0),
        "space_apps_http_200_samples": sum(r.get("space_apps_http_status") == 200 for r in rows),
        "maximum_space_apps_get_seconds": max(
            (r.get("space_apps_get_seconds", 0) for r in rows), default=0
        ),
        "sampling_limit": "Sampled API states/counters; not continuous observer certification",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--factor", type=float, choices=[0.5, 1.0], required=True)
    parser.add_argument("--case", default="single-01")
    parser.add_argument("--mode", choices=["camera", "fused"])
    arguments = parser.parse_args()
    modes = [arguments.mode] if arguments.mode else ["camera", "fused"]
    declared = json.loads(Path("configs/navigation/stereo/reliability.json").read_text())
    declared_case = next(
        (item for item in declared["nominal_cases"] if item["name"] == arguments.case), None
    )
    if declared_case is None:
        parser.error("Choose one declared static stereo configuration")
    os.environ["UAV_REAL_TIME_FACTOR"] = str(arguments.factor)
    root = Path("outputs/reports")
    destination = root / f"stereo-rtf-{arguments.factor}-{time.time_ns()}.json"
    report = {
        "schema_version": 1,
        "passed": False,
        "state": "running",
        "requested_real_time_factor": arguments.factor,
        "cases": [],
        "scene": declared_case["scene"],
        "input_quality": declared_case["input_quality"],
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "scope": __doc__,
        "criteria": "Existing nominal navigation and above-3m observer gates unchanged; achieved clock >=95% requested factor",
        "source_sha256": {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in Path("src").rglob("*.py")
            if "testing" not in path.parts
        },
        "wall_budget_note": "Navigation remains bounded to 240 wall seconds; age limit remains 0.9 simulation seconds. Achieved clock ratio is measured independently.",
        "reporting_interval_seconds": 900,
    }

    def save() -> None:
        report["updated_wall_time"] = time.time()
        write_json_atomic(destination, report)
        write_json_atomic(
            root / "stereo-rtf-current-status.json",
            {
                "state": report["state"],
                "report": str(destination),
                "factor": arguments.factor,
                "completed": len(report["cases"]),
                "total": len(modes),
                "broad_campaign": "awaiting-focused-validation",
                "reporting_interval_seconds": 900,
                "current_mode": report.get("current_mode"),
                "updated_wall_time": report["updated_wall_time"],
            },
        )

    save()
    try:
        wait_ready()
        select_scene(
            str(Path(declared_case["scene"]).relative_to("configs/navigation").with_suffix(""))
        )
        report["actual_gazebo_factor"] = subprocess.check_output(
            [
                "docker",
                "compose",
                "exec",
                "-T",
                "gazebo",
                "printenv",
                "UAV_REAL_TIME_FACTOR",
            ],
            text=True,
        ).strip()
        assert float(report["actual_gazebo_factor"]) == arguments.factor
        for index, mode in enumerate(modes):
            if index:
                reset_vehicle()
            report["current_mode"] = mode
            save()
            label = f"stereo-rtf-{arguments.factor}-{arguments.case}-{mode}"
            log_path = root / f"{label}-{time.time_ns()}.log"
            samples_path = log_path.with_suffix(".samples.jsonl")
            rows = []
            with log_path.open("w") as log, samples_path.open("w") as samples:
                child = subprocess.Popen(
                    [
                        "docker",
                        "compose",
                        "exec",
                        "-T",
                        "gazebo",
                        "python",
                        "-m",
                        "testing.simulation.navigation",
                        "--scenario",
                        report["scene"],
                        "--obstacle-mode",
                        mode,
                        "--input-quality",
                        json.dumps(declared_case["input_quality"]),
                        "--label",
                        label,
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                deadline = time.monotonic() + 360
                try:
                    while child.poll() is None:
                        if time.monotonic() > deadline:
                            raise TimeoutError("Benchmark flight exceeded bounded evaluation")
                        row = sample()
                        rows.append(row)
                        samples.write(json.dumps(row) + "\n")
                        samples.flush()
                        time.sleep(1)
                    child.wait(timeout=5)
                finally:
                    if child.poll() is None:
                        request("navigation/cancel", {"vehicle_id": "vehicle_1"})
                        child.wait(timeout=65)
            result = json.loads(log_path.read_text().strip().splitlines()[-1])
            flight_path = Path(result["report"].replace("/workspace/", ""))
            flight = json.loads(flight_path.read_text())
            case = {
                "mode": mode,
                "passed": result["passed"],
                "report": str(flight_path),
                "samples": str(samples_path),
                "measurements": summarize_samples(rows),
                "perception_metrics": flight.get("perception_metrics"),
                "goal_error_m": flight.get("final_goal_error_m"),
                "clearance_m": flight.get("minimum_clearance_m"),
                "contacts": flight.get("contact_messages"),
                "outcome": flight.get("navigation_result", {}).get("outcome"),
                "wall_end_to_end_seconds": flight["samples"][-1]["wall_time"]
                - flight["started_wall_time"]
                if flight.get("samples")
                else None,
                "error": flight.get("navigation_result", {}).get("error") or flight.get("error"),
            }
            report["cases"].append(case)
            save()
            print(json.dumps(case), flush=True)
            assert all(not v["armed"] for v in fleet().values()), "Actual disarm required"
        report["navigation_pair_passed"] = all(case["passed"] for case in report["cases"])
        report["passed"] = report["navigation_pair_passed"] and all(
            case["measurements"]["airborne_observer_continuity_passed"]
            and case["measurements"]["achieved_real_time_factor"] >= arguments.factor * 0.95
            for case in report["cases"]
        )
        report["state"] = "profile-pair-passed" if report["passed"] else "needs-diagnosis"
    except (
        OSError,
        ValueError,
        KeyError,
        RuntimeError,
        AssertionError,
        subprocess.SubprocessError,
    ) as error:
        report.update(state="blocked", error=str(error))
    finally:
        save()
    print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
