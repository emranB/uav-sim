"""Assemble bounded stereo acceptance evidence without hiding earlier failed attempts."""

import argparse
import hashlib
import json
import subprocess
import time
from pathlib import Path

from testing.simulation.navigation_acceptance import fleet, request
from testing.simulation.stereo_acceptance import (
    interrupted_reports,
    nominal_development_attempt,
    wilson,
)


def load_report(path: Path) -> dict:
    """Read one retained machine-readable validation report."""
    return json.loads(path.read_text())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nominal-matrix", action="append", type=Path, required=True)
    parser.add_argument("--degraded-matrix", type=Path, required=True)
    parser.add_argument("--recovery", type=Path, required=True)
    parser.add_argument("--reciprocal-report", action="append", type=Path, required=True)
    parser.add_argument("--range-regression", type=Path, required=True)
    arguments = parser.parse_args()
    root = Path("outputs/reports")
    destination = root / "stereo-navigation-acceptance.json"
    report = {
        "schema_version": 1,
        "passed": False,
        "updated_wall_time": time.time(),
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "scope": "Static textured fixed-height CPU stereo and fused obstacle avoidance",
        "localization": "PX4 estimated local position/attitude with GNSS, IMU, magnetometer, barometer",
        "truth_usage": "Independent goal, physical contact and sampled clearance evaluation only",
        "noise_seeds": "Texture and image perturbation seeds declared; PX4/Gazebo noise seeds unspecified",
        "baseline": "outputs/reports/uav-integrated-acceptance.json",
        "checks": [],
    }
    try:
        configuration = load_report(Path("configs/navigation/stereo/reliability.json"))
        declared = {
            (case["name"], mode)
            for case in configuration["nominal_cases"]
            for mode in configuration["modes"]
        }
        cases = [case for path in arguments.nominal_matrix for case in load_report(path)["cases"]]
        assert {(case["name"], case["mode"]) for case in cases} == declared and len(cases) == 20, (
            "Nominal slots missing or duplicated"
        )
        report["nominal_matrices"] = [str(path) for path in arguments.nominal_matrix]
        report["nominal_cases"] = cases
        report["retained_case_provenance"] = [
            {
                key: case[key]
                for key in ["name", "mode", "source_revision", "reused_from", "report_sha256"]
            }
            for case in cases
            if case.get("reused")
        ]
        report["rates"] = {}
        flights = [load_report(Path(case["report"])) for case in cases]
        sampling_counts = {}
        for flight in flights:
            stride = tuple(
                flight["navigation_result"].get("camera_point_sampling_stride_px", [4, 4])
            )
            label = f"{stride[0]} columns x {stride[1]} rows"
            sampling_counts[label] = sampling_counts.get(label, 0) + 1
        report["point_sampling_evidence_counts"] = sampling_counts
        report["revision_scope"] = (
            "Results are versioned per case; retained passes and revised sampling trials keep their tested revisions. Failed attempts remain visible with separate denominators."
        )
        for case in cases:
            if case.get("report_sha256"):
                assert (
                    hashlib.sha256(Path(case["report"]).read_bytes()).hexdigest()
                    == case["report_sha256"]
                ), "Retained physical evidence changed"
        for mode in configuration["modes"]:
            selected = [flight for flight in flights if flight["obstacle_mode"] == mode]
            successes = sum(
                flight["passed"] and flight["navigation_result"]["outcome"] == "reached"
                for flight in selected
            )
            report["rates"][mode] = {
                "successes": successes,
                "attempts": len(selected),
                "completion_rate": successes / len(selected),
                "wilson_95": wilson(successes, len(selected)),
                "minimum_sampled_clearance_m": min(
                    flight["minimum_clearance_m"] for flight in selected
                ),
                "maximum_final_goal_error_m": max(
                    flight["final_goal_error_m"] for flight in selected
                ),
                "contact_messages": sum(flight["contact_messages"] for flight in selected),
                "maximum_delivery_to_result_p95_seconds": max(
                    flight["perception_metrics"]["delivery_to_result_p95_seconds"]
                    for flight in selected
                ),
                "maximum_control_sensor_age_sim_seconds": max(
                    flight["perception_metrics"]["maximum_control_sensor_age_sim_seconds"]
                    for flight in selected
                ),
            }
            assert len(selected) == 10 and successes / len(selected) >= 0.9, (
                "Mode-specific completion target not met"
            )
            assert all(
                flight["contact_messages"] == 0
                and flight["minimum_clearance_m"] >= 0.25
                and flight["final_disarmed"]
                and flight["perception_criteria_passed"]
                for flight in selected
            ), "Safety/perception criterion failed"
        report["checks"].append({"name": "declared-twenty-slot-nominal-matrix", "passed": True})
        scene_names = {
            json.dumps(load_report(Path(case["scene"])), sort_keys=True): case["name"]
            for case in configuration["nominal_cases"]
        }
        all_attempts = []
        interrupted = interrupted_reports(root)
        report["externally_interrupted_reports"] = sorted(str(path) for path in interrupted)
        report["benchmark_scope"] = "RTF comparisons remain in their separate profile reports"
        for path in sorted(root.glob("avoidance-stereo-*.json")):
            flight = load_report(path)
            if nominal_development_attempt(flight, path, interrupted):
                all_attempts.append(
                    {
                        "report": str(path),
                        "mode": flight["obstacle_mode"],
                        "case_name": scene_names.get(
                            json.dumps(flight["scenario"], sort_keys=True)
                        ),
                        "started_wall_time": flight["started_wall_time"],
                        "passed": flight["passed"],
                        "outcome": flight.get("navigation_result", {}).get("outcome"),
                        "error": flight.get("navigation_result", {}).get("error")
                        or flight.get("error"),
                        "contacts": flight["contact_messages"],
                    }
                )
        report["all_nominal_attempts_including_development"] = all_attempts
        first_attempts = {}
        for attempt in sorted(all_attempts, key=lambda attempt: attempt["started_wall_time"]):
            if attempt["case_name"] is not None:
                first_attempts.setdefault((attempt["case_name"], attempt["mode"]), attempt)
        report["first_attempt_per_declared_slot"] = list(first_attempts.values())
        report["first_attempt_descriptive_rates"] = {}
        for mode in configuration["modes"]:
            attempted = [attempt for attempt in first_attempts.values() if attempt["mode"] == mode]
            successful = sum(attempt["passed"] for attempt in attempted)
            report["first_attempt_descriptive_rates"][mode] = {
                "successes": successful,
                "attempts": len(attempted),
                "rate": successful / len(attempted),
                "wilson_95": wilson(successful, len(attempted)),
                "interpretation": "First exposures across development revisions; distinct from resolved acceptance slots",
            }
        report["statistical_scope"] = (
            "Acceptance-slot rates include retained passes and focused retries. Development rates and first exposures are reported separately. Wilson intervals are descriptive Bernoulli calculations; configuration selection, retries and software revisions prevent treating them as a certified fixed-system reliability estimate."
        )
        report["held_out_exposure_notes"] = (
            "Scenes07–10 were predeclared held-out. The first heldout-08 camera exposure found the generic detour watchdog bug; its retry is development validation. Scenes09–10 remain unseen before evaluation of the revised policy."
        )
        report["all_attempts_descriptive_rates"] = {}
        for mode in configuration["modes"]:
            selected = [attempt for attempt in all_attempts if attempt["mode"] == mode]
            successes = sum(attempt["passed"] for attempt in selected)
            report["all_attempts_descriptive_rates"][mode] = {
                "successes": successes,
                "attempts": len(selected),
                "rate": successes / len(selected),
                "wilson_95": wilson(successes, len(selected)),
                "interpretation": "Includes earlier software revisions and focused development attempts; not one fixed-system reliability population",
            }
        evidence = [
            ("degraded-sources", arguments.degraded_matrix),
            ("reset-restart-isolation-history", arguments.recovery),
            ("range-only-regression", arguments.range_regression),
        ] + [("reciprocal-target", path) for path in arguments.reciprocal_report]
        for name, path in evidence:
            validation = load_report(path)
            report["checks"].append(
                {"name": name, "passed": validation["passed"], "report": str(path)}
            )
            assert validation["passed"], f"Required evidence failed: {path}"
        report["final_fleet"] = [
            {key: vehicle[key] for key in ["vehicle_id", "run_id", "ready", "armed"]}
            for vehicle in fleet().values()
        ]
        assert all(vehicle["ready"] and not vehicle["armed"] for vehicle in report["final_fleet"])
        report["final_navigation"] = {
            vehicle["vehicle_id"]: request("navigation?vehicle_id=" + vehicle["vehicle_id"])
            for vehicle in report["final_fleet"]
        }
        assert all(
            state["state"] == "idle" and not state["active"]
            for state in report["final_navigation"].values()
        )
        resources = [
            json.loads(line)
            for line in (root / "stereo-coexistence-resources.jsonl").read_text().splitlines()
        ]
        report["coexistence"] = {
            "report": str(root / "stereo-coexistence-resources.jsonl"),
            "samples": len(resources),
            "minimum_available_memory_kib": min(
                sample["memory_available_kib"]
                for sample in resources
                if "memory_available_kib" in sample
            ),
            "static_space_apps_http_200_samples": sum(
                sample.get("space_apps_http") == 200 for sample in resources
            ),
            "maximum_gpu_memory_mib": max(
                (float(sample["gpu"].split(",")[0]) for sample in resources if "gpu" in sample),
                default=None,
            ),
            "scope": "Bounded static frontend GET and host resource reads; no Space Apps restart or workload control",
        }
        report["optional_cuda"] = {
            "status": "failed-external-dependency",
            "report": "outputs/reports/stereo-cuda-context-prefix-failure.json",
            "reason": "Fresh contexts returned cuInit 999; explicit CPU backend used. No driver, host or Docker restart performed.",
        }
        report["source_sha256"] = {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [
                *Path("src/common/navigation").glob("*.py"),
                *Path("src/integrations/vision").glob("navigation*.py"),
                *Path("configs/navigation/stereo").glob("*.json"),
                Path("src/integrations/vision/geometry.py"),
                Path("src/gazebo/ranges.py"),
                Path("src/gazebo/camera_streams.py"),
                Path("src/gazebo/stereo_model.py"),
                *Path("src/gazebo").glob("navigation*.py"),
                Path("src/common/interfaces/navigation.py"),
                Path("src/common/interfaces/obstacles.py"),
                Path("src/integrations/mavlink/connection.py"),
                Path("src/integrations/runtime.py"),
                Path("src/common/simulation_manager/server.py"),
                Path("src/common/configuration/navigation_stereo.py"),
                Path("deploy/compose/stereo-navigation.yaml"),
                Path("deploy/compose/stereo-navigation-gpu.yaml"),
            ]
        }
        report["limits"] = (
            "Finite engineering evidence for the declared static envelope. No universal guarantee, camera-only localization, hardware, swarm, moving/overhead hazard or positioning-disruption validation. Sampled clearance is not a continuous-time collision proof."
        )
        report["passed"] = True
    except (
        OSError,
        ValueError,
        RuntimeError,
        KeyError,
        AssertionError,
        subprocess.SubprocessError,
    ) as error:
        report["error"] = str(error)
    finally:
        destination.write_text(json.dumps(report, indent=2) + "\n")
        print(
            json.dumps(
                {
                    "passed": report["passed"],
                    "report": str(destination),
                    "error": report.get("error"),
                }
            ),
            flush=True,
        )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
