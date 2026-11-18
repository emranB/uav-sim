"""Finite staged paired stereo reliability matrix with retained attempts and uncertainty."""

import argparse
import hashlib
import json
import math
import subprocess
import time
from pathlib import Path

from common.configuration.navigation_stereo import NavigationStereoCalibration
from common.logging.storage import write_json_atomic
from testing.simulation.navigation_acceptance import reset_vehicle, select_scene, wait_ready


def wilson(successes: int, trials: int) -> list[float] | None:
    """Return a two-sided 95% Wilson score interval for the observed denominator."""
    if not trials:
        return None
    z = 1.959963984540054
    proportion = successes / trials
    denominator = 1 + z * z / trials
    center = (proportion + z * z / (2 * trials)) / denominator
    radius = (
        z
        * math.sqrt(proportion * (1 - proportion) / trials + z * z / (4 * trials**2))
        / denominator
    )
    return [max(0, center - radius), min(1, center + radius)]


def interrupted_reports(root: Path) -> set[Path]:
    """Read explicit external interruptions without reclassifying raw flight outcomes."""
    path = root / "stereo-trial-classifications.json"
    if not path.exists():
        return set()
    classifications = json.loads(path.read_text())
    if classifications["schema_version"] != 1:
        raise ValueError("Unsupported stereo trial-classification schema")
    return {
        Path(trial["report"])
        for trial in classifications["trials"]
        if trial["classification"] == "interrupted"
    }


def nominal_development_attempt(flight: dict, path: Path, interrupted: set[Path]) -> bool:
    """Keep RTF benchmarks, degraded checks and external interruptions separate."""
    return (
        path not in interrupted
        and flight.get("expected") == "reached"
        and flight.get("obstacle_mode") in {"camera", "fused"}
        and not flight.get("label", "").startswith(
            ("stereo-degraded", "stereo-reciprocal", "stereo-rtf-")
        )
    )


def load_preserved_cases(
    paths: list[Path], declared: dict[str, dict], modes: list[str], retain_failed: bool = False
) -> list[dict]:
    """Preserve passing physical evidence with its actual revision, scene and input settings."""
    preserved = {}
    for path in paths:
        matrix = json.loads(path.read_text())
        for case in matrix["cases"]:
            if (
                case["status"] not in ({"pass", "fail"} if retain_failed else {"pass"})
                or case["name"] not in declared
                or case["mode"] not in modes
            ):
                continue
            definition = declared[case["name"]]
            flight_path = Path(case["report"])
            flight = json.loads(flight_path.read_text())
            observed = flight.get("navigation_result", {})
            quality = dict(observed.get("camera_input_quality", {}))
            if quality.get("fault") == "none":
                quality.pop("fault")
            if (
                bool(flight.get("passed")) != (case["status"] == "pass")
                or (
                    case["status"] == "pass"
                    and (
                        not flight.get("perception_criteria_passed")
                        or observed.get("outcome") != "reached"
                    )
                )
                or flight.get("obstacle_mode") != case["mode"]
                or flight["scenario"] != json.loads(Path(definition["scene"]).read_text())
                or quality != definition["input_quality"]
                or observed.get("camera_calibration_id") != NavigationStereoCalibration().identifier
            ):
                raise ValueError(
                    f"Cannot preserve incompatible or failed physical evidence: {flight_path}"
                )
            preserved[(case["name"], case["mode"])] = {
                **case,
                "reused": True,
                "reused_from": str(path),
                "source_revision": case.get("source_revision", matrix["revision"]),
                "report_sha256": hashlib.sha256(flight_path.read_bytes()).hexdigest(),
            }
    return list(preserved.values())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", choices=["early", "broader", "all"], default="early")
    parser.add_argument(
        "--modes", nargs="+", choices=["camera", "fused"], default=["camera", "fused"]
    )
    parser.add_argument(
        "--cases", nargs="+", help="Run named declared configurations for focused diagnosis"
    )
    parser.add_argument("--resume-matrix", action="append", type=Path, default=[])
    parser.add_argument(
        "--retain-failed",
        action="store_true",
        help="Retain diagnosed failed slots in the fixed denominator; never retry them",
    )
    arguments = parser.parse_args()
    root = Path("outputs/reports")
    configuration = json.loads(Path("configs/navigation/stereo/reliability.json").read_text())
    cases = [
        case
        for case in configuration["nominal_cases"]
        if arguments.batch == "all" or case["batch"] == arguments.batch
    ]
    prior_nominal = []
    interrupted = interrupted_reports(root)
    for path in sorted(root.glob("avoidance-stereo-*.json")):
        attempt = json.loads(path.read_text())
        if nominal_development_attempt(attempt, path, interrupted):
            prior_nominal.append(
                {"report": str(path), "mode": attempt["obstacle_mode"], "passed": attempt["passed"]}
            )
    if arguments.cases:
        unknown = set(arguments.cases) - {case["name"] for case in configuration["nominal_cases"]}
        if unknown:
            parser.error(f"Unknown declared cases: {sorted(unknown)}")
        cases = [case for case in cases if case["name"] in arguments.cases]
        if not cases:
            parser.error("No selected cases belong to the requested batch")
    preserved = load_preserved_cases(
        arguments.resume_matrix,
        {case["name"]: case for case in cases},
        arguments.modes,
        retain_failed=arguments.retain_failed,
    )
    report = {
        "schema_version": 1,
        "state": "running",
        "passed": False,
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "source_sha256": {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [
                *Path("src/common/navigation").glob("*.py"),
                *Path("src/integrations/vision").glob("navigation*.py"),
                *Path("configs/navigation/stereo").glob("*.json"),
            ]
        },
        "batch": arguments.batch,
        "cases": preserved,
        "preservation_scope": "Passing flights retained across the vertical-sampling repair; original reports keep their tested revisions and sensing metrics",
        "resume_matrices": [str(path) for path in arguments.resume_matrix],
        "prior_nominal_attempts": prior_nominal,
        "planned_cases": len(cases) * len(arguments.modes),
        "localization": "PX4 estimated pose using GNSS/IMU/magnetometer/barometer",
        "truth_usage": "Independent contact, clearance and goal evaluation only",
        "noise_seeds": "Image/texture seeds declared; PX4/Gazebo noise seeds unspecified",
    }
    destination = root / f"stereo-matrix-{arguments.batch}-{time.time_ns()}.json"

    def save() -> None:
        report["updated_wall_time"] = time.time()
        report["rates"] = {}
        for mode in arguments.modes:
            attempted = [case for case in report["cases"] if case["mode"] == mode]
            successful = sum(case["status"] == "pass" for case in attempted)
            report["rates"][mode] = {
                "successes": successful,
                "attempts": len(attempted),
                "rate": successful / len(attempted) if attempted else None,
                "wilson_95": wilson(successful, len(attempted)),
            }
        durations = []
        for case in report["cases"]:
            attempt = json.loads(Path(case["report"]).read_text())
            if attempt.get("samples"):
                durations.append(attempt["samples"][-1]["wall_time"] - attempt["started_wall_time"])
        remaining = report["planned_cases"] - len(report["cases"])
        eta = (
            [
                remaining * (sum(durations) / len(durations) + 10),
                remaining * (sum(durations) / len(durations) * 1.4 + 25),
            ]
            if durations
            else None
        )
        write_json_atomic(destination, report)
        write_json_atomic(
            root / "stereo-navigation-current-status.json",
            {
                "updated_wall_time": time.time(),
                "stage": arguments.batch
                + "/"
                + report.get("validation_stage", "preparation")
                + " nominal acceptance: "
                + report["state"],
                "current_case": report.get("current_case"),
                "report": str(destination),
                "completed": len(report["cases"]),
                "total": report["planned_cases"],
                "failed": sum(case["status"] == "fail" for case in report["cases"]),
                "blocked": int("error" in report),
                "not_run": report["planned_cases"] - len(report["cases"]),
                "rates": report["rates"],
                "prior_nominal_failed_attempts": sum(not item["passed"] for item in prior_nominal),
                "planned_nominal_slots_total": 20,
                "reporting_interval_seconds": 900,
                "counts_scope": "Current batch; previous attempts retained in linked report",
                "blocker": report.get("error"),
                "new_scope_accepted": False,
                "baseline": "outputs/reports/uav-integrated-acceptance.json",
                "next_steps": "Resolve early failures before broader/degraded acceptance",
                "estimated_batch_remaining_seconds_range": eta,
                "eta_assumption": "Observed flight durations plus reset overhead; excludes future fixes and later stages",
            },
        )

    try:
        wait_ready()
        for case in cases:
            completed = {
                (item["name"], item["mode"])
                for item in report["cases"]
                if item["status"] == "pass" or arguments.retain_failed
            }
            remaining_modes = [
                mode for mode in arguments.modes if (case["name"], mode) not in completed
            ]
            if not remaining_modes:
                continue
            report["validation_stage"] = case["batch"]
            report["current_case"] = case["name"] + ": scene preparation"
            save()
            scene = str(Path(case["scene"]).relative_to("configs/navigation").with_suffix(""))
            select_scene(scene)
            for index, mode in enumerate(remaining_modes):
                if index:
                    reset_vehicle()
                label = f"stereo-{arguments.batch}-{case['name']}-{mode}"
                report["current_case"] = label
                save()
                command = [
                    "docker",
                    "compose",
                    "exec",
                    "-T",
                    "gazebo",
                    "python",
                    "-m",
                    "testing.simulation.navigation",
                    "--scenario",
                    case["scene"],
                    "--obstacle-mode",
                    mode,
                    "--input-quality",
                    json.dumps(case["input_quality"]),
                    "--label",
                    label,
                ]
                result = subprocess.run(command, text=True, capture_output=True, timeout=360)
                (root / f"{label}-{time.time_ns()}.log").write_text(result.stdout + result.stderr)
                summary = json.loads(result.stdout.strip().splitlines()[-1])
                report["cases"].append(
                    {
                        "name": case["name"],
                        "mode": mode,
                        "held_out": case["held_out"],
                        "reused": False,
                        "source_revision": report["revision"],
                        "status": "pass" if summary["passed"] else "fail",
                        "report": summary["report"].replace("/workspace/", ""),
                        "command": command,
                    }
                )
                print(json.dumps(report["cases"][-1]), flush=True)
                save()
                if not summary["passed"]:
                    raise RuntimeError(
                        "Nominal flight failed; focused diagnosis required before expansion"
                    )
        report["state"] = "batch-complete"
        if arguments.retain_failed:
            report["passed"] = all(
                sum(item["status"] == "pass" for item in report["cases"] if item["mode"] == mode)
                / len(cases)
                >= configuration["nominal_completion_target"]
                for mode in arguments.modes
            )
            # A completion target never excuses a physical safety or stale-use violation.
            for case in report["cases"]:
                flight = json.loads(Path(case["report"]).read_text())
                report["passed"] &= (
                    flight["contact_messages"] == 0
                    and flight["minimum_clearance_m"] >= 0.25
                    and flight["final_disarmed"]
                    and flight["perception_criteria_passed"]
                )
        else:
            report["passed"] = all(case["status"] == "pass" for case in report["cases"])
    except (
        OSError,
        ValueError,
        RuntimeError,
        KeyError,
        IndexError,
        subprocess.SubprocessError,
    ) as error:
        report.update(state="needs-focused-fix", error=str(error))
    finally:
        save()
        print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
