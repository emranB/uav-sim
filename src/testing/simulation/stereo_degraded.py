"""Bounded source-failure, recovery and camera range-independence flight checks."""

import json
import subprocess
import time
from pathlib import Path

from testing.simulation.navigation_acceptance import fleet, request, reset_vehicle, select_scene


def main() -> int:
    root = Path("outputs/reports")
    destination = root / f"stereo-degraded-{time.time_ns()}.json"
    report = {
        "schema_version": 1,
        "state": "running",
        "cases": [],
        "passed": False,
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
    }
    cases = [
        ("missing_left", "camera", "missing_left", "none", "failed"),
        ("invalid_depth", "camera", "invalid_depth", "none", "failed"),
        ("dark_images", "camera", "dark", "none", "failed"),
        ("stale", "camera", "stale", "none", "failed"),
        ("calibration", "camera", "calibration", "none", "failed"),
        ("fused_inconsistency", "fused", "depth_scale", "none", "failed"),
        ("fused_range_loss", "fused", "none", "ranges", "failed"),
        ("camera_range_independence", "camera", "none", "ranges", "reached"),
        ("worker_restart", "camera", "none", "none", "failed"),
    ]

    def save() -> None:
        report["updated_wall_time"] = time.time()
        destination.write_text(json.dumps(report, indent=2) + "\n")
        (root / "stereo-navigation-current-status.json").write_text(
            json.dumps(
                {
                    "updated_wall_time": time.time(),
                    "stage": "degraded-source acceptance",
                    "current_case": report.get("current_case"),
                    "report": str(destination),
                    "completed": len(report["cases"]),
                    "total": len(cases),
                    "failed": sum(not case["passed"] for case in report["cases"]),
                    "blocked": int("error" in report),
                    "not_run": len(cases) - len(report["cases"]),
                    "blocker": report.get("error"),
                    "new_scope_accepted": False,
                    "baseline": "outputs/reports/uav-integrated-acceptance.json",
                    "next_steps": "Recovery, reciprocal target and final integrated review",
                    "eta": None,
                },
                indent=2,
            )
            + "\n"
        )

    try:
        select_scene("stereo/single-01")
        for index, (label, mode, camera_fault, fault, expected) in enumerate(cases):
            if index:
                previous_run = fleet()["vehicle_1"]["run_id"]
                reset_vehicle()
                if fleet()["vehicle_1"]["run_id"] == previous_run:
                    raise RuntimeError("Reset failed to establish a new controller generation")
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
                "configs/navigation/stereo/single-01.json",
                "--label",
                "stereo-degraded-" + label,
                "--obstacle-mode",
                mode,
                "--camera-fault",
                camera_fault,
                "--fault",
                fault,
                "--expected",
                expected,
            ]
            with (root / f"stereo-degraded-{label}-{time.time_ns()}.log").open("w+") as stream:
                process = subprocess.Popen(
                    command, stdout=stream, stderr=subprocess.STDOUT, text=True
                )
                restarted = False
                deadline = time.monotonic() + 360
                while process.poll() is None:
                    if time.monotonic() > deadline:
                        process.terminate()
                        raise TimeoutError("Degraded acceptance process exceeded 360 seconds")
                    if label == "worker_restart" and not restarted:
                        state = request("navigation?vehicle_id=vehicle_1")
                        if state.get("state") == "navigating":
                            subprocess.run(
                                ["docker", "compose", "restart", "navigation-stereo"],
                                check=True,
                                timeout=30,
                                stdout=stream,
                                stderr=subprocess.STDOUT,
                            )
                            restarted = True
                    time.sleep(0.3)
                stream.seek(0)
                lines = stream.read().strip().splitlines()
            summary = next(
                json.loads(line) for line in reversed(lines) if line.startswith('{"passed"')
            )
            physical_report = json.loads(
                Path(summary["report"].replace("/workspace/", "")).read_text()
            )
            failure_reason = physical_report.get("navigation_result", {}).get("error") or ""
            reasons = {
                "missing_left": ("missing_left",),
                "invalid_depth": ("Insufficient consistent stereo",),
                "dark_images": ("Insufficient consistent stereo",),
                "stale": ("stale", "expired"),
                "calibration": ("calibration mismatch",),
                "fused_inconsistency": ("Inconsistent stereo/range",),
                "fused_range_loss": ("range subscription disabled",),
                "worker_restart": ("restarted", "stereo unavailable", "stale"),
            }
            if label in reasons and not any(reason in failure_reason for reason in reasons[label]):
                summary["passed"] = False
            if label == "worker_restart" and not restarted:
                summary["passed"] = False
            report["cases"].append(
                {
                    "name": label,
                    "mode": mode,
                    "passed": summary["passed"],
                    "report": summary["report"].replace("/workspace/", ""),
                    "worker_restarted": restarted,
                    "failure_reason": failure_reason,
                    "command": command,
                }
            )
            save()
            print(json.dumps(report["cases"][-1]), flush=True)
            if not summary["passed"]:
                raise RuntimeError("Degraded behavior failed; focused diagnosis required")
        report.update(state="complete", passed=True)
    except (
        OSError,
        RuntimeError,
        ValueError,
        KeyError,
        StopIteration,
        subprocess.SubprocessError,
    ) as error:
        report.update(state="needs-focused-fix", error=str(error))
    finally:
        save()
        print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
