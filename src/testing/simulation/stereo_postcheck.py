"""Finite reciprocal-target flights, range baseline regression and integrated recovery."""

import json
import subprocess
import time
from pathlib import Path

from common.logging.storage import write_json_atomic
from testing.simulation.navigation_acceptance import fleet, request, select_scene, wait_ready


def main() -> int:
    root = Path("outputs/reports")
    destination = root / f"stereo-postcheck-{time.time_ns()}.json"
    report = {
        "schema_version": 1,
        "passed": False,
        "state": "running",
        "cases": [],
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
    }

    def save() -> None:
        report["updated_wall_time"] = time.time()
        write_json_atomic(destination, report)
        write_json_atomic(
            root / "stereo-navigation-current-status.json",
            {
                "updated_wall_time": time.time(),
                "stage": "reciprocal/range/recovery: " + report["state"],
                "current_case": report.get("current_case"),
                "report": str(destination),
                "completed": len(report["cases"]),
                "total": 4,
                "failed": sum(not case["passed"] for case in report["cases"]),
                "blocked": int("error" in report),
                "not_run": 4 - len(report["cases"]),
                "blocker": report.get("error"),
                "new_scope_accepted": False,
                "baseline": "outputs/reports/uav-integrated-acceptance.json",
                "next_steps": "Assemble final nominal/degraded/recovery evidence and document measured results",
            },
        )

    def run_case(label: str, command: list[str], timeout_seconds: float) -> dict:
        report["current_case"] = label
        save()
        result = subprocess.run(command, text=True, capture_output=True, timeout=timeout_seconds)
        (root / f"stereo-postcheck-{label}-{time.time_ns()}.log").write_text(
            result.stdout + result.stderr
        )
        summary = json.loads(result.stdout.strip().splitlines()[-1])
        case = {
            "name": label,
            "passed": summary["passed"] and result.returncode == 0,
            "report": summary["report"].replace("/workspace/", ""),
            "command": command,
        }
        report["cases"].append(case)
        save()
        print(json.dumps(case), flush=True)
        if not case["passed"]:
            raise RuntimeError(f"Required postcheck failed: {label}")
        return case

    try:
        select_scene("stereo/reciprocal")
        for index, mode in enumerate(["camera", "fused"]):
            if index:
                before = fleet()
                request("reset", {"vehicle_id": "vehicle_2"})
                wait_ready({"vehicle_2": before["vehicle_2"]["run_id"]})
                assert fleet()["vehicle_1"]["run_id"] == before["vehicle_1"]["run_id"], (
                    "Reciprocal reset changed peer controller"
                )
            run_case(
                "reciprocal_" + mode,
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
                    "configs/navigation/stereo/reciprocal.json",
                    "--vehicle-id",
                    "vehicle_2",
                    "--obstacle-mode",
                    mode,
                    "--label",
                    "stereo-reciprocal-" + mode,
                ],
                360,
            )
        select_scene("single")
        run_case(
            "range_regression",
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
                "configs/navigation/single.json",
                "--obstacle-mode",
                "range",
                "--label",
                "stereo-range-regression",
            ],
            270,
        )
        select_scene("stereo/single-01")
        run_case("recovery", ["python", "-m", "testing.simulation.stereo_recovery"], 450)
        report.update(state="complete", passed=True)
    except (
        OSError,
        RuntimeError,
        ValueError,
        KeyError,
        IndexError,
        AssertionError,
        subprocess.SubprocessError,
    ) as error:
        report.update(state="needs-focused-fix", error=str(error))
    finally:
        save()
        print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
