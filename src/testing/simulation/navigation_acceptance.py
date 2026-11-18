"""Finite host-driven avoidance matrix; preserves per-case reports and baseline evidence."""

import argparse
import hashlib
import json
import os
import subprocess
import time
import urllib.request
from pathlib import Path


def request(route: str, body: dict | None = None) -> dict:
    operation = urllib.request.Request(
        f"http://127.0.0.1:{int(os.getenv('BACKEND_PORT', '18080'))}/" + route,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(operation, timeout=40) as response:
        return json.load(response)


def fleet() -> dict:
    return {v["vehicle_id"]: v for v in request("vehicles")["items"]}


def wait_ready(previous: dict | None = None) -> None:
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        try:
            current = fleet()
            if all(v["ready"] and not v["armed"] for v in current.values()) and (
                previous is None or all(current[k]["run_id"] != run for k, run in previous.items())
            ):
                return
        except OSError:
            pass
        time.sleep(0.5)
    raise TimeoutError("New controller generations did not become ready within 90 seconds")


def reset_vehicle() -> None:
    before = fleet()["vehicle_1"]["run_id"]
    request("reset", {"vehicle_id": "vehicle_1"})
    wait_ready({"vehicle_1": before})


def select_scene(name: str) -> None:
    before = fleet()
    if any(v["armed"] for v in before.values()):
        raise RuntimeError("Scene change requires every vehicle disarmed")
    subprocess.run(
        ["docker", "compose", "up", "-d", "--no-deps", "--force-recreate", "gazebo"],
        env={**os.environ, "UAV_AVOIDANCE_SCENARIO": name},
        check=True,
        timeout=45,
    )
    wait_ready({key: value["run_id"] for key, value in before.items()})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reuse-multiple", type=Path)
    parser.add_argument("--reuse-source-revision")
    arguments = parser.parse_args()
    if bool(arguments.reuse_multiple) != bool(arguments.reuse_source_revision):
        parser.error("Reuse needs both a report and its tested source revision")
    root = Path("outputs/reports")
    destination = root / f"avoidance-matrix-{time.time_ns()}.json"
    report = {
        "schema_version": 1,
        "state": "running",
        "passed": False,
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "cases": [],
        "scope": "Static fixed-altitude simulated range navigation",
    }

    def save() -> None:
        report["updated_wall_time"] = time.time()
        destination.write_text(json.dumps(report, indent=2) + "\n")
        (root / "avoidance-current-status.json").write_text(
            json.dumps(
                {
                    "state": report["state"],
                    "revision": report["revision"],
                    "report": str(destination),
                    "cases": report["cases"],
                    "current_case": report.get("current_case"),
                    "blocker": report.get("error"),
                    "baseline_observation": "outputs/reports/observation-e2e-acceptance.json",
                },
                indent=2,
            )
            + "\n"
        )

    try:
        configuration = json.loads(
            subprocess.check_output(["docker", "compose", "config", "--format", "json"], text=True)
        )
        if configuration["services"]["gazebo"]["environment"].get("UAV_NAVIGATION") != "true":
            raise RuntimeError("Select the explicit avoidance Compose overlay before validation")
        wait_ready()
        if arguments.reuse_multiple:
            accepted = json.loads(arguments.reuse_multiple.read_text())
            if not accepted["passed"] or accepted["navigation_result"]["outcome"] != "reached":
                raise ValueError("Reusable multiple-obstacle report did not pass")
            core = [
                "src/common/navigation/session.py",
                "src/common/navigation/occupancy.py",
                "src/common/navigation/progress.py",
                "src/integrations/mavlink/connection.py",
                "src/gazebo/ranges.py",
                "src/gazebo/magnetometer.py",
                "src/gazebo/navigation_scene.py",
                "configs/navigation/multiple.json",
            ]
            for source in core:
                prior_source = subprocess.check_output(
                    ["git", "show", f"{arguments.reuse_source_revision}:{source}"]
                )
                if prior_source != Path(source).read_bytes():
                    raise ValueError(f"Cannot reuse flight: {source} changed after tested revision")
            if accepted["scenario"] != json.loads(
                Path("configs/navigation/multiple.json").read_text()
            ):
                raise ValueError("Reusable report used a different physical obstacle configuration")
            tested_commit_time = int(
                subprocess.check_output(
                    ["git", "show", "-s", "--format=%ct", arguments.reuse_source_revision],
                    text=True,
                )
            )
            if accepted["started_wall_time"] < tested_commit_time:
                raise ValueError("Reusable flight predates the declared source revision")
            report["reuse_source_revision"] = arguments.reuse_source_revision
            report["reused_source_sha256"] = {
                source: hashlib.sha256(Path(source).read_bytes()).hexdigest() for source in core
            }
            report["cases"].append(
                {
                    "name": "multiple",
                    "status": "pass",
                    "reused": True,
                    "report": str(arguments.reuse_multiple),
                    "sha256": hashlib.sha256(arguments.reuse_multiple.read_bytes()).hexdigest(),
                }
            )
        cases = [("single", "single", "none", "reached")]
        if not arguments.reuse_multiple:
            cases.append(("multiple", "multiple", "none", "reached"))
        cases += [
            ("blocked", "blocked", "none", "blocked"),
            ("range_loss", "blocked", "ranges", "failed"),
            ("paused_clock_loss", "blocked", "pause", "failed"),
            ("cancel", "blocked", "cancel", "cancelled"),
            ("single_repeat", "single", "none", "reached"),
        ]
        current_scene = None
        for label, scene, fault, expected in cases:
            report["current_case"] = label
            save()
            if scene != current_scene:
                select_scene(scene)
                current_scene = scene
            else:
                reset_vehicle()
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
                f"configs/navigation/{scene}.json",
                "--label",
                "matrix-" + label,
                "--fault",
                fault,
                "--expected",
                expected,
            ]
            result = subprocess.run(command, text=True, capture_output=True, timeout=240)
            (root / f"avoidance-matrix-{label}.log").write_text(result.stdout + result.stderr)
            summary = json.loads(result.stdout.strip().splitlines()[-1])
            path = summary["report"].replace("/workspace/", "")
            report["cases"].append(
                {
                    "name": label,
                    "status": "pass" if summary["passed"] else "fail",
                    "reused": False,
                    "report": path,
                    "command": command,
                }
            )
            print(json.dumps(report["cases"][-1]), flush=True)
            save()
        report["passed"] = all(case["status"] == "pass" for case in report["cases"])
        report["state"] = "complete"
    except (
        OSError,
        ValueError,
        RuntimeError,
        KeyError,
        IndexError,
        subprocess.SubprocessError,
    ) as error:
        report.update(state="stopped-with-error", error=str(error))
    finally:
        save()
        print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
