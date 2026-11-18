"""Verify ROS telemetry loss detection and XRCE agent recovery without restarting physics."""

import argparse
import json
import subprocess
import time
import urllib.request
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18080)
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    report = {"timestamp": time.time(), "passed": False}

    def read(path: str) -> dict:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{arguments.port}/{path}", timeout=5
        ) as response:
            return json.load(response)

    def agent(operation: str) -> None:
        subprocess.run(
            ["docker", "compose", operation, "dds-agent"], cwd=root, timeout=45, check=True
        )

    physics_query = ["docker", "compose", "ps", "-q", "gazebo"]
    stopped = False
    try:
        if read("telemetry")["armed"]:
            raise RuntimeError("Run transport recovery with the vehicle disarmed")
        if not read("capabilities")["ros_connected"]:
            raise RuntimeError("ROS telemetry must be connected before the recovery test")
        before = subprocess.check_output(physics_query, cwd=root, timeout=10)
        if not before.strip():
            raise RuntimeError("No running Gazebo physics container")
        stopped = True
        agent("stop")
        time.sleep(7)
        report["loss_detected"] = not read("capabilities")["ros_connected"]
        agent("start")
        stopped = False
        deadline = time.monotonic() + 60
        while not read("capabilities")["ros_connected"]:
            if time.monotonic() >= deadline:
                raise TimeoutError("ROS telemetry did not recover after XRCE agent restart")
            time.sleep(1)
        report["telemetry_recovered"] = True
        after = subprocess.check_output(physics_query, cwd=root, timeout=10)
        report["physics_container_unchanged"] = before == after
        report["passed"] = report["loss_detected"] and report["physics_container_unchanged"]
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        report["error"] = str(error)
    finally:
        if stopped:
            try:
                agent("start")
            except (OSError, subprocess.SubprocessError) as error:
                report["cleanup_error"] = str(error)
        destination = root / "outputs/reports" / f"dds-recovery-{time.time_ns()}.json"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({**report, "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
