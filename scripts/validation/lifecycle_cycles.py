"""Check repeated acknowledged pause/resume transitions against the actual world clock."""

import argparse
import json
import time
import urllib.request
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--port", type=int, default=18080)
    arguments = parser.parse_args()
    if not 1 <= arguments.cycles <= 100:
        parser.error("--cycles must be in 1..100")
    report = {"timestamp": time.time(), "checks": [], "passed": False}
    destination = (
        Path(__file__).resolve().parents[2] / "outputs/reports" / f"lifecycle-{time.time_ns()}.json"
    )
    paused = False

    def request(resource: str, command: bool = False) -> dict:
        query = urllib.request.Request(
            f"http://127.0.0.1:{arguments.port}/{resource}",
            data=b"{}" if command else None,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(query, timeout=15) as response:
            return json.load(response)

    def clock() -> float:
        return request("scene")["poses"][0]["simulation_time_seconds"]

    try:
        if request("telemetry")["armed"]:
            raise RuntimeError("Lifecycle repetition requires a disarmed simulation")
        for cycle in range(arguments.cycles):
            request("pause", True)
            paused = True
            time.sleep(0.3)
            before = clock()
            time.sleep(0.3)
            after = clock()
            if abs(after - before) > 0.02:
                raise RuntimeError("Clock advanced during a settled pause")
            request("resume", True)
            paused = False
            time.sleep(0.3)
            resumed = clock()
            if resumed <= after:
                raise RuntimeError("Clock did not advance after resuming")
            report["checks"].append(
                {
                    "cycle": cycle + 1,
                    "passed": True,
                    "paused_advance_seconds": after - before,
                    "resumed_advance_seconds": resumed - after,
                }
            )
            print(json.dumps(report["checks"][-1]), flush=True)
        report["passed"] = True
        return 0
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        report["error"] = str(error)
        return 1
    finally:
        if paused:
            try:
                request("resume", True)
            except (OSError, ValueError) as error:
                report["cleanup_error"] = str(error)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"Report: {destination}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
