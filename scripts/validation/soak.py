"""Repeat real flights and presentation recovery checks with archived per-step logs."""

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--port", type=int, default=18080)
    arguments = parser.parse_args()
    if not 1 <= arguments.cycles <= 20:
        parser.error("--cycles must be between 1 and 20")
    root = Path(__file__).resolve().parents[2]
    directory = root / "outputs/reports" / f"soak-{time.time_ns()}"
    directory.mkdir(parents=True)
    report = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "requested_cycles": arguments.cycles,
        "status": "running",
        "checks": [],
    }
    destination = directory / "summary.json"

    def save() -> None:
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    def run(name: str, command: list[str], timeout_seconds: float) -> None:
        print(json.dumps({"started": name}), flush=True)
        started = time.monotonic()
        log_path = directory / f"{name}.log"
        error = None
        with log_path.open("w", encoding="utf-8") as log:
            try:
                result = subprocess.run(
                    command,
                    cwd=root,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=timeout_seconds,
                    check=False,
                )
                exit_code = result.returncode
            except (OSError, subprocess.TimeoutExpired) as failure:
                exit_code = None
                error = str(failure)
        check = {
            "name": name,
            "passed": exit_code == 0,
            "exit_code": exit_code,
            "elapsed_seconds": round(time.monotonic() - started, 2),
            "log": str(log_path.relative_to(root)),
        }
        if error:
            check["error"] = error
        report["checks"].append(check)
        save()
        print(json.dumps(check), flush=True)
        if not check["passed"]:
            raise RuntimeError(f"{name} failed; inspect {log_path}")

    save()
    try:
        for cycle in range(1, arguments.cycles + 1):
            flight = [
                sys.executable,
                "scripts/validation/end_to_end.py",
                "--port",
                str(arguments.port),
            ]
            if cycle % 3 in {0, 2}:
                flight.append("--pause-in-flight")
            if cycle % 3 == 0:
                flight.append("--stop-early")
            run(f"cycle-{cycle}-flight", flight, 400)
            service = "gazebo-viewer" if cycle % 2 else "desktop"
            run(
                f"cycle-{cycle}-{service}",
                [
                    sys.executable,
                    "scripts/validation/viewer_restart.py",
                    "--service",
                    service,
                    "--port",
                    str(arguments.port),
                ],
                90,
            )
            run(
                f"cycle-{cycle}-integration",
                [
                    sys.executable,
                    "scripts/validation/integration.py",
                    "--port",
                    str(arguments.port),
                ],
                120,
            )
        report["status"] = "passed"
        return 0
    except RuntimeError as error:
        report["status"] = "failed"
        report["error"] = str(error)
        return 1
    finally:
        report["passed"] = report["status"] == "passed"
        save()
        print(f"Report: {destination}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
