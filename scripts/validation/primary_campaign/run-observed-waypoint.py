"""Collect independent Gazebo contact, clearance and mapping evidence during a waypoint check."""

import json
import os
import subprocess
import sys
import time
from pathlib import Path

from common.logging.storage import write_json_atomic

harness = Path(__file__).resolve().parent
run = Path(os.environ["UAV_CAMPAIGN_ROOT"])
label, vehicle_id, strict, *arguments = sys.argv[1:]
root = Path("outputs/reports")
report = root / f"{label}-physical-observer.json"
marker = f"/runtime/{label}-observer-complete"
compose = ["docker", "compose", "exec", "-T", "gazebo"]
observer_command = [*compose, "python", "-", vehicle_id, f"/workspace/{report}", marker, strict]
mission_command = [
    sys.executable,
    "scripts/validation/end_to_end.py",
    "--vehicle-id",
    vehicle_id,
    *arguments,
]
with (
    (harness / "waypoint-observer.py").open() as source,
    (run / f"{label}-observer.log").open("w") as diagnostics,
):
    observer = subprocess.Popen(
        observer_command, stdin=source, stdout=diagnostics, stderr=subprocess.STDOUT
    )
    try:
        deadline = time.monotonic() + 25
        while not report.with_suffix(".ready").is_file():
            if observer.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("Gazebo contact observer did not become ready")
            time.sleep(0.2)
        mission = subprocess.run(mission_command, timeout=285)
    finally:
        subprocess.run([*compose, "touch", marker], check=True, timeout=12)
        observer.wait(timeout=12)
observations = json.loads(report.read_text())
passed = (
    mission.returncode == 0
    and observer.returncode == 0
    and observations["contacts"] == 0
    and observations["minimum_clearance_m"] is not None
    and observations["minimum_clearance_m"] >= 0.25
    and (strict == "false" or observations["airborne_tracking"]["passed"])
)
result = {
    "passed": passed,
    "flight_exit": mission.returncode,
    "observer_report": str(report),
    "observer_command": observer_command,
    "mission_command": mission_command,
}
write_json_atomic(root / f"{label}-combined.json", result)
print(json.dumps(result), flush=True)
raise SystemExit(0 if passed else 1)
