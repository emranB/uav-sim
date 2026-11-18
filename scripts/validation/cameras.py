"""Verify live previews, target isolation and independent recorder restart."""

import argparse
import json
import os
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path


def read(resource: str) -> tuple[int, bytes]:
    port = int(os.getenv("BACKEND_PORT", "18080"))
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/{resource}", timeout=5) as response:
            return response.status, response.read(1024 * 1024 + 1)
    except urllib.error.HTTPError as error:
        return error.code, error.read(8192)


def fleet() -> dict:
    status, encoded = read("vehicles")
    if status != 200:
        raise RuntimeError("Vehicle inventory is unavailable")
    return {item["vehicle_id"]: item for item in json.loads(encoded)["items"]}


def wait_for_inventory() -> dict:
    deadline = time.monotonic() + 60
    last_error = "Waiting for the control API"
    while time.monotonic() < deadline:
        try:
            return fleet()
        except (OSError, RuntimeError) as error:
            last_error = str(error)
        time.sleep(0.5)
    raise TimeoutError(f"Control API did not become available: {last_error}")


def wait_for_cameras(identifiers: list[str]) -> dict:
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        states = {}
        for identifier in identifiers:
            status, encoded = read(f"perception?vehicle_id={identifier}")
            if status != 200:
                raise RuntimeError(f"{identifier}: capability inventory failed")
            states[identifier] = json.loads(encoded)["camera"]
        if all(item["ready"] for item in states.values()):
            return states
        time.sleep(0.5)
    raise TimeoutError("Rendered camera frames did not become fresh before the deadline")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--restart", action="store_true")
    arguments = parser.parse_args()
    report = {"schema_version": 1, "passed": False, "checks": []}
    destination = Path("outputs/reports") / f"live-cameras-{time.time_ns()}.json"
    try:
        before = wait_for_inventory()
        identifiers = list(before)
        initial = wait_for_cameras(identifiers)
        for identifier in identifiers:
            for side in ("left", "right"):
                status, image = read(f"perception/image?side={side}&vehicle_id={identifier}")
                header = image.split(b"\n", 3)
                dimensions = header[1].split() if len(header) == 4 else []
                accepted = (
                    status == 200
                    and len(header) == 4
                    and header[0] == b"P6"
                    and header[2] == b"255"
                    and dimensions in ([b"320", b"240"], [b"640", b"480"])
                    and len(header[3]) == int(dimensions[0]) * int(dimensions[1]) * 3
                )
                if not accepted:
                    details = (
                        image[:300].decode(errors="replace") if status != 200 else str(len(image))
                    )
                    raise RuntimeError(
                        f"{identifier}: invalid {side} preview; HTTP {status}; {details}"
                    )
            report["checks"].append({"name": "rendered_images", "vehicle_id": identifier})
        status, _ = read("perception/image?side=left&vehicle_id=unknown_vehicle")
        if status != 400:
            raise RuntimeError("Unknown target was not rejected")
        status, _ = read("perception/image?side=left")
        if status != 400:
            raise RuntimeError("Missing vehicle target was silently defaulted")
        if arguments.restart:
            compose = ["docker", "compose"]
            if not os.getenv("COMPOSE_FILE"):
                compose.extend(["-f", "compose.yaml", "-f", "deploy/compose/inspection.yaml"])
            subprocess.run([*compose, "restart", "camera-streams"], check=True, timeout=30)
            final = wait_for_cameras(identifiers)
            after = fleet()
            for identifier in identifiers:
                if before[identifier]["run_id"] != after[identifier]["run_id"]:
                    raise RuntimeError(f"Recorder restart also restarted {identifier}'s controller")
                if initial[identifier]["directory"] == final[identifier]["directory"]:
                    raise RuntimeError("Recorder restart did not establish a new recording session")
                if not (Path("outputs") / initial[identifier]["recording"]).is_file():
                    raise RuntimeError("Historical camera recording was not preserved")
            report["checks"].append(
                {"name": "independent_restart", "controller_runs_unchanged": True}
            )
        report["passed"] = True
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        report["error"] = str(error)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "report": str(destination)}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
