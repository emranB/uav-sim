"""Verify live observation, pause freshness and restart isolation without flight commands."""

import argparse
import json
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path

COMPOSE = [
    "docker",
    "compose",
    "-f",
    "compose.yaml",
    "-f",
    "deploy/compose/inspection.yaml",
    "-f",
    "deploy/compose/mapping.yaml",
    "-f",
    "deploy/compose/waterfront.yaml",
]


def request(resource: str, body: dict | None = None) -> dict:
    operation = urllib.request.Request(
        "http://127.0.0.1:18080/" + resource,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json", "X-Request-Source": "mapping-validation"},
    )
    try:
        with urllib.request.urlopen(operation, timeout=40) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        payload = json.load(error)
        if payload.get("code") == "CAPABILITY_UNAVAILABLE":
            return payload
        raise RuntimeError(f"{resource}: {payload}") from error


def wait_observations(
    identifiers: list[str],
    timeout_seconds: float = 120,
    require_tracking: bool = False,
) -> dict:
    deadline = time.monotonic() + timeout_seconds
    latest = {}
    while time.monotonic() < deadline:
        latest = {key: request(f"perception/map?vehicle_id={key}") for key in identifiers}
        if all(
            item.get("processed_frames", 0) >= 3
            and not item.get("stale", True)
            and (not require_tracking or (item.get("ready") and item.get("landmarks")))
            for item in latest.values()
        ):
            return latest
        time.sleep(0.5)
    raise TimeoutError(
        f"Mapping did not become ready: {[(key, item.get('error'), item.get('tracking_state')) for key, item in latest.items()]}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--restart", action="store_true")
    parser.add_argument("--require-tracking", action="store_true")
    arguments = parser.parse_args()
    destination = Path("outputs/reports") / f"live-mapping-{time.time_ns()}.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, "passed": False, "checks": []}
    paused = False
    try:
        deadline = time.monotonic() + 60
        while True:
            try:
                fleet = request("vehicles")["items"]
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Control API did not become available within 60 seconds")
                time.sleep(0.5)
        if any(item["armed"] for item in fleet):
            raise RuntimeError("Pause and restart checks require disarmed vehicles")
        identifiers = [item["vehicle_id"] for item in fleet]
        initial = wait_observations(identifiers, require_tracking=arguments.require_tracking)
        report["checks"].append(
            {
                "name": "live_tracked_geometry"
                if arguments.require_tracking
                else "live_stereo_processing",
                "passed": True,
                "vehicles": {
                    key: {
                        "landmarks": len(value["landmarks"]),
                        "surfaces": len(value["surfaces"]),
                        "processed_frames": value["processed_frames"],
                        "session_id": value["session_id"],
                        "tracking_ready": value["ready"],
                    }
                    for key, value in initial.items()
                },
            }
        )
        request("pause", {"vehicle_id": identifiers[0]})
        paused = True
        time.sleep(7)
        stale = {key: request(f"perception/map?vehicle_id={key}") for key in identifiers}
        if not all(not value["ready"] and value["stale"] for value in stale.values()):
            raise RuntimeError("Paused image stream incorrectly remained ready")
        report["checks"].append({"name": "pause_marks_stale", "passed": True})
        request("resume", {"vehicle_id": identifiers[0]})
        paused = False
        wait_observations(identifiers, require_tracking=arguments.require_tracking)
        report["checks"].append({"name": "resume_restores_observation", "passed": True})
        for identifier in identifiers:
            response = request(
                "perception/check",
                {
                    "vehicle_id": identifier,
                    "capability_id": "recovery",
                },
            )
            if response.get("code") != "CAPABILITY_UNAVAILABLE":
                raise RuntimeError("Unsupported operation did not return its explicit error")
        report["checks"].append({"name": "unsupported_control_visible", "passed": True})
        if arguments.restart:
            subprocess.run([*COMPOSE, "restart", "live-mapping"], check=True, timeout=60)
            restarted = wait_observations(identifiers, require_tracking=arguments.require_tracking)
            current = {item["vehicle_id"]: item for item in request("vehicles")["items"]}
            for original in fleet:
                identifier = original["vehicle_id"]
                if current[identifier]["run_id"] != original["run_id"]:
                    raise RuntimeError("Estimator restart unexpectedly restarted a controller")
                if restarted[identifier]["session_id"] == initial[identifier]["session_id"]:
                    raise RuntimeError("Estimator restart reused its previous session")
            report["checks"].append({"name": "independent_estimator_restart", "passed": True})
        report["passed"] = True
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        report["error"] = str(error)
    finally:
        if paused:
            request("resume", {"vehicle_id": "vehicle_1"})
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
