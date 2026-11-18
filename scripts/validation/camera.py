"""Verify camera presets against observed Gazebo GUI poses and tracking state."""

import json
import math
import os
import time
import urllib.error
import urllib.request
from pathlib import Path

from common.logging.storage import write_json_atomic


def request(path: str, body: dict | None = None) -> dict:
    if body is not None:
        body = {**body, "vehicle_id": "vehicle_1"}
    query = urllib.request.Request(
        f"http://127.0.0.1:{int(os.getenv('BACKEND_PORT', '18080'))}" + path,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json", "X-Request-Source": "camera-validation"},
    )
    with urllib.request.urlopen(query, timeout=40) as response:
        return json.load(response)


def main() -> int:
    report = {"passed": False, "checks": []}
    destination = Path("outputs/reports") / f"camera-{time.time_ns()}.json"
    try:
        for preset, follow in (
            ("close", True),
            ("chase", True),
            ("top", True),
            ("wide", False),
            ("chase", True),
        ):
            request(
                "/viewer/camera",
                {
                    "vehicle_id": "vehicle_1",
                    "preset": preset,
                    "follow": follow,
                },
            )
            deadline = time.monotonic() + 10
            while True:
                scene = request("/scene")
                viewer = scene["viewer"]
                if viewer["available"] and viewer["following"] == follow:
                    position = next(
                        pose["position_enu_m"]
                        for pose in scene["poses"]
                        if pose["vehicle_id"] == "vehicle_1"
                    )
                    camera = viewer["camera_position_enu_m"]
                    distance = math.dist(position, camera)
                    expected_distance = {"close": 2.0, "chase": 3.97, "top": 5.0, "wide": 11.09}[
                        preset
                    ]
                    expected_height = {"close": 0.8, "chase": 1.8, "top": 5.0, "wide": 5.0}[preset]
                    if (
                        abs(distance - expected_distance) < 1
                        and abs(camera[2] - position[2] - expected_height) < 0.8
                    ):
                        break
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"Camera did not reach {preset}, follow={follow}: {viewer}")
                time.sleep(0.2)
            evidence = {"preset": preset, "follow": follow, "observed": viewer}
            report["checks"].append(evidence)
            print(json.dumps(evidence), flush=True)
        request("/viewer/camera", {"offset_enu_m": [2, -3, 4], "follow": False})
        before = request("/scene")["viewer"]["camera_position_enu_m"]
        request("/viewer/follow", {"follow": True})
        time.sleep(0.5)
        followed = request("/scene")["viewer"]
        if not followed["following"] or math.dist(before, followed["camera_position_enu_m"]) > 0.3:
            raise RuntimeError("Enabling Follow changed the custom orbit position")
        request("/viewer/follow", {"follow": False})
        time.sleep(0.3)
        if request("/scene")["viewer"]["following"]:
            raise RuntimeError("Follow did not turn off")
        report["checks"].append({"custom_orbit_and_follow_preserve_position": True})
        request("/viewer/camera", {"preset": "close", "follow": True})
        report["passed"] = True
        return 0
    except (OSError, ValueError, RuntimeError, KeyError, StopIteration) as error:
        report["error"] = str(error)
        if isinstance(error, urllib.error.HTTPError):
            report["error_response"] = error.read().decode(errors="replace")
        print(str(error), flush=True)
        return 1
    finally:
        destination.parent.mkdir(parents=True, exist_ok=True)
        write_json_atomic(destination, report)
        print(f"Report: {destination.resolve()}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
