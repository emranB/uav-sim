"""Read-only peer isolation plus owned stereo/backend restart and reset recovery checks."""

import hashlib
import json
import os
import subprocess
import time
import urllib.error
from collections.abc import Callable
from pathlib import Path

from common.logging.storage import write_json_atomic
from testing.simulation.navigation_acceptance import fleet, request, reset_vehicle, wait_ready


def preview(vehicle_id: str, mode: str = "fused") -> dict:
    """Read measured readiness with no arming or navigation commands."""
    result = request(f"navigation/obstacles?vehicle_id={vehicle_id}&obstacle_mode={mode}")
    return {
        key: value
        for key, value in result.items()
        if key not in {"occupied_enu_m", "observed_free_enu_m"}
    }


def await_condition(predicate: Callable[[], bool], timeout_seconds: float = 90) -> None:
    deadline = time.monotonic() + timeout_seconds
    last_error = None
    while time.monotonic() < deadline:
        try:
            if predicate():
                return
        except (OSError, RuntimeError, KeyError) as error:
            last_error = str(error)
        time.sleep(0.3)
    raise TimeoutError(f"Stereo recovery condition did not become ready: {last_error}")


def main() -> int:
    root = Path("outputs/reports")
    destination = root / f"stereo-recovery-{time.time_ns()}.json"
    report = {
        "schema_version": 1,
        "passed": False,
        "checks": [],
        "started_wall_time": time.time(),
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
    }
    historical = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in Path("outputs").glob("**/navigation/*.json")
    }
    try:
        wait_ready()
        before = fleet()
        initial = {key: preview(key) for key in before}
        request(
            "navigation/cameras",
            {"vehicle_id": "vehicle_1", "settings": {"fault": "invalid_depth"}},
        )
        request("navigation/ranges", {"vehicle_id": "vehicle_1", "enabled": False})

        def source_failed() -> bool:
            try:
                preview("vehicle_1", "camera")
            except urllib.error.HTTPError as error:
                return error.code == 409
            return False

        await_condition(source_failed, 10)
        peer = preview("vehicle_2")
        assert peer["source_session_id"] == initial["vehicle_2"]["source_session_id"], (
            "Peer stereo generation changed"
        )
        assert fleet()["vehicle_2"]["run_id"] == before["vehicle_2"]["run_id"], (
            "Peer controller changed"
        )
        report["checks"].append(
            {"name": "vehicle-scoped-camera-and-range-fault-isolation", "passed": True}
        )
        reset_vehicle()
        await_condition(lambda: preview("vehicle_1")["ready"])
        current = fleet()
        state = request("navigation?vehicle_id=vehicle_1")
        assert state["state"] == "idle" and not state["active"], "Reset left active navigation"
        assert not state.get("occupied_enu_m") and not state.get("path_enu_m"), (
            "Reset retained runtime map"
        )
        assert current["vehicle_1"]["run_id"] != before["vehicle_1"]["run_id"], (
            "Reset controller did not change"
        )
        assert current["vehicle_2"]["run_id"] == before["vehicle_2"]["run_id"], (
            "Reset changed peer controller"
        )
        assert preview("vehicle_2")["source_session_id"] == peer["source_session_id"], (
            "Reset changed peer stereo source"
        )
        report["checks"].append(
            {"name": "reset-clears-active-input-faults-and-map-preserves-peer", "passed": True}
        )
        generations = {key: value["run_id"] for key, value in current.items()}
        sources = {key: preview(key)["source_session_id"] for key in current}
        subprocess.run(
            ["docker", "compose", "restart", "navigation-stereo"], check=True, timeout=30
        )
        await_condition(
            lambda: all(
                preview(key)["source_session_id"] != source for key, source in sources.items()
            )
        )
        assert {key: value["run_id"] for key, value in fleet().items()} == generations, (
            "Stereo restart changed controllers"
        )
        sources = {key: preview(key)["source_session_id"] for key in current}
        report["checks"].append(
            {"name": "stereo-worker-restart-recovers-without-controller-restart", "passed": True}
        )
        subprocess.run(["docker", "compose", "restart", "control-panel"], check=True, timeout=30)
        wait_ready()
        await_condition(
            lambda: all(
                preview(key)["source_session_id"] == source for key, source in sources.items()
            )
        )
        assert {key: value["run_id"] for key, value in fleet().items()} == generations, (
            "Backend restart changed controllers"
        )
        report["checks"].append(
            {
                "name": "backend-restart-restores-both-modes-preserves-source-generations",
                "passed": True,
            }
        )
        assert all(
            Path(path).exists() and hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
            for path, digest in historical.items()
        ), "Historical navigation records changed"
        report["checks"].append(
            {
                "name": "historical-navigation-records-preserved",
                "passed": True,
                "files": len(historical),
            }
        )
        report["final_sources"] = {
            key: {mode: preview(key, mode) for mode in ["camera", "fused"]} for key in current
        }
        report["final_fleet"] = [
            {key: value[key] for key in ["vehicle_id", "ready", "armed", "run_id"]}
            for value in fleet().values()
        ]
        assert all(vehicle["ready"] and not vehicle["armed"] for vehicle in report["final_fleet"])

        def original_observation_ready() -> bool:
            for key in current:
                camera = request("perception?vehicle_id=" + key)["camera"]
                mapping = request("perception/map?vehicle_id=" + key)
                if not (camera["ready"] and mapping["ready"] and not mapping["stale"]):
                    return False
            return True

        await_condition(original_observation_ready, 120)
        report["original_observation"] = {
            key: {
                field: value
                for field, value in request("perception/map?vehicle_id=" + key).items()
                if field
                in {
                    "ready",
                    "stale",
                    "position_valid",
                    "surface_backend",
                    "session_id",
                    "processed_frames",
                }
            }
            for key in current
        }
        report["checks"].append(
            {
                "name": "original-camera-and-observer-readiness-restored",
                "passed": True,
                "scope": "Stationary readiness; previous authored flight evidence retained separately",
            }
        )
        await_condition(lambda: request("scene").get("viewer", {}).get("available", False), 60)
        request("viewer/camera", {"vehicle_id": "vehicle_1", "preset": "close", "follow": True})
        await_condition(
            lambda: (
                request("scene")["viewer"].get("following")
                and request("scene")["viewer"].get("target") == "x500_0"
            ),
            30,
        )
        report["viewer"] = request("scene")["viewer"]
        report["checks"].append(
            {"name": "viewer-ready-and-following-restored-focus", "passed": True}
        )
        desktop = subprocess.run(
            [
                "python",
                "scripts/validation/viewer_restart.py",
                "--service",
                "desktop",
                "--port",
                os.environ.get("BACKEND_PORT", "18080"),
                "--unique-report",
            ],
            text=True,
            capture_output=True,
            timeout=120,
        )
        desktop_summary = json.loads(desktop.stdout.strip().splitlines()[-1])
        report["checks"].append(
            {
                "name": "desktop-restart-loads-mode-controls-and-preserves-native-views",
                "passed": desktop.returncode == 0 and desktop_summary["passed"],
                "report": desktop_summary["report"],
            }
        )
        assert report["checks"][-1]["passed"], desktop_summary
        report["passed"] = True
    except (
        OSError,
        RuntimeError,
        ValueError,
        KeyError,
        AssertionError,
        subprocess.SubprocessError,
    ) as error:
        report["error"] = str(error)
    finally:
        for route, body in [
            ("navigation/cameras", {"vehicle_id": "vehicle_1", "settings": {"fault": "none"}}),
            ("navigation/ranges", {"vehicle_id": "vehicle_1", "enabled": True}),
        ]:
            try:
                request(route, body)
            except OSError as error:
                report.update(passed=False, cleanup_error=str(error))
        report["finished_wall_time"] = time.time()
        write_json_atomic(destination, report)
        print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
