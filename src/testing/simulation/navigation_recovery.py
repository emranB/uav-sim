"""Validate disarmed navigation reset, observation restart and historical-record preservation."""

import datetime
import hashlib
import json
import subprocess
import time
from collections.abc import Callable
from pathlib import Path

from testing.simulation.inspection import request


def main() -> int:
    reports_directory = Path("outputs/reports")
    report = {
        "schema_version": 1,
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "passed": False,
        "checks": [],
    }

    def fleet() -> dict:
        return {v["vehicle_id"]: v for v in request("vehicles")["items"]}

    def maps() -> dict:
        return {k: request("perception/map?vehicle_id=" + k) for k in ["vehicle_1", "vehicle_2"]}

    def records() -> dict:
        return {
            str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in Path("outputs").glob("**/navigation/*.json")
        }

    def wait(predicate: Callable[[], bool], seconds: float = 120) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                if predicate():
                    return
            except (OSError, KeyError):
                pass
            time.sleep(0.5)
        raise TimeoutError("Recovery readiness did not reach required observed state")

    try:
        before = fleet()
        initial_maps = maps()
        history = records()
        if any((v["armed"] for v in before.values())):
            raise RuntimeError("Recovery checks require both vehicles disarmed")
        request("navigation/ranges", {"vehicle_id": "vehicle_1", "enabled": False})
        request("pause", {"vehicle_id": "vehicle_1"})
        request("reset", {"vehicle_id": "vehicle_1"})
        wait(
            lambda: (
                all((v["ready"] and (not v["armed"]) for v in fleet().values()))
                and fleet()["vehicle_1"]["run_id"] != before["vehicle_1"]["run_id"]
            )
        )
        wait(lambda: all((v.get("ready") and (not v.get("stale")) for v in maps().values())))
        after = fleet()
        after_maps = maps()
        state = request("navigation?vehicle_id=vehicle_1")
        assert after["vehicle_2"]["run_id"] == before["vehicle_2"]["run_id"], (
            "Peer controller reset"
        )
        assert initial_maps["vehicle_2"]["session_id"] == after_maps["vehicle_2"]["session_id"], (
            "Peer map session reset"
        )
        assert state == {
            "schema_version": 1,
            "state": "idle",
            "vehicle_id": "vehicle_1",
            "available": True,
            "active": False,
        }, state
        ready = request(
            "perception/check", {"vehicle_id": "vehicle_1", "capability_id": "obstacle_avoidance"}
        )
        assert ready["ready"], "Range subscription was not restored"
        report["checks"].append(
            {
                "name": "reset-from-pause-clears-range-fault-and-runtime",
                "passed": True,
                "snapshot": state,
                "peer_controller_and_map_preserved": True,
            }
        )
        assert all((records().get(p) == digest for (p, digest) in history.items())), (
            "History changed during reset"
        )
        report["checks"].append(
            {"name": "historical-native-records-preserved", "passed": True, "files": len(history)}
        )
        generations = {k: v["run_id"] for (k, v) in after.items()}
        subprocess.run(["docker", "compose", "restart", "control-panel"], check=True, timeout=45)
        wait(lambda: all((v["ready"] and (not v["armed"]) for v in fleet().values())))
        assert {k: v["run_id"] for (k, v) in fleet().items()} == generations, (
            "Backend restart changed controller generations"
        )
        wait(
            lambda: request(
                "perception/check",
                {"vehicle_id": "vehicle_1", "capability_id": "obstacle_avoidance"},
            )["ready"]
        )
        report["checks"].append(
            {"name": "backend-restart-preserves-controllers-and-restores-sensors", "passed": True}
        )
        for name, command in [
            ("camera-restart", ["python", "scripts/validation/cameras.py", "--restart"]),
            (
                "observation-restart-and-freshness",
                ["python", "scripts/validation/live_mapping.py", "--restart", "--require-tracking"],
            ),
        ]:
            result = subprocess.run(command, text=True, capture_output=True, timeout=240)
            (reports_directory / ("avoidance-final-" + name + ".log")).write_text(
                result.stdout + result.stderr
            )
            summary = json.loads(result.stdout.strip().splitlines()[-1])
            report["checks"].append(
                {
                    "name": name,
                    "passed": result.returncode == 0 and summary["passed"],
                    "report": summary["report"],
                    "command": command,
                }
            )
            assert report["checks"][-1]["passed"], summary
        wait(lambda: all((v["ready"] and (not v["armed"]) for v in fleet().values())))
        wait(lambda: all((v.get("ready") and (not v.get("stale")) for v in maps().values())))
        wait(lambda: request("scene").get("viewer", {}).get("available", False), 60)
        request("viewer/camera", {"vehicle_id": "vehicle_1", "preset": "close", "follow": True})
        wait(
            lambda: (
                request("scene")["viewer"].get("following")
                and request("scene")["viewer"].get("target") == "x500_0"
            ),
            30,
        )
        report["final_fleet"] = [
            {k: v.get(k) for k in ["vehicle_id", "ready", "armed", "run_id"]}
            for v in fleet().values()
        ]
        report["final_maps"] = {
            k: {
                n: v.get(n)
                for n in [
                    "ready",
                    "stale",
                    "position_valid",
                    "session_id",
                    "map_generation",
                    "surface_backend",
                ]
            }
            | {"landmarks": len(v.get("landmarks", [])), "surfaces": len(v.get("surfaces", []))}
            for (k, v) in maps().items()
        }
        report["viewer"] = request("scene")["viewer"]
        report["passed"] = True
    except (
        OSError,
        ValueError,
        RuntimeError,
        KeyError,
        AssertionError,
        subprocess.SubprocessError,
    ) as e:
        report["error"] = str(e)
    finally:
        try:
            request("navigation/ranges", {"vehicle_id": "vehicle_1", "enabled": True})
            request("resume", {"vehicle_id": "vehicle_1"})
        except (OSError, RuntimeError) as e:
            report["cleanup_error"] = str(e)
            report["passed"] = False
        (reports_directory / "avoidance-final-recovery.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        print(json.dumps(report), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
