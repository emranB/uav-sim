"""Exercise real panel operations, input effects and saved-record round trips."""

import argparse
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

from common.logging.storage import write_json_atomic


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--vehicle-id", default="vehicle_1")
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    report = {"timestamp": time.time(), "checks": [], "passed": False}
    destination = root / "outputs/reports" / f"panels-{time.time_ns()}.json"
    destination.parent.mkdir(parents=True, exist_ok=True)

    def request(resource: str, body: dict | None = None) -> dict:
        if body is not None:
            body = {**body, "vehicle_id": arguments.vehicle_id}
        else:
            separator = "&" if "?" in resource else "?"
            resource += f"{separator}vehicle_id={arguments.vehicle_id}"
        query = urllib.request.Request(
            f"http://127.0.0.1:{arguments.port}/{resource}",
            data=json.dumps(body).encode() if body is not None else None,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(query, timeout=35) as response:
            return json.load(response)

    def check(name: str, condition: bool, evidence: object) -> None:
        report["checks"].append({"name": name, "passed": condition, "evidence": evidence})
        write_json_atomic(destination, report)
        print(json.dumps({"check": name, "passed": condition}), flush=True)
        if not condition:
            raise RuntimeError(f"Panel check failed: {name}")

    def wait_for(predicate, timeout_seconds: float = 45) -> dict:
        deadline = time.monotonic() + timeout_seconds
        while True:
            state = request("telemetry")
            if predicate(state):
                return state
            if time.monotonic() > deadline:
                raise TimeoutError("Panel state did not reach the expected value")
            time.sleep(0.5)

    try:
        deadline = time.monotonic() + 60
        while True:
            try:
                if request("health").get("real_simulator"):
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise TimeoutError("Real simulation did not become ready within 60 seconds")
            time.sleep(0.5)
        state = request("telemetry")
        world = request("workspace")["world"]
        origin_latitude, origin_longitude = world["latitude_deg"], world["longitude_deg"]
        if state["armed"]:
            raise RuntimeError("Panel validation requires a disarmed simulation")
        request("reset", {})
        state = wait_for(
            lambda s: (
                s["ready"]
                and not s["armed"]
                and abs(s["latitude_deg"] - origin_latitude) < 0.0001
                and abs(s["longitude_deg"] - origin_longitude) < 0.0001
            )
        )
        check(
            "configured-world-origin",
            abs(state["latitude_deg"] - origin_latitude) < 0.0001
            and abs(state["longitude_deg"] - origin_longitude) < 0.0001,
            {"latitude_deg": state["latitude_deg"], "longitude_deg": state["longitude_deg"]},
        )
        request("sensors/gnss", {"enabled": False})
        disabled = wait_for(lambda s: s["messages"].get("GPS_RAW_INT", {}).get("fix_type") == 0)
        check(
            "gnss-disable-affects-px4", not disabled["ready"], disabled["messages"]["GPS_RAW_INT"]
        )
        request("interference/clear", {})
        wait_for(lambda s: s["ready"])
        request("interference/gnss", {"satellites": 2})
        wait_for(lambda s: s["messages"].get("GPS_RAW_INT", {}).get("fix_type") == 0)
        check("gnss-degradation-affects-fix", True, {"satellites": 2})
        request("interference/clear", {})
        state = wait_for(lambda s: s["ready"])
        drops = state["communication"]["dropped_packets"]
        request("interference/link", {"latency_ms": 120, "loss_percent": 25, "seed": 17})
        time.sleep(3)
        link = request("telemetry")["communication"]
        check("real-link-packet-loss", link["dropped_packets"] > drops, link)
        request("interference/link", {"latency_ms": 0, "loss_percent": 100, "seed": 17})
        disconnected = wait_for(lambda s: not s["connected"], 12)
        check("link-outage-detected", not disconnected["ready"], disconnected["communication"])
        request("interference/clear", {})
        state = wait_for(lambda s: s["ready"])
        check("link-restored", True, state["communication"])
        original = request("vehicles/read", {})["parameters"]
        applied = request("vehicles/configure", {"cruise_speed_m_s": 3, "climb_speed_m_s": 1.5})
        check(
            "vehicle-parameters-acknowledged",
            applied["parameters"]["cruise_speed_m_s"] == 3
            and applied["parameters"]["climb_speed_m_s"] == 1.5,
            applied,
        )
        request("vehicles/configure", original)
        latitude, longitude = state["latitude_deg"], state["longitude_deg"]
        items = []
        for command, north, east, altitude in (
            (22, 0, 0, 8),
            (16, 0.00015, 0, 8),
            (16, 0.00015, 0.00015, 8),
            (21, 0, 0, 0),
        ):
            items.append(
                {
                    "type": "SimpleItem",
                    "command": command,
                    "frame": 3,
                    "autoContinue": True,
                    "doJumpId": len(items) + 1,
                    "params": [0, 0, 0, None, latitude + north, longitude + east, altitude],
                }
            )
        plan = {"fileType": "Plan", "version": 1, "mission": {"items": items}}
        request("mission", plan)
        name = f"Halifax validation {int(time.time())}"
        request("missions/save", {"name": name})
        check(
            "named-mission-listed", name in [p["name"] for p in request("missions")["items"]], name
        )
        request("missions/load", {"name": name})
        exported = request("missions/export", {"name": name})["mission"]
        check(
            "named-mission-round-trip",
            [p["command"] for p in exported["mission"]["items"]] == [22, 16, 16, 21],
            exported,
        )
        request("experiments/save", {"name": name})
        request("interference/gnss", {"satellites": 2})
        request("experiments/load", {"name": name})
        wait_for(lambda s: s["ready"])
        check(
            "experiment-restores-configuration",
            request("workspace")["environment"]["satellites"] == 10,
            name,
        )
        request("experiments/archive", {"name": name})
        request("missions/archive", {"name": name})
        check(
            "archive-removes-active-name",
            name not in [p["name"] for p in request("missions")["items"]],
            name,
        )
        logs = request("logs?source=px4&query=commander")
        check("searchable-console-recordings", bool(logs["records"]), len(logs["records"]))
        recordings = request("recordings")
        check(
            "persistent-recording-inventory",
            bool(recordings["telemetry"]) and bool(recordings["flight_logs"]),
            recordings,
        )
        newest = max(record["name"] for record in recordings["telemetry"])
        folder = root / "outputs"
        if arguments.vehicle_id != "vehicle_1":
            folder = folder / "vehicles" / arguments.vehicle_id
        folder = (folder / "telemetry").resolve()
        names = []
        # Exercise the API with bounded real fixtures. The newest recording may
        # still be growing, so it cannot be a reproducible comparison input.
        for record in sorted(recordings["telemetry"], key=lambda item: item["bytes"]):
            if record["name"] == newest or not 0 < record["bytes"] < 16 * 1024 * 1024:
                continue
            path = (folder / record["name"]).resolve()
            if not path.is_relative_to(folder):
                raise ValueError("Recording inventory returned an invalid fixture path")
            with path.open("rb") as stream:
                if b'"GLOBAL_POSITION_INT"' not in stream.read(1024 * 1024):
                    continue
            names.append(record["name"])
            if len(names) == 2:
                break
        if len(names) < 2:
            raise RuntimeError(
                "Comparison validation needs two completed position recordings under 16 MiB"
            )
        comparison = request("recordings/compare", {"left": names[0], "right": names[1]})
        check(
            "recording-comparison",
            comparison["comparison"]["left"]["position_samples"] > 0
            and comparison["comparison"]["right"]["position_samples"] > 0,
            comparison,
        )
        request("sensors/gnss", {"enabled": False})
        request("interference/link", {"latency_ms": 0, "loss_percent": 100, "seed": 17})
        wait_for(lambda s: not s["connected"], 12)
        request("reset", {})
        restored = wait_for(lambda s: s["ready"] and not s["armed"])
        settings = request("workspace")["environment"]
        check(
            "global-reset-clears-active-faults",
            settings["gnss_enabled"]
            and settings["satellites"] == 10
            and restored["communication"]["loss_percent"] == 0,
            settings,
        )
        report["passed"] = True
        return 0
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        report["error"] = str(error)
        if isinstance(error, urllib.error.HTTPError):
            report["error_response"] = error.read().decode(errors="replace")
        try:
            request("interference/clear", {})
        except (OSError, ValueError) as cleanup_error:
            report["cleanup_error"] = str(cleanup_error)
        return 1
    finally:
        write_json_atomic(destination, report)
        print(f"Report: {destination}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
