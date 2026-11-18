"""Validate real two-vehicle fault isolation and explicit collection commands."""

import argparse
import copy
import json
import math
import os
import time
import urllib.request
from pathlib import Path

from common.logging.storage import write_json_atomic


def request(path: str, body: dict | None = None) -> dict:
    query = urllib.request.Request(
        f"http://127.0.0.1:{int(os.getenv('BACKEND_PORT', '18080'))}/" + path,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(query, timeout=70) as response:
        return json.load(response)


def wait_for(predicate, timeout_seconds: float = 30) -> dict:
    deadline = time.monotonic() + timeout_seconds
    while True:
        try:
            fleet = {item["vehicle_id"]: item for item in request("vehicles")["items"]}
            if predicate(fleet):
                return fleet
        except OSError:
            # Readiness may be checked while the API is still accepting connections.
            pass
        if time.monotonic() >= deadline:
            raise TimeoutError("Fleet did not reach the expected observed state")
        time.sleep(0.5)


def fly_collection(
    fleet: dict, north_offset_m: float, east_offset_m: float, acceptance_radius_m: float
) -> dict:
    """Upload separated test routes, start the collection and observe both completions."""
    template = json.loads(
        Path("configs/missions/qgc-ui-validation.plan").read_text(encoding="utf-8")
    )
    for identifier, vehicle in fleet.items():
        latitude, longitude = vehicle["latitude_deg"], vehicle["longitude_deg"]
        north_latitude = latitude + north_offset_m / 111320
        east_longitude = longitude + east_offset_m / (111320 * math.cos(math.radians(latitude)))
        plan = copy.deepcopy(template)
        plan["mission"]["plannedHomePosition"] = [latitude, longitude, 20]
        for index, (command, lat, lon, altitude) in enumerate(
            (
                (22, latitude, longitude, 8),
                (16, north_latitude, longitude, 8),
                (16, north_latitude, east_longitude, 8),
                (21, latitude, longitude, 0),
            )
        ):
            item = plan["mission"]["items"][index]
            radius_m = acceptance_radius_m if command == 16 else 0.0
            item.update(command=command, frame=3, params=[0, radius_m, 0, None, lat, lon, altitude])
        request("mission", {**plan, "vehicle_id": identifier})
    targets = list(fleet)
    try:
        result = request("start", {"target_ids": targets})
        if not result["accepted"] or set(result["results"]) != set(targets):
            raise RuntimeError(f"Collection start did not acknowledge every vehicle: {result}")
        evidence = {
            identifier: {"maximum_altitude_m": 0, "last_waypoint": -1} for identifier in targets
        }
        simultaneously_airborne = False
        deadline = time.monotonic() + 240
        while True:
            current = {vehicle["vehicle_id"]: vehicle for vehicle in request("vehicles")["items"]}
            simultaneously_airborne |= all(
                current[identifier]["armed"] and current[identifier]["relative_altitude_m"] > 3
                for identifier in targets
            )
            for identifier in targets:
                sample = current[identifier]
                if not sample["connected"]:
                    raise RuntimeError(f"{identifier} disconnected during collection flight")
                evidence[identifier]["maximum_altitude_m"] = max(
                    evidence[identifier]["maximum_altitude_m"],
                    sample["relative_altitude_m"],
                )
                evidence[identifier]["last_waypoint"] = max(
                    evidence[identifier]["last_waypoint"],
                    sample["messages"].get("MISSION_ITEM_REACHED", {}).get("seq", -1),
                )
            if simultaneously_airborne and all(
                not current[identifier]["armed"] and evidence[identifier]["last_waypoint"] >= 2
                for identifier in targets
            ):
                return {"simultaneously_airborne": True, "vehicles": evidence}
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Collection flights did not complete: {evidence}")
            time.sleep(1)
    finally:
        request("stop", {"target_ids": targets})
        wait_for(lambda fleet: all(not vehicle["armed"] for vehicle in fleet.values()), 60)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fly", action="store_true", help="Execute both independent waypoint missions together."
    )
    parser.add_argument("--north-offset-m", type=float, default=11.132)
    parser.add_argument("--east-offset-m", type=float, default=7.92)
    parser.add_argument("--waypoint-acceptance-radius-m", type=float, default=0.5)
    arguments = parser.parse_args()
    if (
        not all(
            math.isfinite(v)
            for v in [
                arguments.north_offset_m,
                arguments.east_offset_m,
                arguments.waypoint_acceptance_radius_m,
            ]
        )
        or not 8 < math.hypot(arguments.north_offset_m, arguments.east_offset_m) <= 30
        or not 0.2 <= arguments.waypoint_acceptance_radius_m <= 2
    ):
        parser.error("Use finite ENU offsets, total distance 8..30m, and acceptance radius 0.2..2m")
    report = {
        "passed": False,
        "checks": [],
        "waypoint_offset_enu_m": [arguments.east_offset_m, arguments.north_offset_m],
        "waypoint_acceptance_radius_m": arguments.waypoint_acceptance_radius_m,
    }
    destination = Path("outputs/reports") / f"fleet-{time.time_ns()}.json"
    targets = ["vehicle_1", "vehicle_2"]

    def record(name: str, evidence: dict) -> None:
        report["checks"].append({"name": name, "evidence": evidence, "passed": True})
        print(json.dumps({"check": name, "passed": True}), flush=True)

    def fix(vehicle: dict) -> int:
        return vehicle.get("messages", {}).get("GPS_RAW_INT", {}).get("fix_type", 0)

    try:
        fleet = wait_for(
            lambda fleet: len(fleet) == 2 and all(vehicle["ready"] for vehicle in fleet.values())
        )
        if any(vehicle["armed"] for vehicle in fleet.values()):
            raise RuntimeError("Fleet isolation validation requires disarmed vehicles")
        if len({vehicle["mavlink_system_id"] for vehicle in fleet.values()}) != 2:
            raise RuntimeError("Fleet MAVLink identities overlap")
        poses = {pose["vehicle_id"]: pose["position_enu_m"] for pose in request("scene")["poses"]}
        if math.dist(poses["vehicle_1"], poses["vehicle_2"]) < 4:
            raise RuntimeError("Vehicle spawn separation is insufficient")
        record("two-independent-vehicles", {"positions_enu_m": poses})
        request("sensors/gnss", {"target_ids": ["vehicle_2"], "enabled": False})
        fleet = wait_for(
            lambda fleet: fix(fleet["vehicle_2"]) == 0 and fix(fleet["vehicle_1"]) >= 3
        )
        record(
            "gnss-fault-isolated-to-second-vehicle",
            {key: fix(value) for key, value in fleet.items()},
        )
        request("interference/clear", {"target_ids": targets})
        wait_for(lambda fleet: all(vehicle["ready"] for vehicle in fleet.values()))
        request(
            "interference/link",
            {"target_ids": ["vehicle_2"], "loss_percent": 100, "latency_ms": 0, "seed": 31},
        )
        fleet = wait_for(
            lambda fleet: not fleet["vehicle_2"]["connected"] and fleet["vehicle_1"]["ready"]
        )
        record(
            "link-fault-isolated-to-second-vehicle",
            {key: value["connected"] for key, value in fleet.items()},
        )
        request("interference/clear", {"target_ids": targets})
        wait_for(lambda fleet: all(vehicle["ready"] for vehicle in fleet.values()))
        result = request("interference/gnss", {"target_ids": targets, "satellites": 2})
        fleet = wait_for(lambda fleet: all(fix(vehicle) < 3 for vehicle in fleet.values()))
        if set(result["results"]) != set(targets) or not result["accepted"]:
            raise RuntimeError("Collection did not report each vehicle acknowledgement")
        record(
            "collection-fault-acknowledged-by-each-vehicle",
            {key: fix(value) for key, value in fleet.items()},
        )
        request("interference/clear", {"target_ids": targets})
        wait_for(lambda fleet: all(vehicle["ready"] for vehicle in fleet.values()))
        before_reset = {
            vehicle["vehicle_id"]: vehicle["run_id"] for vehicle in request("vehicles")["items"]
        }
        request("reset", {"target_ids": ["vehicle_2"]})
        fleet = wait_for(lambda fleet: all(vehicle["ready"] for vehicle in fleet.values()), 60)
        if fleet["vehicle_1"]["run_id"] != before_reset["vehicle_1"]:
            raise RuntimeError("Resetting the second vehicle restarted the first")
        if fleet["vehicle_2"]["run_id"] == before_reset["vehicle_2"]:
            raise RuntimeError("The second vehicle did not enter a new process generation")
        record(
            "second-vehicle-reset-preserves-first",
            {key: value["run_id"] for key, value in fleet.items()},
        )
        if arguments.fly:
            record(
                "simultaneous-independent-waypoint-flights",
                fly_collection(
                    fleet,
                    arguments.north_offset_m,
                    arguments.east_offset_m,
                    arguments.waypoint_acceptance_radius_m,
                ),
            )
            request("reset", {"target_ids": targets})
            wait_for(lambda fleet: all(vehicle["ready"] for vehicle in fleet.values()), 60)
        report["passed"] = True
        return 0
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        report["error"] = str(error)
        print(str(error), flush=True)
        return 1
    finally:
        try:
            request("interference/clear", {"target_ids": targets})
        except (OSError, ValueError) as error:
            report["cleanup_error"] = str(error)
        destination.parent.mkdir(parents=True, exist_ok=True)
        write_json_atomic(destination, report)
        print(f"Report: {destination.resolve()}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
