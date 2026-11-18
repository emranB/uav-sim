"""Observe fault-free inspection missions; no vision-based flight control is used."""

import argparse
import json
import math
import time
import urllib.error
import urllib.request
from pathlib import Path

from testing.simulation.tracking import TrackingContinuity


def request(resource: str, body: dict | None = None) -> dict:
    command = urllib.request.Request(
        f"http://127.0.0.1:18080/{resource}",
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json", "X-Request-Source": "inspection-validation"},
    )
    try:
        with urllib.request.urlopen(command, timeout=80 if body is not None else 5) as response:
            result = json.load(response)
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"{resource}: {error.read().decode()}") from error
    if result.get("accepted") is False:
        raise RuntimeError(f"{resource}: {result.get('error', result)}")
    return result


def fleet() -> dict:
    return {item["vehicle_id"]: item for item in request("vehicles")["items"]}


def wait_ready(identifiers: list[str], timeout_seconds: float = 60) -> None:
    deadline = time.monotonic() + timeout_seconds
    last_error = "Controller readiness has not been observed"
    while time.monotonic() < deadline:
        try:
            current = fleet()
            if all(current.get(key, {}).get("ready") for key in identifiers) and all(
                request(f"perception?vehicle_id={key}")["camera"]["ready"] for key in identifiers
            ):
                return
        except (OSError, RuntimeError) as error:
            last_error = str(error)
        time.sleep(0.5)
    raise TimeoutError(f"Inspection vehicles did not become ready: {last_error}")


def wait_mapping_processing(identifiers: list[str], timeout_seconds: float = 120) -> None:
    """Separate vocabulary loading from observed flight-time tracking availability."""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        states = [request(f"perception/map?vehicle_id={key}") for key in identifiers]
        if all(
            state.get("processed_frames", 0) >= 3 and not state.get("stale", True)
            for state in states
        ):
            return
        time.sleep(0.5)
    raise TimeoutError("Live estimators did not finish startup before the flight deadline")


def fly(
    identifiers: list[str],
    timeout_seconds: float,
    scene_name: str = "inspection",
    require_mapping: bool = False,
) -> dict:
    evidence = {
        key: {"maximum_altitude_m": 0.0, "last_waypoint": -1, "airborne": False}
        for key in identifiers
    }
    homes = {}
    for key in identifiers:
        plan = json.loads(
            Path(f"configs/missions/{scene_name}-{key}.plan").read_text(encoding="utf-8")
        )
        homes[key] = plan["mission"]["plannedHomePosition"][:2]
        request("mission", {**plan, "vehicle_id": key})
    request("start", {"target_ids": identifiers})
    deadline = time.monotonic() + timeout_seconds
    minimum_separation_m = None
    observations = {
        key: {"samples": 0, "fresh_tracked_samples": 0, "maximum_landmarks": 0}
        for key in identifiers
    }
    continuity = {key: TrackingContinuity() for key in identifiers}
    last_map_stamp = {key: -1 for key in identifiers}
    next_map_sample = 0.0
    while time.monotonic() < deadline:
        current = fleet()
        if require_mapping and time.monotonic() >= next_map_sample:
            for key in identifiers:
                mapping = request(f"perception/map?vehicle_id={key}")
                if current[key]["armed"] and current[key]["relative_altitude_m"] > 3:
                    continuity[key].observe(mapping)
                stamp = mapping.get("simulation_timestamp_ns", -1)
                if stamp != last_map_stamp[key]:
                    last_map_stamp[key] = stamp
                    item = observations[key]
                    item["samples"] += 1
                    item["fresh_tracked_samples"] += int(mapping.get("ready", False))
                    item["maximum_landmarks"] = max(
                        item["maximum_landmarks"],
                        len(mapping.get("landmarks", [])),
                    )
            next_map_sample = time.monotonic() + 2
        scene = request("scene")
        positions = {item["vehicle_id"]: item["position_enu_m"] for item in scene["poses"]}
        if len(identifiers) == 2 and all(key in positions for key in identifiers):
            distance = math.dist(positions[identifiers[0]], positions[identifiers[1]])
            minimum_separation_m = (
                distance if minimum_separation_m is None else min(minimum_separation_m, distance)
            )
        for key in identifiers:
            sample = current[key]
            if not sample["connected"]:
                raise RuntimeError(f"{key} disconnected during inspection")
            item = evidence[key]
            item["maximum_altitude_m"] = max(
                item["maximum_altitude_m"], sample["relative_altitude_m"]
            )
            item["airborne"] |= sample["armed"] and sample["relative_altitude_m"] > 3
            item["last_waypoint"] = max(
                item["last_waypoint"],
                sample["messages"].get("MISSION_ITEM_REACHED", {}).get("seq", -1),
            )
        if all(evidence[key]["airborne"] and not current[key]["armed"] for key in identifiers):
            for key in identifiers:
                if require_mapping:
                    observed = observations[key]
                    if observed["fresh_tracked_samples"] < 5 or not observed["maximum_landmarks"]:
                        raise RuntimeError(
                            f"{key}: insufficient live mapping observations: {observed}"
                        )
                    observed["fresh_tracked_fraction"] = observed["fresh_tracked_samples"] / max(
                        1, observed["samples"]
                    )
                if evidence[key]["last_waypoint"] < 4:
                    raise RuntimeError(f"{key} did not reach the last inspection waypoint")
                latitude, longitude = homes[key]
                north_m = math.radians(current[key]["latitude_deg"] - latitude) * 6378137
                east_m = math.radians(current[key]["longitude_deg"] - longitude) * 6378137
                east_m *= math.cos(math.radians(latitude))
                evidence[key]["endpoint_error_m"] = math.hypot(east_m, north_m)
                if evidence[key]["endpoint_error_m"] > 3:
                    raise RuntimeError(f"{key} ended outside its home region")
            return {
                "vehicles": evidence,
                "minimum_observed_separation_m": minimum_separation_m,
                "scope": "independent waypoint execution; no swarm or collision-avoidance claim",
                "mapping_observations": observations if require_mapping else None,
                "tracking_continuity": {key: value.result() for key, value in continuity.items()},
                "mapping_scope": "availability sampling, not an accuracy guarantee",
            }
        time.sleep(0.5)
    raise TimeoutError(f"Inspection flight deadline exceeded: {evidence}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=["individual", "concurrent", "all"], default="all")
    parser.add_argument("--timeout-seconds", type=float, default=300)
    parser.add_argument("--scene", choices=["inspection", "waterfront"], default="inspection")
    parser.add_argument("--require-live-mapping", action="store_true")
    parser.add_argument("--require-stable-mapping", action="store_true")
    arguments = parser.parse_args()
    if arguments.require_stable_mapping:
        arguments.require_live_mapping = True
    if not 30 <= arguments.timeout_seconds <= 900:
        parser.error("Use a flight deadline of 30..900 seconds")
    destination = Path("outputs/reports") / f"{arguments.scene}-flight-{time.time_ns()}.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, "passed": False, "checks": []}
    identifiers = ["vehicle_1", "vehicle_2"]
    started = False
    try:
        wait_ready(identifiers, 90)
        current = fleet()
        if set(current) != set(identifiers) or any(item["armed"] for item in current.values()):
            raise RuntimeError("Inspection validation requires exactly two disarmed seed vehicles")
        for key in identifiers:
            workspace = request(f"workspace?vehicle_id={key}")
            if workspace["world"].get("scene_name") != arguments.scene:
                raise RuntimeError(
                    f"Start the explicit {arguments.scene} scene before flight validation"
                )
            if not request(f"perception?vehicle_id={key}")["camera"]["ready"]:
                raise RuntimeError(f"{key} has no fresh stereo images")
        phases = []
        if arguments.phase in {"individual", "all"}:
            phases.extend([["vehicle_1"], ["vehicle_2"]])
        if arguments.phase in {"concurrent", "all"}:
            phases.append(identifiers)
        for targets in phases:
            request("reset", {"target_ids": identifiers})
            wait_ready(identifiers)
            if arguments.require_live_mapping:
                wait_mapping_processing(identifiers)
            started = True
            evidence = fly(
                targets,
                arguments.timeout_seconds,
                arguments.scene,
                arguments.require_live_mapping,
            )
            passed = not arguments.require_stable_mapping or all(
                item["passed"] for item in evidence["tracking_continuity"].values()
            )
            report["checks"].append({"targets": targets, "passed": passed, "evidence": evidence})
            if not passed:
                raise RuntimeError("Airborne tracking continuity acceptance failed; see evidence")
            destination.write_text(json.dumps(report, indent=2), encoding="utf-8")
            print(json.dumps({"targets": targets, "passed": True}), flush=True)
        report["passed"] = True
    except (OSError, ValueError, KeyError, RuntimeError, TimeoutError) as error:
        report["error"] = str(error)
    finally:
        if started:
            try:
                request("stop", {"target_ids": identifiers})
            except (OSError, RuntimeError) as error:
                report["cleanup_error"] = str(error)
                report["passed"] = False
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
