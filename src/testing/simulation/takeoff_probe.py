"""Exercise only authored takeoff and landing to isolate the observer's scale transition."""

import argparse
import copy
import hashlib
import json
import time
from pathlib import Path

from testing.simulation.inspection import fleet, request, wait_mapping_processing, wait_ready
from testing.simulation.tracking import TrackingContinuity


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--vehicle-id", choices=["vehicle_1", "vehicle_2"], default="vehicle_1")
    args = parser.parse_args()
    args.destination.mkdir(parents=True, exist_ok=False)
    identifier = args.vehicle_id
    source = Path(f"configs/missions/waterfront-{identifier}.plan")
    plan = copy.deepcopy(json.loads(source.read_text()))
    plan["mission"]["items"] = [plan["mission"]["items"][0], plan["mission"]["items"][-1]]
    plan["mission"]["items"][1]["doJumpId"] = 2
    (args.destination / "probe.plan").write_text(json.dumps(plan, indent=2) + "\n")
    report = dict(
        schema_version=1,
        vehicle_id=identifier,
        passed=False,
        scope="Authored takeoff/landing only; no perception-driven control or waypoint acceptance",
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    )
    started = False
    airborne = False
    maximum_altitude = 0.0
    continuity = TrackingContinuity()
    try:
        if any(item["armed"] for item in fleet().values()):
            raise RuntimeError("Takeoff observer probe requires a disarmed fleet")
        if request(f"workspace?vehicle_id={identifier}")["world"].get("scene_name") != "waterfront":
            raise RuntimeError("Takeoff observer probe requires the waterfront scene")
        request("reset", {"target_ids": [identifier]})
        wait_ready([identifier])
        wait_mapping_processing([identifier])
        mapping = request(f"perception/map?vehicle_id={identifier}")
        report["session_id"] = mapping["session_id"]
        request("mission", {**plan, "vehicle_id": identifier})
        request("start", {"target_ids": [identifier]})
        started = True
        deadline = time.monotonic() + 180
        with (args.destination / "samples.jsonl").open("w") as stream:
            while time.monotonic() < deadline:
                current = fleet()[identifier]
                altitude = current["relative_altitude_m"]
                maximum_altitude = max(maximum_altitude, altitude)
                airborne |= current["armed"] and altitude > 3
                mapping = request(f"perception/map?vehicle_id={identifier}")
                if current["armed"] and altitude > 3:
                    continuity.observe(mapping)
                stream.write(
                    json.dumps(
                        dict(
                            wall_time=time.time(),
                            altitude_m=altitude,
                            armed=current["armed"],
                            map={
                                k: mapping.get(k)
                                for k in (
                                    "simulation_timestamp_ns",
                                    "position_valid",
                                    "map_id",
                                    "map_generation",
                                    "map_origin_id",
                                    "stale",
                                    "tracking_state",
                                    "processed_frames",
                                    "tracked_frames",
                                )
                            },
                        )
                    )
                    + "\n"
                )
                stream.flush()
                if airborne and not current["armed"]:
                    report.update(
                        completed_takeoff_and_landing=True,
                        maximum_altitude_m=maximum_altitude,
                        tracking=continuity.result(),
                    )
                    report["passed"] = maximum_altitude >= 7 and report["tracking"]["passed"]
                    break
                time.sleep(0.2)
            else:
                raise TimeoutError("Takeoff observer probe exceeded 180 seconds")
    except (OSError, ValueError, KeyError, RuntimeError, TimeoutError) as error:
        report["error"] = str(error)
    finally:
        if started:
            try:
                request("stop", {"target_ids": [identifier]})
            except (OSError, RuntimeError) as error:
                report["stop_error"] = str(error)
                report["passed"] = False
        (args.destination / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
