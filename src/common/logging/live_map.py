"""Validate bounded observation snapshots independently of the estimator process."""

import json
import math
import time
from pathlib import Path


def read_live_map(vehicle_id: str, runtime_root: Path = Path("/runtime")) -> dict:
    """A previous controller's map must never satisfy current tracking readiness."""
    directory = runtime_root if vehicle_id == "vehicle_1" else runtime_root / vehicle_id
    try:
        with (directory / "mapping.json").open("rb") as stream:
            contents = stream.read(3 * 1024 * 1024 + 1)
        if len(contents) > 3 * 1024 * 1024:
            raise ValueError("Live map exceeds 3 MiB")
        snapshot = json.loads(contents)
        if snapshot.get("schema_version") != 1 or snapshot.get("vehicle_id") != vehicle_id:
            raise ValueError("Live map schema or vehicle identity mismatch")
        generation = (directory / "px4.run").read_text(encoding="utf-8").strip()
        if snapshot.get("controller_run_id") != generation:
            raise ValueError("Live map belongs to a previous controller generation")
        for key, limit in (("trajectory", 10000), ("landmarks", 20000), ("surfaces", 6000)):
            points = snapshot.get(key, [])
            if not isinstance(points, list) or len(points) > limit:
                raise ValueError(f"Invalid point budget for {key}")
            for point in points:
                if (
                    not isinstance(point, list)
                    or len(point) != 3
                    or any(
                        type(value) not in {int, float} or not math.isfinite(value)
                        for value in point
                    )
                ):
                    raise ValueError(f"Invalid coordinates in {key}")
        age = time.time() - float(snapshot["received_wall_time"])
        if not math.isfinite(age) or type(snapshot.get("position_valid")) is not bool:
            raise ValueError("Live map requires finite timing and an explicit tracking result")
        if snapshot.get("map_frozen"):
            timestamp = snapshot.get("last_tracked_timestamp_seconds")
            if (
                snapshot["position_valid"]
                or type(timestamp) not in {int, float}
                or not math.isfinite(timestamp)
            ):
                raise ValueError(
                    "Frozen geometry requires a historical timestamp and no current pose"
                )
        if snapshot["position_valid"]:
            for key in ("timestamp_seconds", "valid_depth_fraction"):
                value = snapshot.get(key)
                if type(value) not in {int, float} or not math.isfinite(value):
                    raise ValueError(f"Live map requires a finite {key}")
            if not 0 <= snapshot["valid_depth_fraction"] <= 1:
                raise ValueError("Invalid measured depth fraction")
        snapshot["age_seconds"] = max(0, age)
        snapshot["ready"] = 0 <= age < 5 and snapshot.get("position_valid", False)
        snapshot["stale"] = not 0 <= age < 5
        snapshot["state"] = (
            "stale" if snapshot["stale"] else "tracking" if snapshot["ready"] else "untracked"
        )
        return snapshot
    except (OSError, ValueError, KeyError, TypeError) as error:
        return {
            "schema_version": 1,
            "vehicle_id": vehicle_id,
            "ready": False,
            "stale": True,
            "error": str(error),
        }
