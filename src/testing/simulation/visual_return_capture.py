"""Preserve a bounded, identical stereo return clip for offline transport comparison."""

import argparse
import json
import os
import time
from pathlib import Path

from gz.transport13 import Node

from common.configuration.cameras import StereoConfiguration
from common.configuration.fleet import load_fleet
from common.logging.storage import ensure_capacity, write_json_atomic
from gazebo.visual_stereo import LatestStereoPair


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("GZ_PARTITION") != "uav-visual-odometry":
        parser.error("Use the isolated return camera fixture")
    timeline = json.loads(args.timeline.read_text())
    if timeline.get("rig") != "visual_return_rig":
        parser.error("Expected the scripted return fixture timeline")
    settings = StereoConfiguration.configured(maximum_rate_hz=50)
    fleet = tuple(item for item in load_fleet() if item.enabled)
    if len(fleet) != 1:
        parser.error("Use a single vehicle fixture")
    pair_limit = 160
    ensure_capacity(
        args.output.parent, pair_limit * 2 * (settings.width * settings.height * 3 + 64)
    )
    args.output.mkdir(exist_ok=False)
    source = LatestStereoPair(Node(), fleet[0], settings, model_name=timeline["rig"])
    pairs = []
    previous = -1
    deadline = time.monotonic() + 35
    try:
        while len(pairs) < pair_limit:
            if time.monotonic() >= deadline:
                raise TimeoutError("Return capture exceeded its wall deadline")
            pair = source.wait_after(previous, 3)
            previous = pair.timestamp_ns
            if previous < timeline["return_start_ns"] - 1_000_000_000:
                continue
            left, right = (f"{len(pairs):04d}-{side}.ppm" for side in ("left", "right"))
            (args.output / left).write_bytes(pair.left)
            (args.output / right).write_bytes(pair.right)
            pairs.append(
                {
                    "simulation_timestamp_ns": previous,
                    "width": settings.width,
                    "height": settings.height,
                    "left": left,
                    "right": right,
                }
            )
    finally:
        source.close()
    write_json_atomic(
        args.output / "report.json",
        {
            "schema_version": 1,
            "source": "isolated scripted camera return",
            "return_start_ns": timeline["return_start_ns"],
            "calibration": {
                "baseline_m": settings.baseline_m,
                "horizontal_fov_rad": settings.horizontal_fov_rad,
                "distortion": "none",
            },
            "pairs": pairs,
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
