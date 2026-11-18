"""Viewer-only midpoint tracking for an explicitly selected pair of vehicles."""

import argparse
import json
import math
import signal
import time
import urllib.request
from collections.abc import Sequence


def group_camera_pose(positions: Sequence[Sequence[float]]) -> tuple[tuple, tuple]:
    """Return ENU focus and offset with a fixed viewing direction and separation margin."""
    if len(positions) != 2 or any(
        len(position) != 3 or any(not math.isfinite(value) for value in position)
        for position in positions
    ):
        raise ValueError("Shared camera requires two finite ENU vehicle positions")
    focus = tuple(sum(position[axis] for position in positions) / 2 for axis in range(3))
    radius = max(math.dist(position, focus) for position in positions)
    scale = max(1.0, (radius + 4.0) / 8.0)
    return focus, tuple(value * scale for value in (0.8, -1.2, 8.0))


def colour_vehicles(node: object, identities: list[str]) -> None:
    """Apply distinct high-contrast visual materials without modifying physical geometry."""
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.visual_pb2 import Visual

    from common.configuration.fleet import load_fleet

    definitions = {vehicle.vehicle_id: vehicle.model_name for vehicle in load_fleet()}
    acknowledged, scene = node.request("/world/default/scene/info", Empty(), Empty, Scene, 3000)
    if not acknowledged:
        raise TimeoutError("Shared viewer could not read the scene for vehicle materials")
    for identity, colour in zip(identities, ((0.05, 1.0, 0.6), (0.2, 0.5, 1.0))):
        name = definitions[identity]
        model = next((model for model in scene.model if model.name == name), None)
        if model is None:
            raise ValueError("Shared viewer vehicle is absent: " + identity)
        for link in model.link:
            for original in link.visual:
                visual = Visual(id=original.id)
                material = visual.material
                material.diffuse.r, material.diffuse.g, material.diffuse.b = colour
                material.diffuse.a = 1
                material.ambient.CopyFrom(material.diffuse)
                material.emissive.CopyFrom(material.diffuse)
                acknowledged, response = node.request(
                    "/world/default/visual_config", visual, Visual, Boolean, 3000
                )
                if not acknowledged or not response.data:
                    raise RuntimeError("Vehicle material was not acknowledged: " + identity)


def main() -> int:
    """Track fresh scene poses for a bounded interval; never issue vehicle commands."""
    # Transport imports are optional outside the Gazebo runtime.
    from gz.transport13 import Node

    from gazebo.control_worker import execute_camera

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicles", nargs=2, required=True)
    parser.add_argument("--duration-seconds", type=float, default=1200)
    parser.add_argument("--colour-vehicles", action="store_true")
    args = parser.parse_args()
    if len(set(args.vehicles)) != 2 or not 0 < args.duration_seconds <= 7200:
        parser.error("Choose two distinct vehicles and a duration in (0, 7200] seconds")
    stopped = False

    def stop(signum: int, frame: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    node = Node()
    if args.colour_vehicles:
        colour_vehicles(node, args.vehicles)
    deadline = time.monotonic() + args.duration_seconds
    previous = None
    try:
        while not stopped and time.monotonic() < deadline:
            with urllib.request.urlopen(
                "http://127.0.0.1:8080/scene?vehicle_id=" + args.vehicles[0], timeout=3
            ) as response:
                snapshot = json.load(response)
            received = snapshot.get("pose_received_monotonic", {})
            poses = {pose["vehicle_id"]: pose for pose in snapshot["poses"]}
            if all(
                identity in poses and time.monotonic() - received.get(identity, 0) < 2
                for identity in args.vehicles
            ):
                focus, offset = group_camera_pose(
                    [poses[identity]["position_enu_m"] for identity in args.vehicles]
                )
                # Keep a stable world direction; smooth translation and zoom together.
                target = (*focus, *offset)
                if previous is not None:
                    target = tuple(old + 0.4 * (new - old) for old, new in zip(previous, target))
                execute_camera(node, {"position_enu_m": target[:3], "offset_enu_m": target[3:]})
                previous = target
            time.sleep(0.6)
    finally:
        node = None
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
