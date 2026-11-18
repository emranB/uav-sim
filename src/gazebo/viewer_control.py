"""Viewer-only camera operations, isolated from the physics control transport."""

import math
import time

from common.interfaces.scene import SceneSource
from gazebo.camera import CAMERA_OFFSETS
from gazebo.transport import GazeboControlConnection


class GazeboViewer:
    def __init__(
        self,
        scene: SceneSource,
        model_name: str = "x500_0",
        vehicle_id: str = "vehicle_1",
        connection: GazeboControlConnection | None = None,
    ) -> None:
        self.scene = scene
        self.model_name = model_name
        self.vehicle_id = vehicle_id
        self.owns_connection = connection is None
        self.connection = connection or GazeboControlConnection()

    def set_camera(
        self, preset: str, follow: bool, position_enu_m: tuple, offset_enu_m: tuple | None = None
    ) -> None:
        if preset not in {*CAMERA_OFFSETS, "custom"} or type(follow) is not bool:
            raise ValueError("Choose close, chase, top or wide and an explicit follow setting")
        offset = tuple(offset_enu_m) if offset_enu_m is not None else CAMERA_OFFSETS.get(preset)
        if (
            offset is None
            or len(offset) != 3
            or any(type(value) not in {int, float} or not math.isfinite(value) for value in offset)
            or not 0.8 <= math.sqrt(sum(value * value for value in offset)) <= 100
        ):
            raise ValueError(
                "Camera offset must have three finite coordinates at a distance of 0.8..100 m"
            )
        if len(position_enu_m) != 3 or not all(math.isfinite(value) for value in position_enu_m):
            raise ValueError("Camera needs a finite ENU vehicle position")
        self.connection.request(
            "camera",
            {
                "preset": preset,
                "follow": follow,
                "position_enu_m": position_enu_m,
                "offset_enu_m": offset,
            },
            timeout_seconds=24,
        )
        target = tuple(position + delta for position, delta in zip(position_enu_m, offset))
        started_at = time.monotonic()
        deadline = started_at + 10
        settled_samples = 0
        while True:
            viewer = self.scene.snapshot()["viewer"]
            observed = viewer.get("camera_position_enu_m")
            settled_samples = (
                settled_samples + 1
                if viewer["available"] and observed and math.dist(target, observed) < 0.01
                else 0
            )
            # An unchanged position can still have a running rotation animation.
            # Harmonic's move helper runs for 0.5 s; its completion callback must
            # finish before another request can replace the pending target.
            if settled_samples >= 3 and time.monotonic() - started_at >= 0.8:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Gazebo camera did not reach the requested view")
            time.sleep(0.1)
        # Moving and following both write camera pose. Enable follow only after the
        # move animation finishes, using observed pose rather than a fixed delay.
        if follow:
            self.set_follow(True, position_enu_m)

    def set_follow(self, follow: bool, position_enu_m: tuple) -> None:
        """Toggle tracking at the current camera position without reapplying a preset."""
        if type(follow) is not bool:
            raise ValueError("Follow must be true or false")
        snapshot = self.scene.snapshot()
        camera = snapshot["viewer"].get("camera_position_enu_m")
        if not snapshot["viewer"]["available"] or camera is None:
            raise RuntimeError("Camera pose is unavailable")
        offset = tuple(value - origin for value, origin in zip(camera, position_enu_m))
        # Gazebo's follow offset uses the target's local frame. Convert ENU once.
        pose = next(
            (pose for pose in snapshot.get("poses", []) if pose["vehicle_id"] == self.vehicle_id),
            None,
        )
        if pose:
            offset = tuple(value - origin for value, origin in zip(camera, pose["position_enu_m"]))
            x, y, z, w = pose["orientation_xyzw"]
            dx, dy, dz = offset
            offset = (
                (1 - 2 * y * y - 2 * z * z) * dx
                + (2 * x * y + 2 * z * w) * dy
                + (2 * x * z - 2 * y * w) * dz,
                (2 * x * y - 2 * z * w) * dx
                + (1 - 2 * x * x - 2 * z * z) * dy
                + (2 * y * z + 2 * x * w) * dz,
                (2 * x * z + 2 * y * w) * dx
                + (2 * y * z - 2 * x * w) * dy
                + (1 - 2 * x * x - 2 * y * y) * dz,
            )
        self.connection.request(
            "camera_follow",
            {
                "follow": follow,
                "offset": offset,
                "model_name": self.model_name,
            },
            timeout_seconds=12,
        )

    def close(self) -> None:
        if self.owns_connection:
            self.connection.close()
