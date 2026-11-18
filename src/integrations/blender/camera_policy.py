"""Presentation-only bounded chase motion; recorded aircraft poses are never modified."""

import math
from dataclasses import dataclass

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class ChaseCameraState:
    position: Vector3
    target: Vector3
    heading_rad: float


def advance_chase_camera(
    position: Vector3,
    heading_rad: float,
    previous: ChaseCameraState | None,
    side_m: float,
    height_m: float,
    fps: int,
    back_m: float = 5.5,
) -> ChaseCameraState:
    """Keep world up stable and bound camera translation/yaw after initial placement."""
    if previous is not None:
        turn = (heading_rad - previous.heading_rad + math.pi) % (2 * math.pi) - math.pi
        heading_rad = previous.heading_rad + max(-0.8 / fps, min(0.8 / fps, turn))
    forward = (math.cos(heading_rad), math.sin(heading_rad), 0)
    side = (-forward[1], forward[0], 0)
    look_ahead_m = 0.4 if back_m >= 0 else -0.4
    target = tuple(
        position[i] + forward[i] * look_ahead_m + (0.4 if i == 2 else 0) for i in range(3)
    )
    desired = tuple(
        position[i] - forward[i] * back_m + side[i] * side_m + (height_m if i == 2 else 0)
        for i in range(3)
    )
    if previous is None:
        return ChaseCameraState(desired, target, heading_rad)
    alpha = 1 - math.exp(-1 / (fps * 0.45))
    delta = tuple((desired[i] - previous.position[i]) * alpha for i in range(3))
    length = math.sqrt(sum(value * value for value in delta))
    factor = min(1, 5 / fps / length) if length else 1
    camera = tuple(previous.position[i] + delta[i] * factor for i in range(3))
    aimed = tuple(previous.target[i] + (target[i] - previous.target[i]) * alpha for i in range(3))
    return ChaseCameraState(camera, aimed, heading_rad)
