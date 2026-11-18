"""Viewer-neutral scene state; coordinates use the Gazebo ENU world frame."""

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class ScenePose:
    vehicle_id: str
    simulation_time_seconds: float
    position_enu_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


class SceneSource(Protocol):
    def snapshot(self) -> dict: ...
    def begin_run(self) -> None: ...


class ViewerAdapter(Protocol):
    def set_camera(
        self, preset: str, follow: bool, position_enu_m: tuple, offset_enu_m: tuple | None = None
    ) -> None: ...
    def set_follow(self, follow: bool, position_enu_m: tuple) -> None: ...
