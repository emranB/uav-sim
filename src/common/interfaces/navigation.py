"""Fixed-altitude navigation inputs; metres, radians and monotonic receive time."""

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class RangeScan:
    """Horizontal FLU rays. Positive infinity is observed no-return; NaN is unknown.

    timestamp_seconds uses simulator sensor time. received_monotonic uses wall-clock
    monotonic seconds for freshness, independently of a paused simulation clock.
    """

    timestamp_seconds: float
    received_monotonic: float
    angle_min_rad: float
    angle_step_rad: float
    maximum_range_m: float
    ranges_m: tuple[float, ...]


@dataclass(frozen=True)
class NavigationPose:
    """PX4 estimated local ENU position and ENU yaw, never simulator model truth."""

    position_enu_m: tuple[float, float, float]
    yaw_enu_rad: float
    received_monotonic: float
    run_id: str
    armed: bool
    flight_mode: str
    simulation_timestamp_seconds: float | None = None
    roll_rad: float = 0.0
    pitch_rad: float = 0.0


class RangeSource(Protocol):
    def latest(self) -> RangeScan: ...
    def nearest(self, timestamp_seconds: float, maximum_skew_seconds: float) -> RangeScan: ...
    def set_enabled(self, enabled: bool) -> None: ...
    def close(self) -> None: ...


class NavigationVehicle(Protocol):
    def navigation_pose(self) -> NavigationPose: ...
    def navigation_pose_at(self, timestamp_seconds: float) -> NavigationPose: ...
    def send_navigation_velocity(self, velocity_enu_m_s: tuple[float, float, float]) -> None: ...
    def navigation_armed(self) -> bool: ...
    def begin_navigation(self) -> None: ...
    def end_navigation(self) -> None: ...
    def land(self) -> None: ...
