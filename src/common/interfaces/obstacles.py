"""Calibrated measured obstacle evidence; no simulator or transport message types."""

from dataclasses import dataclass
from typing import Protocol


class StaleObservationError(RuntimeError):
    """A correctly identified observation is too old; it must never guide movement.

    This excludes malformed input, changed generations/calibration, future timestamps
    and invalid pose alignment. Those failures remain fatal rather than retryable.
    """

    def __init__(
        self,
        vehicle_id: str,
        message: str,
        *,
        timestamp_seconds: float,
        sensor_age_sim_seconds: float | None = None,
        delivery_age_wall_seconds: float | None = None,
        source_session_id: str | None = None,
    ) -> None:
        super().__init__(message)
        self.vehicle_id = vehicle_id
        self.timestamp_seconds = timestamp_seconds
        self.sensor_age_sim_seconds = sensor_age_sim_seconds
        self.delivery_age_wall_seconds = delivery_age_wall_seconds
        self.source_session_id = source_session_id


@dataclass(frozen=True)
class StereoObstacleFrame:
    """Optical samples (right,down,forward metres, rig index), invalid pixels omitted."""

    vehicle_id: str
    controller_run_id: str
    source_session_id: str
    timestamp_seconds: float
    captured_monotonic: float
    published_monotonic: float
    calibration_id: str
    points: tuple[tuple[float, float, float, int], ...]
    valid_fractions: tuple[float, ...]
    processing_seconds: float
    backend: str
    quality_settings: tuple[tuple[str, str | int | float], ...] = ()
    sampling_stride_px: tuple[int, int] = (4, 4)


@dataclass(frozen=True)
class ObstacleRay:
    """ENU XY ray with a radial occupied interval and perpendicular lateral half-width.

    Distances are metres. Lateral uncertainty must not enlarge the radial interval.
    """

    origin_xy_m: tuple[float, float]
    direction_xy: tuple[float, float]
    free_distance_m: float
    occupied_start_m: float | None
    occupied_end_m: float | None
    lateral_uncertainty_m: float = 0.0


class StereoObstacleSource(Protocol):
    def latest(self, controller_run_id: str) -> StereoObstacleFrame: ...
    def configure(self, controller_run_id: str, settings: dict) -> None: ...
    def reset(self, controller_run_id: str) -> None: ...
