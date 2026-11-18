"""Read-only observed replay data; no flight-control operations are exposed here."""

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from threading import Event
from typing import Protocol

from common.interfaces.health import SystemHealthEvidence
from common.interfaces.navigation import NavigationPose
from common.interfaces.scene import ScenePose


@dataclass(frozen=True)
class ReplayRecorderHealth:
    """Actual diagnostic writer state, independent of the replay capture writer."""

    available: bool
    path: str | None = None
    bytes_written: int = 0
    error: str | None = None
    phase: str = "unknown"
    telemetry_write_losses: int | None = None
    link_packet_losses: int | None = None
    camera_frame_losses: int | None = None
    camera_pair_drops: int | None = None


@dataclass(frozen=True)
class ReplayAidingEvidence:
    """Observed native fusion and requested fault state; None means unavailable."""

    received_monotonic: float | None = None
    simulation_timestamp_us: int | None = None
    gps_requested_enabled: bool | None = None
    gps_loss_injected: bool | None = None
    gps_fix_type: int | None = None
    satellites_used: int | None = None
    gnss_xy_active: bool | None = None
    gnss_velocity_active: bool | None = None
    gnss_height_active: bool | None = None
    vision_xy_active: bool | None = None
    vision_height_active: bool | None = None
    barometric_height_active: bool | None = None
    map_generation: int | None = None
    source_samples_fresh: bool | None = None
    visual_pose_received_monotonic: float | None = None
    visual_pose_healthy: bool | None = None
    xy_reset_counter: int | None = None
    z_reset_counter: int | None = None
    heading_reset_counter: int | None = None


@dataclass(frozen=True)
class ReplayFrameBinding:
    """Declared translation from estimated local ENU to the launch-site scene ENU."""

    controller_generation: str
    estimated_home_enu_m: tuple[float, float, float]
    scene_launch_enu_m: tuple[float, float, float]
    source: str = "navigation-start estimated pose at configured launch site"


@dataclass(frozen=True)
class ReplayWaypointEvent:
    """Observed application waypoint acknowledgement, separate from planned positions."""

    waypoint_id: str
    simulation_timestamp_seconds: float | None
    controller_generation: str


@dataclass(frozen=True)
class ReplayMissionReference:
    """Mission metadata only; planned waypoints never become observed trajectory samples."""

    execution_id: str
    name: str | None
    sha256: str | None
    state: str
    outcome: str | None
    planned_goal_offset_enu_m: tuple[float, float] | None = None
    current_waypoint: str | None = None
    waypoint_events: tuple[ReplayWaypointEvent, ...] = ()
    leg: str | None = None


@dataclass(frozen=True)
class ReplayObservation:
    """One selected vehicle's measured state, with truth kept as evaluation data."""

    vehicle_id: str
    label: str
    controller_generation: str | None
    world_epoch: int
    pose: NavigationPose | None
    pose_error: str | None
    recorder: ReplayRecorderHealth
    aiding: ReplayAidingEvidence | None = None
    frame_binding: ReplayFrameBinding | None = None
    mission: ReplayMissionReference | None = None
    evaluation_truth: ScenePose | None = None
    systems: tuple[SystemHealthEvidence, ...] = ()
    evaluation_truth_received_monotonic: float | None = None


class ReplayObservationSource(Protocol):
    """Observe application data without accessing command authority or process handles."""

    def vehicle_ids(self) -> tuple[str, ...]: ...
    def observe(self, vehicle_id: str) -> ReplayObservation: ...
    def context(self) -> dict[str, object]: ...


class ReplayRenderer(Protocol):
    """Optional offline renderer; reads only an exported immutable observation interval."""

    def availability(self) -> dict: ...
    def render(
        self,
        directory: Path,
        cancelled: Event,
        check: Callable[[], None],
        progress: Callable[[float], None],
    ) -> None: ...


@dataclass(frozen=True)
class ReplayCaptureLimits:
    """Finite per-capture limits; images/video are not captured by this service."""

    maximum_seconds: float = 600.0
    sample_period_seconds: float = 0.1
    maximum_bytes: int = 64 * 1024**2
    maximum_samples_per_vehicle: int = 20000
    maximum_vehicles: int = 8
    unretained_capture_count: int = 3
    maximum_store_bytes: int = 2 * 1024**3

    def __post_init__(self) -> None:
        for name in ("maximum_seconds", "sample_period_seconds"):
            if type(getattr(self, name)) not in {int, float}:
                raise ValueError(f"{name} must be numeric wall seconds")
        for name in (
            "maximum_bytes",
            "maximum_samples_per_vehicle",
            "maximum_vehicles",
            "unretained_capture_count",
            "maximum_store_bytes",
        ):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be integer")
        if not self.maximum_bytes <= self.maximum_store_bytes <= 16 * 1024**3:
            raise ValueError("Replay store budget must cover a capture and be at most 16 GiB")
        if not 1 <= self.maximum_seconds <= 1800:
            raise ValueError("Replay capture duration must be in 1..1800 wall seconds")
        if not 0.05 <= self.sample_period_seconds <= 1:
            raise ValueError("Replay capture period must be in 0.05..1 wall seconds")
        if not 1024 <= self.maximum_bytes <= 512 * 1024**2:
            raise ValueError("Replay capture byte budget must be in 1 KiB..512 MiB")
        if not 2 <= self.maximum_samples_per_vehicle <= 40000:
            raise ValueError("Replay capture sample limit must be in 2..40000")
        if not 1 <= self.maximum_vehicles <= 8 or not 1 <= self.unretained_capture_count <= 20:
            raise ValueError("Replay capture vehicle/retention limits are outside supported bounds")
