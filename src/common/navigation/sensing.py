"""Separate camera-only obstacle mapping from explicit conservative multi-sensor fusion."""

import math
import time

from common.configuration.navigation_stereo import NavigationStereoCalibration
from common.interfaces.navigation import NavigationPose, NavigationVehicle, RangeSource
from common.interfaces.obstacles import (
    StaleObservationError,
    StereoObstacleFrame,
    StereoObstacleSource,
)
from common.navigation.occupancy import GridConfiguration, ObservedGrid
from common.navigation.stereo_projection import (
    check_source_consistency,
    range_world_rays,
    stereo_world_rays,
)


class NavigationSensing:
    """Own per-source grids; one source's free rays cannot erase another's obstacles."""

    def __init__(
        self,
        mode: str,
        vehicle: NavigationVehicle,
        ranges: RangeSource,
        stereo: StereoObstacleSource,
        controller_run_id: str,
        origin_xy_m: tuple[float, float],
    ) -> None:
        if mode not in {"camera", "fused"}:
            raise ValueError("Unsupported stereo sensing mode")
        self.mode, self.vehicle, self.ranges, self.stereo = mode, vehicle, ranges, stereo
        self.controller_run_id = controller_run_id
        self.calibration = NavigationStereoCalibration()
        self.camera_grid = ObservedGrid(origin_xy_m, GridConfiguration())
        self.range_grid = ObservedGrid(origin_xy_m, GridConfiguration())
        self.grid = ObservedGrid(origin_xy_m, GridConfiguration())
        frame = stereo.latest(controller_run_id)
        self.source_session_id = frame.source_session_id
        vehicle.navigation_pose_at(frame.timestamp_seconds)
        if mode == "fused":
            ranges.latest()
        self.camera_frames = 0
        self.range_frames = 0
        self.metrics = {}
        self.observation_deadline = frame.captured_monotonic + 1.8
        self.observation_timestamp_seconds = frame.timestamp_seconds
        self.observation_received_monotonic = frame.captured_monotonic
        self._camera_rays = ()

    def health(self) -> tuple[StereoObstacleFrame, NavigationPose]:
        """Validate both requested sources and align capture to estimated localization."""
        try:
            frame = self.stereo.latest(self.controller_run_id)
        except StaleObservationError as failure:
            if failure.source_session_id != self.source_session_id:
                raise RuntimeError("Stereo obstacle source restarted; reset required") from failure
            current = self.vehicle.navigation_pose()
            aligned = self.vehicle.navigation_pose_at(failure.timestamp_seconds)
            if current.run_id != self.controller_run_id or aligned.run_id != self.controller_run_id:
                raise RuntimeError("Estimated-pose generation changed") from failure
            if current.simulation_timestamp_seconds is None:
                raise RuntimeError(
                    "Estimated localization lacks a synchronized simulation clock"
                ) from failure
            age = current.simulation_timestamp_seconds - failure.timestamp_seconds
            if not math.isfinite(age) or age < -0.08:
                raise RuntimeError(
                    "Stereo sensor timestamp inconsistent with estimated clock"
                ) from failure
            raise
        if frame.source_session_id != self.source_session_id:
            raise RuntimeError("Stereo obstacle source restarted; reset required")
        pose = self.vehicle.navigation_pose_at(frame.timestamp_seconds)
        if pose.run_id != self.controller_run_id:
            raise RuntimeError("Estimated-pose generation changed")
        current = self.vehicle.navigation_pose()
        if current.simulation_timestamp_seconds is None:
            raise RuntimeError("Estimated localization lacks a synchronized simulation clock")
        simulation_age = current.simulation_timestamp_seconds - frame.timestamp_seconds
        if not math.isfinite(simulation_age) or simulation_age < -0.08:
            raise RuntimeError("Stereo sensor timestamp inconsistent with estimated clock")
        if time.monotonic() - frame.captured_monotonic > 1.8:
            raise StaleObservationError(
                frame.vehicle_id,
                "Stereo measurement expired while awaiting estimated localization",
                timestamp_seconds=frame.timestamp_seconds,
                delivery_age_wall_seconds=time.monotonic() - frame.captured_monotonic,
            )
        if simulation_age > 0.9:
            raise StaleObservationError(
                frame.vehicle_id,
                "Stereo sensor timestamp stale or inconsistent with estimated clock: "
                f"age={simulation_age:.3f} simulation seconds, "
                f"delivery_age={time.monotonic() - frame.captured_monotonic:.3f} wall seconds, "
                f"processing={frame.processing_seconds:.3f} wall seconds, "
                f"frame_time={frame.timestamp_seconds:.3f}, "
                f"estimated_time={current.simulation_timestamp_seconds:.3f}",
                timestamp_seconds=frame.timestamp_seconds,
                sensor_age_sim_seconds=simulation_age,
                delivery_age_wall_seconds=time.monotonic() - frame.captured_monotonic,
            )
        scan = None
        if self.mode == "fused":
            scan = self.ranges.nearest(frame.timestamp_seconds, 0.5)
            if abs(scan.timestamp_seconds - frame.timestamp_seconds) > 0.5:
                raise RuntimeError("Stereo/range source skew exceeds 0.5 simulation seconds")
            range_age = current.simulation_timestamp_seconds - scan.timestamp_seconds
            if range_age < -0.08:
                raise RuntimeError("Range sensor timestamp inconsistent with estimated clock")
            if range_age > 0.9:
                raise StaleObservationError(
                    frame.vehicle_id,
                    "Range sensor timestamp stale",
                    timestamp_seconds=scan.timestamp_seconds,
                    sensor_age_sim_seconds=range_age,
                    delivery_age_wall_seconds=time.monotonic() - scan.received_monotonic,
                )
        self.observation_deadline = min(
            frame.captured_monotonic + 1.8,
            scan.received_monotonic + 1.5 if scan else float("inf"),
        )
        self.observation_timestamp_seconds = min(
            frame.timestamp_seconds, scan.timestamp_seconds if scan else float("inf")
        )
        self.observation_received_monotonic = min(
            frame.captured_monotonic, scan.received_monotonic if scan else float("inf")
        )
        return frame, pose

    def update(self) -> ObservedGrid:
        health_started = time.monotonic()
        frame, pose = self.health()
        source_health_seconds = time.monotonic() - health_started
        projection_started = time.monotonic()
        if frame.timestamp_seconds > self.camera_grid.timestamp_seconds:
            self._camera_rays = stereo_world_rays(frame, pose, self.calibration)
        projection_seconds = time.monotonic() - projection_started
        camera_rays = self._camera_rays
        consistency = {}
        scan = None
        if self.mode == "fused":
            scan = self.ranges.nearest(frame.timestamp_seconds, 0.5)
            if abs(scan.timestamp_seconds - frame.timestamp_seconds) > 0.5:
                raise RuntimeError("Stereo/range source skew exceeds 0.5 simulation seconds")
            range_pose = self.vehicle.navigation_pose_at(scan.timestamp_seconds)
            consistency = check_source_consistency(camera_rays, scan, range_pose)
            if scan.timestamp_seconds > self.range_grid.timestamp_seconds:
                self.range_grid.observe_rays(
                    scan.timestamp_seconds, range_world_rays(scan, range_pose)
                )
                self.range_frames += 1
        raster_started = time.monotonic()
        if frame.timestamp_seconds > self.camera_grid.timestamp_seconds:
            self.camera_grid.observe_rays(frame.timestamp_seconds, camera_rays)
            self.camera_frames += 1
        raster_seconds = time.monotonic() - raster_started
        self.grid.timestamp_seconds = max(
            frame.timestamp_seconds, scan.timestamp_seconds if scan else -1
        )
        self.grid.free = dict(self.camera_grid.free)
        self.grid.occupied = set(self.camera_grid.occupied)
        if self.mode == "fused":
            self.grid.free.update(self.range_grid.free)
            self.grid.occupied.update(self.range_grid.occupied)
        self.observation_deadline = min(
            frame.captured_monotonic + 1.8,
            scan.received_monotonic + 1.5 if scan else float("inf"),
        )
        self.observation_timestamp_seconds = min(
            frame.timestamp_seconds, scan.timestamp_seconds if scan else float("inf")
        )
        self.observation_received_monotonic = min(
            frame.captured_monotonic, scan.received_monotonic if scan else float("inf")
        )
        self.metrics = {
            "obstacle_mode": self.mode,
            "obstacle_sources_used": ["stereo"] + (["horizontal_range"] if scan else []),
            "camera_frames_used": self.camera_frames,
            "range_frames_used": self.range_frames,
            "camera_ray_count": len(camera_rays),
            "camera_projection_wall_seconds": projection_seconds,
            "camera_raster_wall_seconds": raster_seconds,
            "camera_age_wall_seconds": time.monotonic() - frame.captured_monotonic,
            "camera_sensor_age_sim_seconds": self.vehicle.navigation_pose().simulation_timestamp_seconds
            - frame.timestamp_seconds,
            "camera_delivery_to_result_seconds": frame.published_monotonic
            - frame.captured_monotonic,
            "camera_processing_seconds": frame.processing_seconds,
            "camera_backend": frame.backend,
            "camera_calibration_id": frame.calibration_id,
            "pose_alignment_sim_seconds": abs(
                pose.simulation_timestamp_seconds - frame.timestamp_seconds
            ),
            "range_age_wall_seconds": time.monotonic() - scan.received_monotonic if scan else None,
            "camera_input_quality": dict(frame.quality_settings),
            "camera_point_sampling_stride_px": frame.sampling_stride_px,
            "source_skew_sim_seconds": abs(scan.timestamp_seconds - frame.timestamp_seconds)
            if scan
            else None,
            "source_session_id": frame.source_session_id,
            "source_consistency": consistency,
            "localization_inputs": "PX4 estimated local position/attitude (GNSS, IMU, magnetometer, barometer)",
            "source_health_lookup_wall_seconds": source_health_seconds,
            "pose_time_alignment": "estimated history, <=0.08 simulation seconds",
        }
        return self.grid
