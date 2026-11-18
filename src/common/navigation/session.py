"""Exclusive, cancellable fixed-altitude simulation navigation and run records."""

import math
import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path

from common.interfaces.navigation import NavigationPose, NavigationVehicle, RangeSource
from common.interfaces.obstacles import StaleObservationError, StereoObstacleSource
from common.logging.storage import ensure_capacity, ensure_run_capacity, write_json_atomic
from common.navigation.freshness import ObservationRecovery, ObservationWaitPolicy
from common.navigation.mission import VisualMission
from common.navigation.occupancy import GridConfiguration, ObservedGrid
from common.navigation.progress import GoalProgress
from common.navigation.sensing import NavigationSensing

OUT_AND_BACK_MAXIMUM_WALL_SECONDS = 330


class NavigationSession:
    """One controller authority per run; no simulator scene/configuration input."""

    def __init__(
        self,
        vehicle_id: str,
        vehicle: NavigationVehicle,
        ranges: RangeSource,
        operation_lock: threading.Lock,
        directory: Path,
        stereo: StereoObstacleSource | None = None,
        freshness_policy: ObservationWaitPolicy | None = None,
        localization_guard: Callable[[str], tuple[int, int]] | None = None,
        localization_snapshot: Callable[[], dict] | None = None,
        stereo_reference_factory: Callable[[], NavigationVehicle] | None = None,
        grid_configuration: GridConfiguration | None = None,
    ) -> None:
        self.vehicle_id, self.vehicle, self.ranges = vehicle_id, vehicle, ranges
        self.grid_configuration = grid_configuration or GridConfiguration()
        self.freshness_policy = freshness_policy or ObservationWaitPolicy()
        self.stereo = stereo
        self.localization_guard = localization_guard
        self.localization_snapshot = localization_snapshot
        self.stereo_reference_factory = stereo_reference_factory
        self.operation_lock, self.directory = operation_lock, directory
        self.lock = threading.Lock()
        self.cancelled = threading.Event()
        self.thread: threading.Thread | None = None
        self.status = {
            "schema_version": 1,
            "state": "idle",
            "vehicle_id": vehicle_id,
            "available": True,
            "active": False,
        }

    def snapshot(self) -> dict:
        with self.lock:
            snapshot = dict(self.status)
            if self.localization_snapshot is not None:
                snapshot["visual_localization"] = self.localization_snapshot()
            if self.stereo is not None:
                snapshot["supported_obstacle_modes"] = ["range", "camera", "fused"]
                snapshot["localization_inputs"] = (
                    "Stereo horizontal map with PX4 height and attitude"
                    if snapshot.get("horizontal_reference") == "stereo-map"
                    else "PX4 local pose with required external vision aiding"
                    if self.localization_guard is not None
                    else "PX4 estimated local pose: GNSS/IMU/magnetometer/barometer"
                )
            if "obstacle_mode" in snapshot:
                live = snapshot.get("active", False) and snapshot.get("state") == "navigating"
                snapshot["observation_live"] = live
                snapshot["map_frozen"] = not live and bool(snapshot.get("occupied_enu_m"))
            return snapshot

    def update(self, **values: object) -> None:
        with self.lock:
            self.status.update(values)

    def start(self, body: dict, *, mission: VisualMission | None = None) -> dict:
        """Start one bounded run to a caller-supplied offset in local ENU metres."""
        mode = body.get("obstacle_mode", "range")
        return_to_start = body.get("return_to_start", False)
        if type(return_to_start) is not bool:
            raise ValueError("return_to_start must be a boolean")
        if return_to_start and self.localization_guard is None:
            raise ValueError("Out-and-back requires the isolated visual navigation profile")
        stereo_reference = body.get("stereo_horizontal_reference", False)
        if type(stereo_reference) is not bool:
            raise ValueError("stereo_horizontal_reference must be a boolean")
        if stereo_reference and (not return_to_start or self.stereo_reference_factory is None):
            raise ValueError("Stereo horizontal reference requires the isolated out-and-back")
        if mode not in {"range", "camera", "fused"}:
            raise ValueError("Choose range, camera or fused obstacle sensing")
        if mode != "range" and self.stereo is None:
            raise RuntimeError(
                "Camera/fused obstacle sensing unavailable; enable the stereo navigation overlay"
            )
        maximum_goal_distance_m = (
            mission.maximum_radius_m if mission is not None else 15 if mode == "range" else 10
        )
        goal = body.get("goal_offset_enu_m")
        if (
            not isinstance(goal, list)
            or len(goal) != 2
            or any(type(v) not in {int, float} or not math.isfinite(v) for v in goal)
            or not 2 <= math.hypot(*goal) <= maximum_goal_distance_m
        ):
            raise ValueError(
                f"Goal offset needs two finite ENU metres, distance 2..{maximum_goal_distance_m}m"
            )
        if mission is not None:
            if not return_to_start or mission.planned_distance_m > mission.maximum_total_distance_m:
                raise ValueError("Saved visual route requires return and a finite route budget")
            for waypoint in mission.outbound_waypoints:
                if any(
                    abs(v) > self.grid_configuration.half_width_m for v in waypoint.offset_enu_m
                ):
                    raise ValueError(
                        "Saved visual waypoint lies outside the navigation world bounds"
                    )
        ensure_run_capacity(self.directory)
        if not self.operation_lock.acquire(blocking=False):
            raise RuntimeError("Another mission, navigation or lifecycle operation owns authority")
        try:
            route_vehicle = self.stereo_reference_factory() if stereo_reference else self.vehicle
            pose = route_vehicle.navigation_pose()
            if mission is not None and (
                pose.simulation_timestamp_seconds is None
                or not math.isfinite(pose.simulation_timestamp_seconds)
            ):
                raise RuntimeError("Saved visual missions require a valid simulation clock")
            localization_generation = (
                self.localization_guard(pose.run_id)
                if self.localization_guard is not None and not stereo_reference
                else None
            )
            sensing = None
            if mode == "range":
                self.ranges.latest()
            else:
                sensing = NavigationSensing(
                    mode,
                    route_vehicle,
                    self.ranges,
                    self.stereo,
                    pose.run_id,
                    pose.position_enu_m[:2],
                )
                sensing.health()
            if pose.armed:
                raise RuntimeError(
                    "Navigation requires a ready disarmed vehicle at a clear launch site"
                )
            self.directory.mkdir(parents=True, exist_ok=True)
            self.cancelled.clear()
            self.status = {
                "schema_version": 1 if mode == "range" else 2,
                "vehicle_id": self.vehicle_id,
                "available": True,
                "active": True,
                "state": "starting",
                "run_id": uuid.uuid4().hex,
                "controller_run_id": pose.run_id,
                "goal_offset_enu_m": goal,
                "obstacle_mode": mode,
                "obstacle_sources_used": ["horizontal_range"]
                if mode == "range"
                else ["stereo"] + (["horizontal_range"] if mode == "fused" else []),
                "configuration": asdict(self.grid_configuration),
                "command_authority": "backend-mavlink-offboard",
                "altitude_above_start_m": 4.0,
                "progress_timeout_wall_seconds": 40.0,
                "progress_policy": "goal distance or new observed route cells",
                "maximum_navigation_wall_seconds": (
                    mission.maximum_navigation_wall_seconds
                    if mission is not None
                    else OUT_AND_BACK_MAXIMUM_WALL_SECONDS
                    if return_to_start
                    else 150
                    if mode == "range"
                    else 240
                ),
                "maximum_velocity_m_s": 0.8 if mode == "range" else 0.5,
                "velocity_change_m_s_per_wall_second": 0.5,
            }
            if sensing is not None:
                self.status.update(
                    freshness_policy=asdict(self.freshness_policy),
                    observation_stop_policy="zero velocity request; physical braking is not instantaneous",
                    stop_request_bypasses_route_velocity_slew=True,
                )
            if return_to_start:
                self.status.update(return_to_start=True, leg="outbound")
            if stereo_reference:
                self.status["horizontal_reference"] = "stereo-map"
            if mission is not None:
                self.status.update(
                    saved_mission=asdict(mission),
                    home_local_enu_m=list(pose.position_enu_m),
                    route_binding="saved planned-home offsets at this vehicle's estimated launch",
                    current_waypoint="takeoff",
                    waypoint_events=[],
                    outbound_waypoint_count=len(mission.outbound_waypoints),
                    planned_route_distance_m=mission.planned_distance_m,
                )
            self.thread = threading.Thread(
                target=self.run,
                args=(
                    pose,
                    goal,
                    sensing,
                    return_to_start,
                    localization_generation,
                    route_vehicle,
                    mission,
                ),
                daemon=True,
            )
            self.thread.start()
            return {"accepted": True, **self.snapshot()}
        except (ValueError, RuntimeError, OSError):
            self.operation_lock.release()
            raise

    def reset(self) -> None:
        """Clear runtime presentation after a disarmed reset, retaining run records."""
        if self.snapshot().get("active") or self.vehicle.navigation_armed():
            raise RuntimeError("Navigation reset requires an inactive, disarmed vehicle")
        self.ranges.set_enabled(True)
        if self.stereo is not None:
            self.stereo.reset("")
        self.cancelled.clear()
        with self.lock:
            self.status = {
                "schema_version": 1,
                "state": "idle",
                "vehicle_id": self.vehicle_id,
                "available": True,
                "active": False,
            }

    def preview_obstacles(self, mode: str) -> dict:
        """Inspect measured space without arming or issuing flight commands."""
        if self.snapshot().get("active"):
            raise RuntimeError("Use the active navigation snapshot while a run owns authority")
        pose = self.vehicle.navigation_pose()
        if mode == "range":
            grid = ObservedGrid(pose.position_enu_m[:2], self.grid_configuration)
            grid.observe(self.ranges.latest(), pose)
            metrics = {"obstacle_mode": "range", "obstacle_sources_used": ["horizontal_range"]}
        elif mode in {"camera", "fused"} and self.stereo is not None:
            sensing = NavigationSensing(
                mode, self.vehicle, self.ranges, self.stereo, pose.run_id, pose.position_enu_m[:2]
            )
            grid = sensing.update()
            metrics = sensing.metrics
        else:
            raise RuntimeError("Requested obstacle sensor combination is unavailable")
        return {
            "schema_version": 1,
            "vehicle_id": self.vehicle_id,
            "ready": True,
            **metrics,
            "position_enu_m": pose.position_enu_m,
            "observed_free_enu_m": [grid.point(c) for c in sorted(grid.free)],
            "occupied_enu_m": [grid.point(c) for c in sorted(grid.occupied)],
            "traversable_cells": len(grid.traversable()),
            "commands_issued": 0,
        }

    def configure_camera_input(self, settings: dict) -> dict:
        """Apply a declared simulation camera-input condition to this vehicle only."""
        if self.stereo is None:
            raise RuntimeError("Stereo obstacle source is not configured")
        if not isinstance(settings, dict):
            raise ValueError("Camera input settings must be a JSON object")
        self.stereo.configure(self.vehicle.navigation_pose().run_id, settings)
        return {"accepted": True, "vehicle_id": self.vehicle_id, "camera_input": settings}

    def sensing_ready(self, mode: str) -> dict:
        """Check source availability independently of execution acknowledgement."""
        pose = self.vehicle.navigation_pose()
        if mode == "range":
            self.ranges.latest()
        elif mode in {"camera", "fused"} and self.stereo is not None:
            NavigationSensing(
                mode, self.vehicle, self.ranges, self.stereo, pose.run_id, pose.position_enu_m[:2]
            ).health()
        else:
            raise RuntimeError("Requested obstacle sensor combination is unavailable")
        return {"ready": True, "obstacle_mode": mode}

    def cancel(self) -> dict:
        self.cancelled.set()
        return {"accepted": True, "vehicle_id": self.vehicle_id, "operation": "navigation/cancel"}

    def close(self) -> None:
        self.cancel()
        if self.thread:
            self.thread.join(timeout=20)
        self.ranges.close()

    def run(
        self,
        initial: NavigationPose,
        goal_offset: list[float],
        sensing: NavigationSensing | None = None,
        return_to_start: bool = False,
        localization_generation: tuple[int, int] | None = None,
        route_vehicle: NavigationVehicle | None = None,
        mission: VisualMission | None = None,
    ) -> None:
        """Execute observed paths only, then land; state records completion separately."""
        vehicle = route_vehicle or self.vehicle
        start = initial.position_enu_m
        altitude = start[2] + 4
        goal = start[0] + goal_offset[0], start[1] + goal_offset[1]
        returning = False
        waypoint_arrival_sim_seconds = None
        waypoint_index = 0
        waypoints = mission.outbound_waypoints if mission is not None else ()
        waypoint_hold_sim_seconds = waypoints[0].hold_sim_seconds if waypoints else 0.0
        # Leave margin between the estimate-based arrival and the 0.75 m truth
        # gate. The 0.4 m grid can place a goal centre up to 0.283 m away.
        arrival_radius_m = 0.4 if return_to_start else 0.65
        if return_to_start:
            self.update(estimated_arrival_radius_m=arrival_radius_m)
        target = [start[0], start[1], start[2]]
        target_lock = threading.RLock()
        streaming = threading.Event()
        stream_errors = []
        records = []
        outcome = "failed"
        error = None
        velocity_limit = 0.8 if sensing is None else 0.5
        mapped_observation = [(math.inf, None, None)]
        recovery = ObservationRecovery(self.vehicle_id, self.freshness_policy)

        def publish_recovery() -> None:
            self.update(**recovery.snapshot(time.monotonic()))
            if recovery.waiting:
                self.update(state="waiting_for_fresh_observation")

        def reject_observation(
            failure: StaleObservationError,
            origin: str,
            pose_simulation_seconds: float | None = None,
        ) -> None:
            with target_lock:
                recovery.reject(
                    failure,
                    time.monotonic(),
                    origin=origin,
                    pose_simulation_seconds=pose_simulation_seconds,
                )
            publish_recovery()

        def validate_pose(pose: NavigationPose) -> None:
            if mission is not None and (
                pose.simulation_timestamp_seconds is None
                or not math.isfinite(pose.simulation_timestamp_seconds)
            ):
                raise RuntimeError("Saved visual mission lost its simulation clock")
            if pose.run_id != initial.run_id:
                raise RuntimeError("Controller generation changed during navigation")
            if (
                self.localization_guard is not None
                and localization_generation is not None
                and self.localization_guard(pose.run_id) != localization_generation
            ):
                raise RuntimeError("Visual or PX4 local position origin changed")
            if sensing is not None:
                if time.monotonic() - pose.received_monotonic > 1.0:
                    raise RuntimeError("Fresh estimated local pose unavailable")
                if pose.simulation_timestamp_seconds is None:
                    raise RuntimeError(
                        "Estimated localization lacks a synchronized simulation clock"
                    )

        def validate_observation(observation: tuple, pose: NavigationPose) -> None:
            expiry, sensor_time, received = observation
            if sensor_time is None:
                return
            age = pose.simulation_timestamp_seconds - sensor_time
            if age < -0.08 or not math.isfinite(age):
                raise RuntimeError("Mapped sensor timestamp inconsistent with estimated clock")
            if age > 0.9 or time.monotonic() > expiry:
                raise StaleObservationError(
                    self.vehicle_id,
                    "Mapped sensor timestamp stale; refusing control",
                    timestamp_seconds=sensor_time,
                    sensor_age_sim_seconds=age,
                    delivery_age_wall_seconds=time.monotonic() - received,
                )

        if sensing is not None:
            publish_recovery()

        def stream() -> None:
            velocity = [0.0, 0.0, 0.0]
            maximum_sensor_age = 0.0
            maximum_delivery_age = 0.0
            mapped_control_updates = 0
            while not streaming.wait(0.1):
                try:
                    # Only cached evidence, a local pose snapshot and the UDP command
                    # share this lock. Perception and planning never run under it.
                    with target_lock:
                        point = tuple(target)
                        observation = mapped_observation[0]
                        control_pose = vehicle.navigation_pose()
                        validate_pose(control_pose)
                        if sensing is not None:
                            try:
                                validate_observation(observation, control_pose)
                            except StaleObservationError as failure:
                                recovery.reject(
                                    failure,
                                    time.monotonic(),
                                    origin="velocity_stream",
                                    pose_simulation_seconds=control_pose.simulation_timestamp_seconds,
                                )
                            recovery.check(time.monotonic())
                        if recovery.waiting or self.cancelled.is_set():
                            if recovery.waiting and (
                                not control_pose.armed or control_pose.flight_mode != "OFFBOARD"
                            ):
                                raise RuntimeError(
                                    "Healthy armed Offboard localization required during observation wait"
                                )
                            velocity = [0.0, 0.0, 0.0]
                            vehicle.send_navigation_velocity(tuple(velocity))
                            recovery.record_zero_command(
                                time.monotonic(),
                                control_pose.position_enu_m,
                                control_pose.simulation_timestamp_seconds,
                            )
                            if sensing is not None:
                                publish_recovery()
                            continue
                        if sensing is not None:
                            expiry, sensor_time, received = observation
                            if sensor_time is not None:
                                sensor_age = control_pose.simulation_timestamp_seconds - sensor_time
                                maximum_sensor_age = max(maximum_sensor_age, sensor_age)
                                maximum_delivery_age = max(
                                    maximum_delivery_age, time.monotonic() - received
                                )
                                mapped_control_updates += 1
                                self.update(
                                    maximum_control_sensor_age_sim_seconds=maximum_sensor_age,
                                    maximum_control_delivery_age_wall_seconds=maximum_delivery_age,
                                    mapped_control_updates=mapped_control_updates,
                                    control_checked_monotonic=time.monotonic(),
                                    control_sensor_simulation_seconds=sensor_time,
                                    control_pose_simulation_seconds=control_pose.simulation_timestamp_seconds,
                                    control_sensor_age_sim_seconds=sensor_age,
                                )
                        position = control_pose.position_enu_m
                        desired = [0.9 * (point[i] - position[i]) for i in range(3)]
                        horizontal = math.hypot(*desired[:2])
                        if horizontal > velocity_limit:
                            desired[:2] = [
                                value * velocity_limit / horizontal for value in desired[:2]
                            ]
                        desired[2] = max(-velocity_limit, min(velocity_limit, desired[2]))
                        # Route commands slew at 0.5m/s per wall second. A stop
                        # request goes directly to zero; PX4 still has to brake.
                        change = [desired[i] - velocity[i] for i in range(3)]
                        norm = math.sqrt(sum(value * value for value in change))
                        fraction = min(1.0, 0.05 / norm) if norm else 1.0
                        velocity = [velocity[i] + change[i] * fraction for i in range(3)]
                        vehicle.send_navigation_velocity(tuple(velocity))
                except (RuntimeError, OSError) as failure:
                    stream_errors.append(str(failure))
                    return

        sender = threading.Thread(
            target=stream, name=f"{self.vehicle_id}-velocity-stream", daemon=True
        )
        sender.start()
        destination = self.directory / (self.snapshot()["run_id"] + ".json")
        try:
            # PX4 requires >2Hz proof of life for more than one second before Offboard.
            warmup_deadline = time.monotonic() + 1.5
            while time.monotonic() < warmup_deadline:
                if self.cancelled.is_set():
                    outcome = "cancelled"
                    return
                if stream_errors:
                    raise RuntimeError(stream_errors[0])
                if sensing is not None:
                    sensing.health()
                time.sleep(0.1)
            if self.cancelled.is_set():
                outcome = "cancelled"
                return
            if self.localization_guard is not None:
                validate_pose(vehicle.navigation_pose())
            vehicle.begin_navigation()
            with target_lock:
                target[2] = altitude
            self.update(state="taking_off", goal_local_enu_m=goal)
            grid = ObservedGrid(start[:2], self.grid_configuration)
            maximum_wall_seconds = (
                mission.maximum_navigation_wall_seconds
                if mission is not None
                else OUT_AND_BACK_MAXIMUM_WALL_SECONDS
                if return_to_start
                else 150
                if sensing is None
                else 240
            )
            deadline = time.monotonic() + maximum_wall_seconds
            self.update(navigation_deadline_monotonic=deadline)
            last_plan = 0.0
            # Full-speed stereo measurements can already consume most of the
            # 0.9 simulation-second budget. Refresh their map before the old
            # evidence expires; keep the accepted range-only cadence unchanged.
            planning_interval_seconds = 0.4 if sensing is None else 0.1
            no_path_since = None
            navigating = False
            progress = None
            mode_deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                ensure_capacity(self.directory, required_bytes=1024**2)
                if self.cancelled.is_set():
                    outcome = "cancelled"
                    break
                if stream_errors:
                    raise RuntimeError(stream_errors[0])
                pose = vehicle.navigation_pose()
                validate_pose(pose)
                recovery.check(time.monotonic())
                scan = self.ranges.latest() if sensing is None else None
                if pose.run_id != initial.run_id:
                    raise RuntimeError("Controller generation changed during navigation")
                if pose.flight_mode != "OFFBOARD":
                    if time.monotonic() > mode_deadline:
                        raise RuntimeError(
                            "Offboard authority lost; refusing to compete with another mode"
                        )
                    time.sleep(0.05)
                    continue
                position = pose.position_enu_m
                if abs(position[2] - altitude) < 0.3:
                    navigating = True
                if (
                    navigating
                    and not recovery.waiting
                    and math.dist(position[:2], goal) < arrival_radius_m
                ):
                    if return_to_start and not returning:
                        if mission is not None and waypoint_arrival_sim_seconds is None:
                            waypoint_arrival_sim_seconds = pose.simulation_timestamp_seconds
                            waypoint_id = f"outbound-{waypoint_index + 1}"
                            self.update(
                                current_waypoint=waypoint_id,
                                waypoint_events=[
                                    *self.snapshot()["waypoint_events"],
                                    {
                                        "waypoint_id": waypoint_id,
                                        "simulation_timestamp_seconds": waypoint_arrival_sim_seconds,
                                        "controller_run_id": pose.run_id,
                                    },
                                ],
                            )
                        holding = (
                            mission is not None
                            and pose.simulation_timestamp_seconds
                            < waypoint_arrival_sim_seconds + waypoint_hold_sim_seconds
                        )
                        if holding:
                            self.update(current_waypoint=f"outbound-{waypoint_index + 1}-hold")
                        else:
                            if mission is not None and waypoint_index + 1 < len(waypoints):
                                waypoint_index += 1
                                waypoint = waypoints[waypoint_index]
                                goal = tuple(start[i] + waypoint.offset_enu_m[i] for i in (0, 1))
                                waypoint_hold_sim_seconds = waypoint.hold_sim_seconds
                                waypoint_arrival_sim_seconds = None
                                progress = None
                                no_path_since = None
                                last_plan = 0.0
                                self.update(
                                    current_waypoint=f"outbound-{waypoint_index + 1}",
                                    goal_local_enu_m=list(goal),
                                )
                                with target_lock:
                                    target[:] = [position[0], position[1], altitude]
                                continue
                            returning = True
                            self.update(
                                leg="return",
                                current_waypoint="home",
                                outbound_arrival_enu_m=position,
                                outbound_arrival_simulation_seconds=pose.simulation_timestamp_seconds,
                                outbound_goal_enu_m=goal,
                                goal_local_enu_m=start[:2],
                            )
                            goal = start[:2]
                            progress = None
                            no_path_since = None
                            last_plan = 0.0
                            with target_lock:
                                target[:] = [position[0], position[1], altitude]
                            continue
                    else:
                        outcome = "reached"
                        if returning:
                            self.update(
                                return_arrival_enu_m=position,
                                return_arrival_simulation_seconds=pose.simulation_timestamp_seconds,
                            )
                        if mission is not None:
                            events = self.snapshot()["waypoint_events"]
                            self.update(
                                current_waypoint="home",
                                waypoint_events=[
                                    *events,
                                    {
                                        "waypoint_id": "home",
                                        "simulation_timestamp_seconds": pose.simulation_timestamp_seconds,
                                        "controller_run_id": pose.run_id,
                                    },
                                ],
                            )
                        self.update(
                            position_enu_m=position, goal_error_m=math.dist(position[:2], goal)
                        )
                        break
                if progress is not None and recovery.waiting:
                    if (
                        time.monotonic() - progress.last_progress_monotonic
                        >= progress.timeout_seconds
                    ):
                        raise RuntimeError("No progress within the unchanged progress deadline")
                if navigating and not recovery.waiting:
                    error_m = math.dist(position[:2], goal)
                    route_cell = grid.cell(position[:2])
                    observed_route_cell = route_cell if route_cell in grid.free else None
                    if progress is None:
                        progress = GoalProgress(
                            error_m,
                            time.monotonic(),
                            initial_route_cell=observed_route_cell,
                        )
                    elif progress.blocked(error_m, time.monotonic(), observed_route_cell):
                        outcome = "blocked"
                        self.update(
                            blocked_reason="No goal progress or new observed route cells for 40 wall seconds",
                            position_enu_m=position,
                            goal_error_m=error_m,
                        )
                        break
                if sensing is not None and not navigating and not recovery.waiting:
                    try:
                        sensing.health()
                        with target_lock:
                            mapped_observation[0] = (
                                (
                                    sensing.observation_deadline,
                                    sensing.observation_timestamp_seconds,
                                    sensing.observation_received_monotonic,
                                )
                                if hasattr(sensing, "observation_deadline")
                                else mapped_observation[0]
                            )
                    except StaleObservationError as failure:
                        reject_observation(failure, "sensing_health")
                if (
                    navigating or recovery.waiting
                ) and time.monotonic() - last_plan >= planning_interval_seconds:
                    last_plan = time.monotonic()
                    if sensing is None:
                        grid.observe(scan, pose)
                    else:
                        mapping_started = time.monotonic()
                        try:
                            grid = sensing.update()
                        except StaleObservationError as failure:
                            reject_observation(failure, "map_update")
                            time.sleep(0.05)
                            continue
                        next_observation = (
                            sensing.observation_deadline,
                            sensing.observation_timestamp_seconds,
                            sensing.observation_received_monotonic,
                        )
                        self.update(
                            **sensing.metrics, mapping_seconds=time.monotonic() - mapping_started
                        )
                    planning_started = time.monotonic()
                    path = grid.plan(position[:2], goal)
                    self.update(planning_seconds=time.monotonic() - planning_started)
                    holding_waypoint = (
                        mission is not None
                        and not returning
                        and waypoint_arrival_sim_seconds is not None
                        and pose.simulation_timestamp_seconds
                        < waypoint_arrival_sim_seconds + waypoint_hold_sim_seconds
                        and math.dist(position[:2], goal) < arrival_radius_m
                    )
                    # A one-cell plan is a verified stationary route during the
                    # saved hold. Commit its new observation through the same
                    # freshness checks used for a moving route.
                    if len(path) > 1 or (holding_waypoint and len(path) == 1):
                        no_path_since = None
                        # Follow one adjacent cell; no straight-line shortcut across an obstacle.
                        next_point = path[1] if len(path) > 1 else position[:2]
                        direction = (next_point[0] - path[0][0], next_point[1] - path[0][1])
                        for index in range(2, min(4, len(path))):
                            step = (
                                path[index][0] - path[index - 1][0],
                                path[index][1] - path[index - 1][1],
                            )
                            if any(abs(step[i] - direction[i]) > 1e-6 for i in (0, 1)):
                                break
                            next_point = path[index]
                        with target_lock:
                            if self.cancelled.is_set() or stream_errors:
                                continue
                            if sensing is not None:
                                latest_pose = vehicle.navigation_pose()
                                validate_pose(latest_pose)
                                try:
                                    validate_observation(next_observation, latest_pose)
                                except StaleObservationError as failure:
                                    reject_observation(
                                        failure,
                                        "route_commit",
                                        latest_pose.simulation_timestamp_seconds,
                                    )
                                    continue
                                if not recovery.route_ready(next_observation[1], time.monotonic()):
                                    continue
                                mapped_observation[0] = next_observation
                            target[:] = (
                                [*next_point, altitude]
                                if navigating
                                else [start[0], start[1], altitude]
                            )
                        if sensing is not None:
                            publish_recovery()
                        self.update(
                            state="navigating",
                            path_enu_m=path,
                            occupied_enu_m=[grid.point(c) for c in sorted(grid.occupied)],
                            observed_free_cells=len(grid.free),
                            observed_free_enu_m=[grid.point(c) for c in sorted(grid.free)],
                            scan_simulation_seconds=grid.timestamp_seconds,
                            position_enu_m=position,
                            goal_error_m=math.dist(position[:2], goal),
                        )
                    else:
                        with target_lock:
                            target[:] = [position[0], position[1], altitude]
                        if not recovery.waiting:
                            self.update(state="blocked", path_enu_m=[], position_enu_m=position)
                        if no_path_since is None:
                            no_path_since = time.monotonic()
                        if time.monotonic() - no_path_since > 5:
                            outcome = "blocked"
                            break
                    records.append(
                        {
                            "wall_time": time.time(),
                            "scan_simulation_seconds": grid.timestamp_seconds,
                            "position_enu_m": position,
                            "target_enu_m": list(target),
                            "goal_error_m": math.dist(position[:2], goal),
                            "occupied_cells": len(grid.occupied),
                            "free_cells": len(grid.free),
                            "sensing": dict(sensing.metrics) if sensing is not None else None,
                            "scan": None
                            if scan is None
                            else {
                                "angle_min_rad": scan.angle_min_rad,
                                "angle_step_rad": scan.angle_step_rad,
                                "maximum_range_m": scan.maximum_range_m,
                                "range_encoding": "maximum-range-no-return,null-invalid",
                                "ranges_m": [
                                    v
                                    if math.isfinite(v)
                                    else (scan.maximum_range_m if v == math.inf else None)
                                    for v in scan.ranges_m
                                ],
                                "yaw_enu_rad": pose.yaw_enu_rad,
                            },
                        }
                    )
                pause_seconds = 0.05
                if sensing is not None and last_plan and (navigating or recovery.waiting):
                    # Mapping can consume the entire planning interval. Poll only
                    # until the next update is due instead of adding a fixed delay.
                    until_next_plan = last_plan + planning_interval_seconds - time.monotonic()
                    pause_seconds = min(pause_seconds, max(0.0, until_next_plan))
                if pause_seconds > 0:
                    self.cancelled.wait(pause_seconds)
            else:
                raise TimeoutError(
                    f"Navigation exceeded its {maximum_wall_seconds}-second wall-clock limit"
                )
        except (OSError, ValueError, RuntimeError) as failure:
            error = str(failure)
        finally:
            # Stop Offboard ownership before requesting the existing acknowledged landing operation.
            streaming.set()
            sender.join(timeout=2)
            if sensing is not None:
                recovery.abort(time.monotonic(), error or outcome)
                publish_recovery()
            self.update(state="landing", outcome=outcome, error=error)
            try:
                vehicle.land()
                self.update(landing_acknowledged=True)
                deadline = time.monotonic() + 45
                while vehicle.navigation_armed():
                    if time.monotonic() > deadline:
                        raise TimeoutError("Landing did not complete within 45 seconds")
                    time.sleep(0.2)
                vehicle.end_navigation()
            except (OSError, RuntimeError, ValueError) as failure:
                error = f"{error or ''} Landing: {failure}".strip()
                outcome = "failed"
            self.update(
                state=outcome,
                active=False,
                outcome=outcome,
                error=error,
                samples=len(records),
                report=str(destination),
                finished_wall_time=time.time(),
            )
            try:
                write_json_atomic(destination, {**self.snapshot(), "samples": records})
                self.update(report_persisted=True)
            except OSError as failure:
                # Flight safety cleanup has completed; recording failure still makes
                # the operation unsuccessful and must remain visible to the caller.
                recording_error = (
                    f"{self.vehicle_id}: cannot save navigation record {destination}: {failure}"
                )
                self.update(
                    state="failed",
                    outcome="failed",
                    error=f"{error} Recording: {recording_error}" if error else recording_error,
                    report_persisted=False,
                )
            finally:
                self.operation_lock.release()
