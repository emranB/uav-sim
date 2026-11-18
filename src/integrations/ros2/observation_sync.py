"""Opt-in Jazzy timing boundary; no image processing, simulator truth or flight commands."""

import math
import threading
from collections import deque
from dataclasses import dataclass, replace

import message_filters
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException

from common.interfaces.navigation import NavigationPose


def ros_time(nanoseconds: int) -> Time:
    """Construct a simulation-domain ROS timestamp, never a wall-clock stamp."""
    return Time(nanoseconds=nanoseconds, clock_type=ClockType.ROS_TIME)


@dataclass(frozen=True)
class ObservationStamp:
    """Observed measurement timing; availability uses the caller's declared receive clock."""

    vehicle_id: str
    controller_run_id: str
    source_session_id: str
    calibration_id: str
    simulation_seconds: float
    available_seconds: float


@dataclass(frozen=True)
class SynchronizedObservation:
    """One immutable observation and its estimated pose in local ENU/FLU."""

    observation: ObservationStamp
    pose: NavigationPose
    aligned_at_seconds: float


class EstimatedObservationSynchronizer:
    """Cache unequal-rate estimates and interpolate through explicitly populated local tf2.

    No image/pose one-to-one synchronization is performed. The upstream worker already
    pairs its calibrated stereo images. This experiment receives completed observation
    stamps and PX4 estimates; it cannot repair an already stale producer output.
    """

    def __init__(self, vehicle_id: str, controller_run_id: str, calibration_id: str) -> None:
        self.vehicle_id = vehicle_id
        self.calibration_id = calibration_id
        self._lock = threading.RLock()
        self.reset(controller_run_id)

    def reset(self, controller_run_id: str) -> None:
        """Discard pending observations, estimates and transforms at a declared epoch reset."""
        with self._lock:
            self._controller_run_id = controller_run_id
            self._pose_input = message_filters.SimpleFilter()
            self._pose_cache = message_filters.Cache(self._pose_input, 192, allow_headerless=False)
            self._transforms = Buffer(cache_time=Duration(seconds=8))
            self._poses = {}
            self._pending = deque(maxlen=2)
            self._current_pose = None
            self._latest_observation = None
            self._source_session_id = None
            self._state = "waiting-for-estimates"
            self._accepted = 0
            self._dropped = 0

    def add_pose(self, pose: NavigationPose, now_seconds: float) -> None:
        """Insert an actual estimated pose at its simulation timestamp, never receipt time."""
        with self._lock:
            if pose.run_id != self._controller_run_id:
                self._state = "controller-generation-mismatch"
                self._latest_observation = None
                return
            timestamp = pose.simulation_timestamp_seconds
            if timestamp is None or not all(
                math.isfinite(value)
                for value in (
                    timestamp,
                    *pose.position_enu_m,
                    pose.yaw_enu_rad,
                    pose.roll_rad,
                    pose.pitch_rad,
                )
            ):
                raise ValueError("Estimated pose needs finite ENU/FLU values and simulation time")
            if self._current_pose and timestamp < self._current_pose.simulation_timestamp_seconds:
                self.reset(pose.run_id)
                self._state = "clock-jump-reset"
            stamp = ros_time(round(timestamp * 1e9)).to_msg()
            message = PoseStamped()
            message.header.stamp = stamp
            message.header.frame_id = f"{self.vehicle_id}/local_enu"
            message.pose.position.x, message.pose.position.y, message.pose.position.z = map(
                float, pose.position_enu_m
            )
            cr, sr = math.cos(pose.roll_rad / 2), math.sin(pose.roll_rad / 2)
            cp, sp = math.cos(pose.pitch_rad / 2), math.sin(pose.pitch_rad / 2)
            cy, sy = math.cos(pose.yaw_enu_rad / 2), math.sin(pose.yaw_enu_rad / 2)
            q = message.pose.orientation
            q.x, q.y, q.z, q.w = (
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            )
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = f"{self.vehicle_id}/estimated_flu"
            transform.transform.translation.x = message.pose.position.x
            transform.transform.translation.y = message.pose.position.y
            transform.transform.translation.z = message.pose.position.z
            transform.transform.rotation = q
            self._transforms.set_transform(transform, "recorded-px4-estimate")
            self._pose_input.signalMessage(message)
            self._poses[round(timestamp * 1e9)] = pose
            while len(self._poses) > 192:
                self._poses.pop(next(iter(self._poses)))
            self._current_pose = pose
            self._drain(now_seconds)

    def offer(self, observation: ObservationStamp, now_seconds: float) -> None:
        """Queue at most two completed stamps; never fabricate missing camera measurements."""
        with self._lock:
            if (
                not all(
                    math.isfinite(value)
                    for value in (
                        observation.simulation_seconds,
                        observation.available_seconds,
                        now_seconds,
                    )
                )
                or observation.simulation_seconds <= 0
            ):
                self._state = "invalid-or-uninitialized-observation-clock"
                self._latest_observation = None
                self._dropped += 1
                return
            if (
                observation.vehicle_id != self.vehicle_id
                or observation.controller_run_id != self._controller_run_id
                or observation.calibration_id != self.calibration_id
            ):
                self._state = "observation-identity-or-calibration-mismatch"
                self._latest_observation = None
                self._dropped += 1
                return
            if self._source_session_id not in {None, observation.source_session_id}:
                self._state = "source-restarted-reset-required"
                self._latest_observation = None
                self._pending.clear()
                self._dropped += 1
                return
            self._source_session_id = observation.source_session_id
            if (
                self._latest_observation is not None
                and observation.simulation_seconds
                <= self._latest_observation.observation.simulation_seconds
            ):
                self._state = "duplicate-or-out-of-order-observation"
                self._dropped += 1
                return
            if len(self._pending) == 2:
                self._dropped += 1
            self._pending.append(observation)
            self._drain(now_seconds)

    def _drain(self, now_seconds: float) -> None:
        if self._current_pose is None:
            return
        remaining = deque(maxlen=2)
        for observation in self._pending:
            if (
                self._latest_observation is not None
                and observation.simulation_seconds
                <= self._latest_observation.observation.simulation_seconds
            ):
                self._dropped += 1
                continue
            age = self._current_pose.simulation_timestamp_seconds - observation.simulation_seconds
            if age > 0.9 or now_seconds - observation.available_seconds > 1.8:
                self._state = "stale-observation"
                self._dropped += 1
                continue
            stamp = ros_time(round(observation.simulation_seconds * 1e9))
            before = self._pose_cache.getElemBeforeTime(stamp)
            after = self._pose_cache.getElemAfterTime(stamp)
            nearby = self._pose_cache.getInterval(
                ros_time(stamp.nanoseconds - 80_000_000),
                ros_time(stamp.nanoseconds + 80_000_000),
            )
            lookup = stamp
            usable = bool(nearby) and age >= -0.08
            if before is not None and after is not None:
                usable &= (
                    Time.from_msg(after.header.stamp).nanoseconds
                    - Time.from_msg(before.header.stamp).nanoseconds
                ) <= 120_000_000
            elif nearby:
                nearest = min(
                    nearby,
                    key=lambda p: abs(
                        Time.from_msg(p.header.stamp).nanoseconds - stamp.nanoseconds
                    ),
                )
                lookup = Time.from_msg(nearest.header.stamp)
                usable &= abs(lookup.nanoseconds - stamp.nanoseconds) <= 40_000_000
            if not usable:
                if now_seconds - observation.available_seconds <= 0.25:
                    remaining.append(observation)
                    self._state = "waiting-for-estimated-transform"
                else:
                    self._dropped += 1
                    self._state = "pose-alignment-unavailable"
                continue
            try:
                transform = self._transforms.lookup_transform(
                    f"{self.vehicle_id}/local_enu", f"{self.vehicle_id}/estimated_flu", lookup
                )
            except TransformException:
                remaining.append(observation)
                self._state = "estimated-transform-unavailable"
                continue
            q, position = transform.transform.rotation, transform.transform.translation
            nearest_pose = min(
                self._poses.values(),
                key=lambda p: abs(p.simulation_timestamp_seconds - observation.simulation_seconds),
            )
            pose = replace(
                nearest_pose,
                position_enu_m=(position.x, position.y, position.z),
                roll_rad=math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
                pitch_rad=math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
                yaw_enu_rad=math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                ),
                simulation_timestamp_seconds=lookup.nanoseconds / 1e9,
            )
            self._latest_observation = SynchronizedObservation(observation, pose, now_seconds)
            self._accepted += 1
            self._state = "aligned"
        self._pending = remaining

    def latest(self, now_seconds: float) -> SynchronizedObservation | None:
        """Read one coherent pair and recheck freshness at the caller's receive clock."""
        with self._lock:
            selected = self._latest_observation
            if selected is None or self._current_pose is None:
                return None
            age = (
                self._current_pose.simulation_timestamp_seconds
                - selected.observation.simulation_seconds
            )
            if (
                not -0.08 <= age <= 0.9
                or now_seconds - selected.observation.available_seconds > 1.8
            ):
                self._state = "stale-observation"
                return None
            return selected

    def snapshot(self) -> dict:
        """Return bounded experiment counters; no transport handles or command authority."""
        with self._lock:
            return {
                "state": self._state,
                "accepted": self._accepted,
                "dropped": self._dropped,
                "pending": len(self._pending),
                "pose_count": len(self._poses),
            }
