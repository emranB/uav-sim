"""Bounded time alignment of estimated poses, not simulator truth."""

import math
from collections import deque
from dataclasses import replace

from common.interfaces.navigation import NavigationPose


class EstimatedPoseHistory:
    """Caller serializes append/read; keep at most 192 estimated pose samples."""

    def __init__(self) -> None:
        self.samples: deque[NavigationPose] = deque(maxlen=192)

    def clear(self) -> None:
        self.samples.clear()

    def append(self, pose: NavigationPose) -> None:
        timestamp = pose.simulation_timestamp_seconds
        if timestamp is None:
            return
        if self.samples and (
            pose.run_id != self.samples[-1].run_id
            or timestamp < self.samples[-1].simulation_timestamp_seconds
        ):
            self.clear()
        self.samples.append(pose)

    def needs_future_sample(self, timestamp_seconds: float) -> bool:
        """Whether a newer estimate could bracket this measurement time."""
        return (
            bool(self.samples) and timestamp_seconds > self.samples[-1].simulation_timestamp_seconds
        )

    def at(self, timestamp_seconds: float) -> NavigationPose:
        if not math.isfinite(timestamp_seconds) or not self.samples:
            raise RuntimeError("Estimated pose history is unavailable")
        nearest = min(
            self.samples,
            key=lambda pose: abs(pose.simulation_timestamp_seconds - timestamp_seconds),
        )
        if abs(nearest.simulation_timestamp_seconds - timestamp_seconds) > 0.08:
            raise RuntimeError(
                f"Measurement/estimated-pose alignment exceeds 0.08 simulation seconds: "
                f"requested={timestamp_seconds:.3f}, nearest={nearest.simulation_timestamp_seconds:.3f}"
            )
        before = next(
            (
                pose
                for pose in reversed(self.samples)
                if pose.simulation_timestamp_seconds <= timestamp_seconds
            ),
            None,
        )
        after = next(
            (
                pose
                for pose in self.samples
                if pose.simulation_timestamp_seconds >= timestamp_seconds
            ),
            None,
        )
        if before is None or after is None:
            if abs(nearest.simulation_timestamp_seconds - timestamp_seconds) > 0.04:
                raise RuntimeError(
                    f"Measurement lies outside estimated pose history: requested={timestamp_seconds:.3f}, "
                    f"oldest={self.samples[0].simulation_timestamp_seconds:.3f}, "
                    f"newest={self.samples[-1].simulation_timestamp_seconds:.3f}"
                )
            return nearest
        gap = after.simulation_timestamp_seconds - before.simulation_timestamp_seconds
        if gap > 0.12 or before.run_id != after.run_id:
            raise RuntimeError("Estimated pose history has a gap or generation change")
        fraction = (timestamp_seconds - before.simulation_timestamp_seconds) / gap if gap else 0
        yaw_delta = math.atan2(
            math.sin(after.yaw_enu_rad - before.yaw_enu_rad),
            math.cos(after.yaw_enu_rad - before.yaw_enu_rad),
        )
        return replace(
            before,
            position_enu_m=tuple(
                before.position_enu_m[i]
                + fraction * (after.position_enu_m[i] - before.position_enu_m[i])
                for i in range(3)
            ),
            yaw_enu_rad=before.yaw_enu_rad + fraction * yaw_delta,
            roll_rad=before.roll_rad + fraction * (after.roll_rad - before.roll_rad),
            pitch_rad=before.pitch_rad + fraction * (after.pitch_rad - before.pitch_rad),
            simulation_timestamp_seconds=timestamp_seconds,
        )
