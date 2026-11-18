"""Acceptance measurements for a live observer during a waypoint flight."""

from dataclasses import dataclass, field


@dataclass
class TrackingContinuity:
    """Count every processed frame through counter deltas, excluding frozen geometry."""

    processed_frames: int = 0
    tracked_frames: int = 0
    stale_samples: int = 0
    coordinate_frames: set[tuple[int, int]] = field(default_factory=set)
    missing_frame_identity: bool = False
    map_ids: set[int] = field(default_factory=set)
    session_ids: set[str] = field(default_factory=set)
    previous: tuple[int, int] | None = None
    initial_loss_events: int | None = None
    loss_events: int = 0
    initial_gap_events: int | None = None
    gap_events: int = 0
    maximum_pose_speed_m_s: float = 0.0

    def observe(self, snapshot: dict) -> None:
        """Sample only while above the declared airborne evaluation altitude."""
        self.stale_samples += int(snapshot.get("stale", True))
        self.maximum_pose_speed_m_s = max(
            self.maximum_pose_speed_m_s, snapshot.get("maximum_pose_speed_m_s", 0.0)
        )
        if snapshot.get("session_id"):
            self.session_ids.add(snapshot["session_id"])
        current = (snapshot.get("processed_frames", 0), snapshot.get("tracked_frames", 0))
        total_losses = snapshot.get(
            "tracking_loss_events", (self.initial_loss_events or 0) + self.loss_events
        )
        if self.initial_loss_events is None:
            self.initial_loss_events = total_losses
        self.loss_events = total_losses - self.initial_loss_events
        total_gaps = snapshot.get(
            "input_gap_events", (self.initial_gap_events or 0) + self.gap_events
        )
        if self.initial_gap_events is None:
            self.initial_gap_events = total_gaps
        self.gap_events = total_gaps - self.initial_gap_events
        if self.previous is not None:
            frames = current[0] - self.previous[0]
            tracked = current[1] - self.previous[1]
            if frames < 0 or tracked < 0 or tracked > frames:
                raise ValueError("Estimator counters changed generation during flight")
            self.processed_frames += frames
            self.tracked_frames += tracked
        self.previous = current
        if snapshot.get("position_valid") and snapshot.get("map_id", -1) >= 0:
            self.map_ids.add(snapshot["map_id"])
            generation = snapshot.get("map_generation")
            if type(generation) is int and generation >= 0:
                self.coordinate_frames.add((snapshot["map_id"], generation))
            else:
                self.missing_frame_identity = True

    def result(self) -> dict:
        """Require 98% tracking, one map/session and no stale airborne samples."""
        fraction = self.tracked_frames / max(1, self.processed_frames)
        return {
            "passed": self.processed_frames >= 20
            and fraction >= 0.98
            and len(self.map_ids) == 1
            and len(self.coordinate_frames) == 1
            and not self.missing_frame_identity
            and len(self.session_ids) == 1
            and self.stale_samples == 0
            and self.loss_events == 0
            and self.gap_events == 0
            and self.maximum_pose_speed_m_s <= 10,
            "tracking_loss_events": self.loss_events,
            "input_gap_events": self.gap_events,
            "maximum_pose_speed_m_s": self.maximum_pose_speed_m_s,
            "pose_speed_scope": "whole session; consecutive valid poses in one map",
            "processed_frames": self.processed_frames,
            "tracked_frames": self.tracked_frames,
            "tracked_fraction": fraction,
            "stale_samples": self.stale_samples,
            "map_ids": sorted(self.map_ids),
            "coordinate_frames": sorted(self.coordinate_frames),
            "frame_identity_complete": not self.missing_frame_identity,
            "session_ids": sorted(self.session_ids),
            "scope": "Above 3 m relative altitude; frame counters include unsampled frames",
        }
