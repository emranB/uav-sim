"""Finite observation-recovery budgets, independent of transports and vehicle commands."""

import math
import threading
from collections import deque
from dataclasses import dataclass

from common.interfaces.obstacles import StaleObservationError


@dataclass(frozen=True)
class ObservationWaitPolicy:
    """Wall-clock limits: at most 2.5 nominal 5Hz source periods per episode."""

    maximum_wait_seconds: float = 0.5
    maximum_total_wait_seconds: float = 1.0
    maximum_wait_episodes: int = 3

    def __post_init__(self) -> None:
        for value, maximum in [
            (self.maximum_wait_seconds, 0.5),
            (self.maximum_total_wait_seconds, 1.0),
        ]:
            if (
                type(value) not in {int, float}
                or not math.isfinite(value)
                or not 0 < value <= maximum
            ):
                raise ValueError("Observation recovery requires finite short wall-clock budgets")
        if self.maximum_wait_seconds > self.maximum_total_wait_seconds:
            raise ValueError("Episode wait must fit the cumulative wall-clock budget")
        if type(self.maximum_wait_episodes) is not int or not 1 <= self.maximum_wait_episodes <= 3:
            raise ValueError("Observation recovery permits one to three episodes per run")


class ObservationRecovery:
    """Own bounded wait history; a fresh route, not a health probe, ends a wait."""

    def __init__(self, vehicle_id: str, policy: ObservationWaitPolicy) -> None:
        self.vehicle_id = vehicle_id
        self.policy = policy
        self._lock = threading.RLock()
        self._started: float | None = None
        self._required_timestamp = -math.inf
        self._count = 0
        self._recovered = 0
        self._aborted = 0
        self._elapsed = 0.0
        self._failure: str | None = None
        self._events: deque[dict] = deque(maxlen=128)
        self._dropped_events = 0

    @property
    def waiting(self) -> bool:
        with self._lock:
            return self._started is not None

    def _event(self, kind: str, now: float, **values: object) -> None:
        if len(self._events) == self._events.maxlen:
            self._dropped_events += 1
        self._events.append(
            {"kind": kind, "vehicle_id": self.vehicle_id, "monotonic": now, **values}
        )

    def reject(
        self,
        failure: StaleObservationError,
        now: float,
        *,
        origin: str | None = None,
        pose_simulation_seconds: float | None = None,
    ) -> None:
        """Reject evidence and start one episode without extending an existing deadline."""
        with self._lock:
            if failure.vehicle_id != self.vehicle_id:
                raise RuntimeError("Observation recovery vehicle identity mismatch")
            self.check(now)
            observation = {
                "sensor_timestamp_seconds": failure.timestamp_seconds,
                "sensor_age_sim_seconds": failure.sensor_age_sim_seconds,
                "delivery_age_wall_seconds": failure.delivery_age_wall_seconds,
                "source_session_id": failure.source_session_id,
                "origin": origin,
                "pose_simulation_seconds": pose_simulation_seconds,
            }
            if self._started is None:
                if self._count >= self.policy.maximum_wait_episodes:
                    self._aborted += 1
                    self._failure = "episode limit"
                    self._event(
                        "wait_aborted",
                        now,
                        reason="episode limit",
                        observation_reason=str(failure),
                        **observation,
                    )
                    raise RuntimeError(
                        f"{self.vehicle_id}: observation wait episode limit exceeded"
                    )
                if self._elapsed >= self.policy.maximum_total_wait_seconds:
                    self._failure = "cumulative observation wait exhausted"
                    self._aborted += 1
                    self._event("wait_aborted", now, reason=self._failure)
                    raise RuntimeError(f"{self.vehicle_id}: cumulative observation wait exhausted")
                self._started = now
                self._count += 1
                self._required_timestamp = failure.timestamp_seconds
                self._event("wait_started", now, episode=self._count)
            self._required_timestamp = max(self._required_timestamp, failure.timestamp_seconds)
            self._event(
                "observation_rejected",
                now,
                reason=str(failure),
                **observation,
            )

    def check(self, now: float) -> None:
        """Abort a spent deadline; budgets are never renewed by another stale sample."""
        with self._lock:
            if self._failure is not None:
                raise RuntimeError(f"{self.vehicle_id}: observation wait {self._failure}")
            if self._started is None:
                return
            elapsed = now - self._started
            reason = None
            if elapsed >= self.policy.maximum_wait_seconds:
                reason = "episode deadline"
            elif self._elapsed + elapsed >= self.policy.maximum_total_wait_seconds:
                reason = "cumulative wall-clock budget"
            if reason:
                self.abort(now, reason)
                raise RuntimeError(f"{self.vehicle_id}: observation wait {reason} exceeded")

    def route_ready(self, timestamp_seconds: float, now: float) -> bool:
        """End a wait only after the caller validated and replanned newer evidence."""
        with self._lock:
            self.check(now)
            if not math.isfinite(timestamp_seconds):
                raise RuntimeError("Invalid fresh-route observation timestamp")
            if self._started is None:
                return True
            if timestamp_seconds <= self._required_timestamp:
                return False
            self._elapsed += now - self._started
            self._started = None
            self._recovered += 1
            self._event("fresh_route_recovered", now, sensor_timestamp_seconds=timestamp_seconds)
            return True

    def record_zero_command(
        self, now: float, position_enu_m: tuple, pose_simulation_seconds: float | None
    ) -> None:
        with self._lock:
            if self._started is not None:
                self._event(
                    "zero_velocity_command",
                    now,
                    position_enu_m=position_enu_m,
                    pose_simulation_seconds=pose_simulation_seconds,
                )

    def abort(self, now: float, reason: str) -> None:
        with self._lock:
            if self._started is not None:
                self._elapsed += now - self._started
                self._started = None
                self._aborted += 1
                self._failure = reason
                self._event("wait_aborted", now, reason=reason)

    def snapshot(self, now: float) -> dict:
        """Return versioned, bounded wait evidence, with wall and simulation units explicit."""
        with self._lock:
            active_seconds = now - self._started if self._started is not None else 0.0
            return {
                "freshness_policy_version": 1,
                "freshness_waiting": self._started is not None,
                "freshness_wait_count": self._count,
                "freshness_recovery_count": self._recovered,
                "freshness_abort_count": self._aborted,
                "freshness_wait_total_wall_seconds": self._elapsed + active_seconds,
                "freshness_current_wait_wall_seconds": active_seconds,
                "freshness_wait_events": list(self._events),
                "freshness_wait_events_dropped": self._dropped_events,
            }
