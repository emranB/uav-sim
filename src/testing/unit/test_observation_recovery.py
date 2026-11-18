"""Freshness recovery never refreshes old evidence or resets finite budgets."""

import unittest

from common.interfaces.obstacles import StaleObservationError
from common.navigation.freshness import ObservationRecovery, ObservationWaitPolicy


class ObservationRecoveryTests(unittest.TestCase):
    def stale(self, timestamp: float = 10.0) -> StaleObservationError:
        return StaleObservationError(
            "vehicle_1",
            "Stereo sensor timestamp stale",
            timestamp_seconds=timestamp,
            sensor_age_sim_seconds=0.904,
            delivery_age_wall_seconds=0.975,
        )

    def test_only_newer_replanned_observation_completes_wait(self) -> None:
        recovery = ObservationRecovery("vehicle_1", ObservationWaitPolicy())
        recovery.reject(self.stale(), 1.0)
        self.assertTrue(recovery.waiting)
        self.assertFalse(recovery.route_ready(10.0, 1.1))
        self.assertTrue(recovery.route_ready(10.2, 1.2))
        snapshot = recovery.snapshot(1.2)
        self.assertEqual(snapshot["freshness_wait_count"], 1)
        self.assertEqual(snapshot["freshness_recovery_count"], 1)
        self.assertAlmostEqual(snapshot["freshness_wait_total_wall_seconds"], 0.2)

    def test_rejections_cannot_extend_episode_deadline(self) -> None:
        recovery = ObservationRecovery("vehicle_1", ObservationWaitPolicy())
        recovery.reject(self.stale(), 1.0)
        recovery.reject(self.stale(10.2), 1.4)
        with self.assertRaisesRegex(RuntimeError, "deadline"):
            recovery.check(1.501)
        self.assertFalse(recovery.waiting)
        self.assertEqual(recovery.snapshot(1.501)["freshness_abort_count"], 1)

    def test_repeated_waits_have_cumulative_time_and_episode_caps(self) -> None:
        recovery = ObservationRecovery("vehicle_1", ObservationWaitPolicy())
        for timestamp, start in [(10, 1.0), (11, 2.0)]:
            recovery.reject(self.stale(timestamp), start)
            self.assertTrue(recovery.route_ready(timestamp + 0.2, start + 0.4))
        recovery.reject(self.stale(12), 3.0)
        with self.assertRaisesRegex(RuntimeError, "cumulative"):
            recovery.check(3.21)
        capped = ObservationRecovery("vehicle_1", ObservationWaitPolicy())
        for index in range(3):
            capped.reject(self.stale(10 + index), index + 1.0)
            capped.route_ready(10.2 + index, index + 1.01)
        with self.assertRaisesRegex(RuntimeError, "episode"):
            capped.reject(self.stale(14), 5.0)

    def test_policy_rejects_unbounded_or_nonfinite_limits(self) -> None:
        for values in [
            {"maximum_wait_seconds": 2},
            {"maximum_total_wait_seconds": 10},
            {"maximum_wait_episodes": 100},
            {"maximum_wait_seconds": float("nan")},
        ]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                ObservationWaitPolicy(**values)

    def test_abort_closes_wait_without_recovery_and_preserves_rejected_age(self) -> None:
        recovery = ObservationRecovery("vehicle_1", ObservationWaitPolicy())
        recovery.reject(self.stale(), 1.0)
        recovery.record_zero_command(1.1, (0, 0, 4), 10.904)
        recovery.abort(1.2, "cancelled")
        snapshot = recovery.snapshot(1.3)
        self.assertEqual(snapshot["freshness_recovery_count"], 0)
        self.assertEqual(snapshot["freshness_abort_count"], 1)
        self.assertTrue(
            any(e.get("sensor_age_sim_seconds") == 0.904 for e in snapshot["freshness_wait_events"])
        )
        self.assertTrue(
            any(e["kind"] == "zero_velocity_command" for e in snapshot["freshness_wait_events"])
        )

    def test_late_replan_cannot_revive_an_expired_episode(self) -> None:
        recovery = ObservationRecovery("vehicle_1", ObservationWaitPolicy())
        recovery.reject(self.stale(), 1.0)
        with self.assertRaisesRegex(RuntimeError, "deadline"):
            recovery.check(1.6)
        with self.assertRaisesRegex(RuntimeError, "deadline"):
            recovery.route_ready(10.2, 1.7)
        self.assertEqual(recovery.snapshot(1.7)["freshness_recovery_count"], 0)
