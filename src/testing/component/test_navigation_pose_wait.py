"""Bounded future-pose alignment releases the receiver without accepting stale history."""

import threading
import time
import unittest
from unittest.mock import patch

from common.interfaces.navigation import NavigationPose
from common.navigation.pose_history import EstimatedPoseHistory
from integrations.mavlink.connection import SimulatedVehicleConnection


def recorded_pose(timestamp_seconds: float) -> NavigationPose:
    """Explicit estimated-pose fixture; no simulator or transport connection."""
    return NavigationPose(
        (0, 0, 4), 0, time.monotonic(), "fixture", True, "OFFBOARD", timestamp_seconds
    )


class NavigationPoseWaitTests(unittest.TestCase):
    def setUp(self) -> None:
        # Exercise the adapter's alignment operation without opening a competing socket.
        self.connection = object.__new__(SimulatedVehicleConnection)
        self.connection.condition = threading.Condition()
        self.connection.navigation_history = EstimatedPoseHistory()
        self.connection.navigation_history.append(recorded_pose(10.0))

    def test_future_bracket_arrives_and_is_used_at_its_actual_time(self) -> None:
        def arrive() -> None:
            with self.connection.condition:
                self.connection.navigation_history.append(recorded_pose(10.06))
                self.connection.condition.notify_all()

        arrival = threading.Timer(0.02, arrive)
        arrival.start()
        try:
            pose = self.connection.navigation_pose_at(10.06)
            self.assertEqual(pose.simulation_timestamp_seconds, 10.06)
        finally:
            arrival.join(timeout=1)
        self.assertFalse(arrival.is_alive())

    def test_future_bracket_timeout_does_not_extend_extrapolation(self) -> None:
        started = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, "outside estimated pose history"):
            self.connection.navigation_pose_at(10.06)
        elapsed = time.monotonic() - started
        self.assertGreaterEqual(elapsed, 0.2)
        self.assertLess(elapsed, 1.0)

    def test_old_measurement_and_real_history_gap_do_not_wait(self) -> None:
        with patch.object(
            self.connection.condition,
            "wait",
            side_effect=AssertionError("Cannot repair old history by waiting"),
        ):
            with self.assertRaisesRegex(RuntimeError, "outside estimated pose history"):
                self.connection.navigation_pose_at(9.94)
            self.connection.navigation_history.append(recorded_pose(10.2))
            with self.assertRaisesRegex(RuntimeError, "gap"):
                self.connection.navigation_pose_at(10.06)
