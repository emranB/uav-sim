"""Jazzy adapter checks need actual ROS libraries but no simulator or DDS publishers."""

from __future__ import annotations

import importlib
import unittest

from common.interfaces.navigation import NavigationPose

try:
    rclpy = importlib.import_module("rclpy")
    importlib.import_module("message_filters")
    importlib.import_module("tf2_ros")
except ModuleNotFoundError as error:
    rclpy = None
    ROS_UNAVAILABLE = str(error)
else:
    ROS_UNAVAILABLE = ""
    from integrations.ros2.observation_sync import (
        EstimatedObservationSynchronizer,
        ObservationStamp,
    )


def pose(timestamp: float, run: str = "run") -> NavigationPose:
    return NavigationPose((timestamp, 0, 4), 0, timestamp, run, False, "recorded", timestamp)


def stamp(timestamp: float, available: float, source: str = "source") -> ObservationStamp:
    return ObservationStamp("vehicle_1", "run", source, "calibration", timestamp, available)


@unittest.skipIf(rclpy is None, "Jazzy environment unavailable: " + ROS_UNAVAILABLE)
class RosObservationSyncTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init(domain_id=84)

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.sync = EstimatedObservationSynchronizer("vehicle_1", "run", "calibration")

    def test_unequal_rate_estimates_interpolate_without_one_to_one_pairing(self) -> None:
        for timestamp in [10.0, 10.04, 10.08, 10.12]:
            self.sync.add_pose(pose(timestamp), timestamp)
        self.sync.offer(stamp(10.06, 10.12), 10.12)
        selected = self.sync.latest(10.12)
        self.assertIsNotNone(selected)
        self.assertAlmostEqual(selected.pose.position_enu_m[0], 10.06)
        self.assertAlmostEqual(selected.pose.simulation_timestamp_seconds, 10.06)

    def test_future_pose_wait_and_true_gap_keep_original_bounds(self) -> None:
        self.sync.add_pose(pose(10), 0)
        self.sync.offer(stamp(10.06, 0.01), 0.01)
        self.assertIsNone(self.sync.latest(0.01))
        self.sync.add_pose(pose(10.08), 0.04)
        self.assertIsNotNone(self.sync.latest(0.04))
        self.sync.reset("run")
        self.sync.add_pose(pose(10), 0)
        self.sync.add_pose(pose(10.2), 0.01)
        self.sync.offer(stamp(10.07, 0.02), 0.02)
        self.assertIsNone(self.sync.latest(0.02))
        self.sync.add_pose(pose(10.24), 0.4)
        self.assertEqual(self.sync.snapshot()["state"], "pose-alignment-unavailable")

    def test_newest_completed_observation_does_not_regress(self) -> None:
        self.sync.add_pose(pose(10.1), 0)
        self.sync.add_pose(pose(10.18), 0.01)
        self.sync.offer(stamp(10.16, 0.02), 0.02)
        self.sync.offer(stamp(10.12, 0.03), 0.03)
        self.assertAlmostEqual(self.sync.latest(0.03).observation.simulation_seconds, 10.16)
        self.assertEqual(self.sync.snapshot()["dropped"], 1)

    def test_stale_calibration_epoch_reset_and_bounded_history_are_explicit(self) -> None:
        self.sync.add_pose(pose(10.9), 0)
        self.sync.offer(stamp(9.98, 0.01), 0.01)
        self.assertEqual(self.sync.snapshot()["state"], "stale-observation")
        self.sync.reset("run")
        self.sync.add_pose(pose(10), 0)
        self.sync.offer(stamp(10, 0.01), 0.01)
        self.sync.offer(stamp(10, 0.02, "new-source"), 0.02)
        self.assertIsNone(self.sync.latest(0.02))
        self.assertEqual(self.sync.snapshot()["state"], "source-restarted-reset-required")
        self.sync.reset("new-run")
        self.assertEqual(self.sync.snapshot()["pose_count"], 0)
        for index in range(300):
            self.sync.add_pose(pose(10 + index * 0.01, "new-run"), index * 0.01)
        self.assertLessEqual(self.sync.snapshot()["pose_count"], 192)
        self.sync.add_pose(pose(9, "new-run"), 4.0)
        self.assertEqual(self.sync.snapshot()["pose_count"], 1)
        self.assertIsNone(self.sync.latest(4.0))
        self.sync.reset("run")
        self.sync.add_pose(pose(10), 0)
        self.sync.offer(ObservationStamp("vehicle_1", "run", "source", "wrong", 10, 0.01), 0.01)
        self.assertEqual(
            self.sync.snapshot()["state"], "observation-identity-or-calibration-mismatch"
        )
