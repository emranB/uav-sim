"""Estimated-pose interpolation, frame angles and epoch boundaries."""

import math
import unittest

from common.interfaces.navigation import NavigationPose
from common.navigation.pose_history import EstimatedPoseHistory


class PoseHistoryTests(unittest.TestCase):
    def test_interpolation_handles_yaw_wrap(self) -> None:
        history = EstimatedPoseHistory()
        history.append(NavigationPose((0, 0, 4), math.pi - 0.02, 1, "a", True, "OFFBOARD", 1))
        history.append(NavigationPose((1, 0, 4), -math.pi + 0.02, 1, "a", True, "OFFBOARD", 1.1))
        pose = history.at(1.05)
        self.assertAlmostEqual(pose.position_enu_m[0], 0.5)
        self.assertAlmostEqual(pose.yaw_enu_rad, math.pi)

    def test_gap_and_generation_are_not_interpolated(self) -> None:
        history = EstimatedPoseHistory()
        history.append(NavigationPose((0, 0, 4), 0, 1, "a", True, "OFFBOARD", 1))
        with self.assertRaises(RuntimeError):
            history.at(1.2)
        history.append(NavigationPose((0, 0, 4), 0, 1, "b", True, "OFFBOARD", 2))
        with self.assertRaises(RuntimeError):
            history.at(1)

    def test_paused_duplicate_time_does_not_expand_storage(self) -> None:
        history = EstimatedPoseHistory()
        for i in range(300):
            history.append(NavigationPose((0, 0, 4), 0, 1, "a", True, "OFFBOARD", 1))
        self.assertEqual(history.at(1).position_enu_m, (0, 0, 4))
        self.assertLessEqual(len(history.samples), 192)
