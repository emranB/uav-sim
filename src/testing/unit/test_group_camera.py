"""Shared viewer framing must react to separation without flipping its orientation."""

import math
import unittest

from gazebo.group_camera import group_camera_pose


class GroupCameraTests(unittest.TestCase):
    def test_separation_expands_view_about_midpoint(self):
        focus, close = group_camera_pose([(0, 0, 4), (6, 0, 4)])
        self.assertEqual(focus, (3, 0, 4))
        focus, wide = group_camera_pose([(0, 0, 4), (6, 20, 4)])
        self.assertEqual(focus, (3, 10, 4))
        self.assertGreater(math.dist(wide, (0, 0, 0)), math.dist(close, (0, 0, 0)))
        self.assertAlmostEqual(wide[0] / wide[1], close[0] / close[1])
        self.assertAlmostEqual(wide[2] / wide[1], close[2] / close[1])

    def test_invalid_poses_rejected(self):
        for positions in [[], [(0, 0, 0)], [(0, 0, 0), (float("nan"), 0, 0)]]:
            with self.assertRaises(ValueError):
                group_camera_pose(positions)
