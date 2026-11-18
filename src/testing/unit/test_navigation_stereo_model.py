"""Declared camera geometry must match the optical processing contract."""

import math
import unittest
import xml.etree.ElementTree as ET

from common.configuration.navigation_stereo import NavigationStereoCalibration
from gazebo.navigation_stereo_model import add_navigation_stereo


class NavigationStereoModelTests(unittest.TestCase):
    def test_pairs_have_parallel_axes_and_declared_baselines(self) -> None:
        calibration = NavigationStereoCalibration()
        model = ET.Element("model")
        add_navigation_stereo(model)
        self.assertEqual(len(model.findall("link/sensor")), 12)
        for rig in range(6):
            left = model.find(f"link[@name='navigation_stereo_{rig}_left']")
            right = model.find(f"link[@name='navigation_stereo_{rig}_right']")
            a, b = [[float(v) for v in link.findtext("pose").split()] for link in [left, right]]
            self.assertAlmostEqual(math.dist(a[:3], b[:3]), calibration.baseline_m)
            self.assertEqual(a[3:], b[3:])
            self.assertEqual(left.findtext("sensor/camera/image/width"), "320")
            self.assertEqual(left.findtext("sensor/camera/horizontal_fov"), "1.8")
            self.assertIsNone(left.find("collision"))
