"""Mapping inputs keep calibration explicit and evaluation truth out of estimator inputs."""

import unittest
from pathlib import Path

from common.logging.stereo import StereoFrame, StereoRecording
from integrations.vision.mapping import camera_configuration


class MappingConfigurationTests(unittest.TestCase):
    def test_parallel_stereo_calibration_is_explicit(self) -> None:
        recording = StereoRecording(
            Path("."),
            (StereoFrame(0, "left.ppm", "right.ppm", 640, 480),),
            {"baseline_m": 0.16, "horizontal_fov_rad": 1.0, "distortion": "none"},
        )
        configuration = camera_configuration(recording)
        self.assertIn('Camera.type: "PinHole"', configuration)
        self.assertIn("Stereo.T_c1_c2: !!opencv-matrix", configuration)
        self.assertIn("Stereo.b: 0.16", configuration)
        self.assertIn("Camera.fps: 5\n", configuration)
        self.assertNotIn("evaluation", configuration)
        self.assertIn("Camera.fps: 10\n", camera_configuration(recording, frame_rate_hz=10))

    def test_unknown_calibration_is_not_silently_defaulted(self) -> None:
        for baseline in (-1, None, float("nan"), True):
            recording = StereoRecording(
                Path("."),
                (StereoFrame(0, "left.ppm", "right.ppm", 640, 480),),
                {"baseline_m": baseline, "horizontal_fov_rad": 1.0, "distortion": "none"},
            )
            with self.subTest(baseline=baseline), self.assertRaises(ValueError):
                camera_configuration(recording)

    def test_pyramid_geometry_is_rejected_before_native_abort(self) -> None:
        for width, height, safe_levels in ((640, 480, 15), (320, 240, 11)):
            recording = StereoRecording(
                Path("."),
                (StereoFrame(0, "left.ppm", "right.ppm", width, height),),
                {"baseline_m": 0.4, "horizontal_fov_rad": 1.4, "distortion": "none"},
            )
            camera_configuration(recording, pyramid_levels=safe_levels)
            with self.assertRaisesRegex(ValueError, "32 pixels"):
                camera_configuration(recording, pyramid_levels=safe_levels + 1)
