"""Stereo measurements distinguish measured surfaces from unknown image regions."""

import unittest
from unittest.mock import patch

import cv2
import numpy as np

from integrations.vision.geometry import StereoGeometry
from integrations.vision.live import optical_points_in_map


class GeometryTests(unittest.TestCase):
    @staticmethod
    def encoded(image: np.ndarray) -> bytes:
        accepted, contents = cv2.imencode(".png", image)
        if not accepted:
            raise RuntimeError("Fixture encoding failed")
        return contents.tobytes()

    def test_known_disparity_recovers_metric_surface(self) -> None:
        random = np.random.default_rng(19)
        left = random.integers(0, 256, (240, 320), dtype=np.uint8)
        right = np.zeros_like(left)
        right[:, :-8] = left[:, 8:]
        for baseline_m, fov_rad, stride, row_stride in [(0.16, 1.0, 8, 8), (0.4, 1.8, 4, 2)]:
            with self.subTest(baseline_m=baseline_m, sample_stride=stride):
                geometry = StereoGeometry(
                    320,
                    240,
                    baseline_m,
                    fov_rad,
                    sample_stride=stride,
                    vertical_sample_stride=row_stride,
                )
                images = (self.encoded(left), self.encoded(right))
                result = geometry.measure(*images)
                with patch(
                    "integrations.vision.geometry.cv2.imencode",
                    side_effect=RuntimeError("Unused preview encoder must not run"),
                ):
                    measured = geometry.measure(*images, include_preview=False)
                self.assertEqual(measured.points, result.points)
                self.assertEqual(measured.valid_fraction, result.valid_fraction)
                self.assertEqual(measured.preview_png, b"")
                self.assertTrue(result.preview_png)
                self.assertGreater(result.valid_fraction, 0.4)
                self.assertGreater(len(result.points), 200)
                self.assertAlmostEqual(
                    float(np.median(np.array(result.points)[:, 2])),
                    geometry.focal_px * baseline_m / 8,
                    delta=0.1,
                )

    def test_blank_images_do_not_fabricate_surface_measurements(self) -> None:
        image = self.encoded(np.zeros((240, 320), np.uint8))
        result = StereoGeometry(320, 240, 0.16, 1).measure(image, image)
        self.assertEqual(result.valid_fraction, 0)
        self.assertEqual(result.points, [])

    def test_optical_transform_uses_estimated_orientation_and_translation(self) -> None:
        transformed = optical_points_in_map([[1, 0, 0]], [2, 3, 4], [0, 0, 0, 1])
        self.assertEqual(transformed, [[3, 3, 4]])
        with self.assertRaises(ValueError):
            optical_points_in_map([[1, 0, 0]], [0, 0, 0], [0, 0, 0, 0])
