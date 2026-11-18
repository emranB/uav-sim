"""Image detections are validated against synthetic pixel-space ground truth."""

import unittest

import cv2
import numpy as np

from integrations.vision.regions import detect_regions


class RegionTests(unittest.TestCase):
    def test_detects_rectangle_bounds_and_centroid_without_semantic_claims(self) -> None:
        image = np.zeros((200, 300, 3), dtype=np.uint8)
        cv2.rectangle(image, (60, 40), (180, 140), (255, 255, 255), -1)
        regions = detect_regions(image)
        self.assertEqual(len(regions), 1)
        self.assertAlmostEqual(regions[0].centroid_xy_px[0], 120, delta=2)
        self.assertAlmostEqual(regions[0].centroid_xy_px[1], 90, delta=2)
        self.assertLessEqual(abs(regions[0].bounds_xywh_px[2] - 120), 3)

    def test_blank_frame_has_no_detections_and_invalid_input_is_rejected(self) -> None:
        self.assertEqual(detect_regions(np.zeros((100, 100, 3), np.uint8)), ())
        with self.assertRaises(ValueError):
            detect_regions(np.zeros((100, 100), np.uint8))
