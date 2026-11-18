"""Explicit GPU-stage acceptance; unavailable GPU dependencies are reported as skips."""

import os

import cv2
import numpy as np
import pytest

from integrations.vision.geometry import StereoGeometry

pytestmark = pytest.mark.skipif(
    os.environ.get("UAV_TEST_CUDA_SURFACES") != "true",
    reason="Select the VPI CUDA test deployment explicitly",
)


@pytest.mark.parametrize("width,height", [(320, 240), (640, 480)])
def test_cuda_metric_depth_and_unknown_pixels(width: int, height: int) -> None:
    random = np.random.default_rng(19)
    left = random.integers(0, 256, (height, width), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-8] = left[:, 8:]
    geometry = StereoGeometry(width, height, 0.16, 1, "vpi-cuda")
    encoded = [cv2.imencode(".png", image)[1].tobytes() for image in (left, right)]
    result = geometry.measure(*encoded)
    assert result.valid_fraction > 0.4
    assert len(result.points) > 200
    assert np.median(np.asarray(result.points)[:, 2]) == pytest.approx(
        geometry.focal_px * 0.16 / 8, abs=0.1
    )
    blank = cv2.imencode(".png", np.zeros_like(left))[1].tobytes()
    unknown = geometry.measure(blank, blank)
    assert unknown.valid_fraction == 0
    assert unknown.points == []
