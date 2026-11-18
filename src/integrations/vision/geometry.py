"""Read-only stereo surface measurements in the left camera optical frame."""

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class SurfaceMeasurements:
    points: list[list[float]]
    valid_fraction: float
    preview_png: bytes


class StereoGeometry:
    """Match rectified stereo; absent or inconsistent disparity remains unknown."""

    def __init__(
        self,
        width: int,
        height: int,
        baseline_m: float,
        fov_rad: float,
        backend: str = "cpu",
        sample_stride: int = 8,
        vertical_sample_stride: int | None = None,
    ) -> None:
        if (width, height) not in {(320, 240), (640, 480)}:
            raise ValueError("Unsupported calibrated stereo dimensions")
        if not 0 < baseline_m < 1 or not 0 < fov_rad < math.pi:
            raise ValueError("Invalid stereo baseline or field of view")
        if backend not in {"cpu", "vpi-cuda"}:
            raise ValueError("Surface backend must be cpu or vpi-cuda")
        if sample_stride not in {4, 8}:
            raise ValueError("Surface sampling stride must be 4 or 8 pixels")
        vertical_sample_stride = (
            sample_stride if vertical_sample_stride is None else vertical_sample_stride
        )
        if vertical_sample_stride not in {2, 4, 8}:
            raise ValueError("Vertical surface sampling stride must be 2, 4 or 8 pixels")
        self.sample_stride = sample_stride
        self.vertical_sample_stride = vertical_sample_stride
        self.backend = backend
        self.cuda_matcher = None
        if backend == "vpi-cuda":
            from integrations.vision.vpi_stereo import VpiStereoMatcher

            self.cuda_matcher = VpiStereoMatcher()
        self.width, self.height = width, height
        self.focal_px = width / (2 * math.tan(fov_rad / 2))
        self.baseline_m = baseline_m
        options = dict(
            numDisparities=64,
            blockSize=5,
            P1=200,
            P2=800,
            uniquenessRatio=12,
            speckleWindowSize=50,
            speckleRange=2,
            disp12MaxDiff=1,
        )
        self.left_matcher = cv2.StereoSGBM_create(minDisparity=0, **options)
        self.right_matcher = cv2.StereoSGBM_create(minDisparity=-64, **options)

    def measure(
        self, left: bytes, right: bytes, *, include_preview: bool = True
    ) -> SurfaceMeasurements:
        """Return measured surfaces, with empty preview bytes when encoding is not requested."""
        if type(include_preview) is not bool:
            raise ValueError("Surface preview selection must be a boolean")
        images = [
            cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_GRAYSCALE)
            for encoded in (left, right)
        ]
        if any(image is None or image.shape != (self.height, self.width) for image in images):
            raise ValueError("Stereo image geometry differs from calibration")
        if self.cuda_matcher is not None:
            disparity, reverse = self.cuda_matcher.compute(images)
        else:
            disparity = self.left_matcher.compute(*images).astype(np.float32) / 16
            reverse = self.right_matcher.compute(images[1], images[0]).astype(np.float32) / 16
        rows, columns = np.indices(disparity.shape)
        matching_column = np.rint(columns - disparity).astype(np.int32)
        inside = (matching_column >= 0) & (matching_column < self.width)
        reverse_sample = reverse[rows, np.clip(matching_column, 0, self.width - 1)]
        valid = inside & (disparity > 0) & (np.abs(disparity + reverse_sample) <= 1)
        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        depth[valid] = self.focal_px * self.baseline_m / disparity[valid]
        valid &= (depth >= 0.2) & (depth <= 30)
        # Sample display geometry rather than exporting a full image-sized point cloud.
        sampled = (
            valid & (rows % self.vertical_sample_stride == 0) & (columns % self.sample_stride == 0)
        )
        z = depth[sampled]
        x = (columns[sampled] - self.width / 2) * z / self.focal_px
        y = (rows[sampled] - self.height / 2) * z / self.focal_px
        points = np.column_stack((x, y, z)).round(4).tolist()
        if not include_preview:
            return SurfaceMeasurements(points, float(valid.mean()), b"")
        display = np.zeros(disparity.shape, dtype=np.uint8)
        display[valid] = np.clip(depth[valid] / 30 * 255, 0, 255).astype(np.uint8)
        color = cv2.applyColorMap(display, cv2.COLORMAP_TURBO)
        color[~valid] = 0
        accepted, encoded = cv2.imencode(".png", color)
        if not accepted:
            raise RuntimeError("Cannot encode measured-depth preview")
        return SurfaceMeasurements(points, float(valid.mean()), encoded.tobytes())
