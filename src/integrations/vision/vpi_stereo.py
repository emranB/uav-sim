"""Optional CUDA stereo matching, isolated from the native pose estimator."""

import cv2
import numpy as np


class VpiStereoMatcher:
    """Own CUDA stereo buffers for one serial caller; return pixel disparities."""

    def __init__(self) -> None:
        try:
            import vpi  # Optional deployment dependency; CPU operation must not import it.
        except ImportError as error:
            raise RuntimeError(
                "VPI CUDA stereo is unavailable; select the cpu surface backend"
            ) from error
        self.vpi = vpi

    def compute(self, images: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
        """Match both directions; VPI requires nonnegative minimum disparity.

        Mirroring the swapped pair expresses reverse correspondence as positive
        disparity. Undo that mirror and negate before the shared consistency check.
        VPI output has five fractional bits, independent of OpenCV's four-bit format.
        """
        options = dict(
            backend=self.vpi.Backend.CUDA, maxdisp=64, window=5, uniqueness=0.88, p1=8, p2=32
        )
        left, right = [self.vpi.asimage(image) for image in images]
        forward = self.vpi.stereodisp(left, right, **options)
        mirrored = [self.vpi.asimage(np.ascontiguousarray(image[:, ::-1])) for image in images]
        backward = self.vpi.stereodisp(mirrored[1], mirrored[0], **options)
        raw_forward = forward.cpu().copy()
        raw_backward = backward.cpu().copy()
        for raw in (raw_forward, raw_backward):
            cv2.filterSpeckles(raw, -32768, 50, 2 * 32)
        disparity = raw_forward.astype(np.float32) / 32
        reverse = -raw_backward[:, ::-1].astype(np.float32) / 32
        return disparity, reverse
