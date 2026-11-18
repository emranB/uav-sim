"""Validated camera geometry shared by acquisition and model generation."""

import math
import os
from dataclasses import dataclass
from typing import ClassVar


@dataclass(frozen=True)
class StereoConfiguration:
    mount_forward_m: ClassVar[float] = 0.4
    width: int = 320
    height: int = 240
    rate_hz: int = 5
    horizontal_fov_rad: float = 1.0
    mount_pitch_rad: float = 0.7
    baseline_m: float = 0.16
    mount_height_m: float = 0.15

    @classmethod
    def configured(cls, *, maximum_rate_hz: int = 10) -> "StereoConfiguration":
        """Read camera settings, with a separate rate ceiling for opt-in consumers."""
        width = int(os.environ.get("UAV_CAMERA_WIDTH", "320"))
        height = int(os.environ.get("UAV_CAMERA_HEIGHT", "240"))
        rate = int(os.environ.get("UAV_CAMERA_RATE_HZ", "5"))
        if (width, height) not in {(320, 240), (640, 480)} or not 1 <= rate <= maximum_rate_hz:
            raise ValueError(
                f"Stereo requires 320x240 or 640x480 and an integer rate in 1..{maximum_rate_hz} Hz"
            )
        fov = float(os.environ.get("UAV_CAMERA_FOV_RAD", "1.0"))
        pitch = float(os.environ.get("UAV_CAMERA_PITCH_RAD", "0.7"))
        if not math.isfinite(fov) or not 0.5 <= fov <= 1.6:
            raise ValueError("Camera horizontal field of view must be in 0.5..1.6 radians")
        if not math.isfinite(pitch) or not 0 <= pitch <= math.pi / 2:
            raise ValueError("Camera downward pitch must be in 0..pi/2 radians")
        baseline = float(os.environ.get("UAV_CAMERA_BASELINE_M", "0.16"))
        if not math.isfinite(baseline) or not 0.05 <= baseline <= 0.8:
            raise ValueError("Stereo baseline must be in 0.05..0.8 metres")
        height_m = float(os.environ.get("UAV_CAMERA_HEIGHT_M", "0.15"))
        if not math.isfinite(height_m) or not 0.05 <= height_m <= 2:
            raise ValueError("Camera mount height must be in 0.05..2 metres")
        return cls(width, height, rate, fov, pitch, baseline, height_m)
