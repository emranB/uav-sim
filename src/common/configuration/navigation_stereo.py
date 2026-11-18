"""Declared horizontal stereo geometry, independent of simulator and GUI libraries."""

import hashlib
import json
import math
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class NavigationStereoCalibration:
    """Parallel pinhole pairs; left optical +Z forward,+X right,+Y down, metres."""

    width: int = 320
    height: int = 240
    baseline_m: float = 0.4
    horizontal_fov_rad: float = 1.8
    mount_height_m: float = 0.35
    rate_hz: int = 5
    rig_count: int = 6

    @property
    def focal_px(self) -> float:
        return self.width / (2 * math.tan(self.horizontal_fov_rad / 2))

    @property
    def identifier(self) -> str:
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()

    def yaw_rad(self, rig: int) -> float:
        if not 0 <= rig < self.rig_count:
            raise ValueError("Choose a configured horizontal stereo rig")
        return rig * 2 * math.pi / self.rig_count
