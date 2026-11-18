"""Finite deployment limits for saved visual routes, independent of transport."""

import math
from collections.abc import Mapping
from dataclasses import dataclass


@dataclass(frozen=True)
class VisualMissionLimits:
    maximum_radius_m: float = 10.0
    maximum_total_distance_m: float = 60.0
    maximum_navigation_wall_seconds: float = 330.0

    def __post_init__(self) -> None:
        for name, lower, upper in (
            ("maximum_radius_m", 2, 100),
            ("maximum_total_distance_m", 2, 600),
            ("maximum_navigation_wall_seconds", 30, 2400),
        ):
            value = getattr(self, name)
            if (
                type(value) not in {int, float}
                or not math.isfinite(value)
                or not lower <= value <= upper
            ):
                raise ValueError(f"Visual mission {name} must be finite in {lower}..{upper}")

    @classmethod
    def from_environment(cls, environment: Mapping[str, str]) -> "VisualMissionLimits":
        """Read explicit deployment limits; invalid values fail startup."""
        return cls(
            float(environment.get("UAV_VISUAL_MISSION_MAX_RADIUS_M", "10")),
            float(environment.get("UAV_VISUAL_MISSION_MAX_DISTANCE_M", "60")),
            float(environment.get("UAV_VISUAL_MISSION_TIMEOUT_SECONDS", "330")),
        )
