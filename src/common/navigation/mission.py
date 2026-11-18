"""Validated saved-mission intent for the bounded visual comparison envelope."""

import math
import re
from dataclasses import dataclass

from common.configuration.missions import parse_plan
from common.configuration.visual_missions import VisualMissionLimits


@dataclass(frozen=True)
class VisualWaypoint:
    """Launch-relative ENU metres and a simulation-time hold at one route corner."""

    offset_enu_m: tuple[float, float]
    hold_sim_seconds: float


@dataclass(frozen=True)
class VisualMission:
    """Ordered outbound waypoints and home, bound to each vehicle's estimated launch."""

    name: str
    sha256: str
    goal_offset_enu_m: tuple[float, float]
    waypoint_hold_sim_seconds: float
    waypoints: tuple[VisualWaypoint, ...] = ()
    maximum_radius_m: float = 10.0
    maximum_total_distance_m: float = 60.0
    maximum_navigation_wall_seconds: float = 330.0

    @property
    def outbound_waypoints(self) -> tuple[VisualWaypoint, ...]:
        """Keep the original single-waypoint constructor compatible."""
        return self.waypoints or (
            VisualWaypoint(self.goal_offset_enu_m, self.waypoint_hold_sim_seconds),
        )

    @property
    def planned_distance_m(self) -> float:
        """Polyline length through the supplied corners and back to launch."""
        points = [(0.0, 0.0), *(p.offset_enu_m for p in self.outbound_waypoints), (0.0, 0.0)]
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))

    @classmethod
    def from_plan(
        cls, name: str, sha256: str, plan: dict, limits: VisualMissionLimits | None = None
    ) -> "VisualMission":
        """Convert geographic offsets from planned home; never read simulator truth."""
        if not isinstance(sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", sha256):
            raise ValueError("Visual mission requires the saved file's SHA-256")
        items = parse_plan(plan)
        limits = limits or VisualMissionLimits()
        if (
            not 3 <= len(items) <= 14
            or items[0].command != 22
            or items[-1].command != 20
            or any(item.command != 16 for item in items[1:-1])
        ):
            raise ValueError("Visual route requires takeoff, 1..12 ordered waypoints and RTL")
        home = plan["mission"].get("plannedHomePosition")
        if (
            not isinstance(home, list)
            or len(home) != 3
            or any(type(value) not in {int, float} or not math.isfinite(value) for value in home)
            or not -85 <= home[0] <= 85
            or not -180 <= home[1] <= 180
        ):
            raise ValueError("Visual mission needs a finite planned home away from the poles")
        takeoff = items[0]
        if (
            any(
                item.parameters[1:3] != (0, 0) or not math.isnan(item.parameters[3])
                for item in items[:-1]
            )
            or takeoff.parameters[0] != 0
        ):
            raise ValueError("Visual mission requires default yaw, acceptance and takeoff settings")
        # Downloaded plan headers use the current GNSS home estimate, while mission
        # coordinates are quantized separately. Accept centimetre-scale metadata noise.
        north_error_m = math.radians(takeoff.parameters[4] - home[0]) * 6378137
        east_error_m = (
            math.radians(takeoff.parameters[5] - home[1])
            * 6378137
            * math.cos(math.radians(home[0]))
        )
        if math.hypot(east_error_m, north_error_m) > 0.25:
            raise ValueError("Takeoff must identify the saved planned home within 0.25 m")
        if any(item.parameters[6] != 4 for item in items[:-1]):
            raise ValueError("The visual comparison envelope requires four metres above launch")
        waypoints = []
        previous = (0.0, 0.0)
        for index, waypoint in enumerate(items[1:-1], 1):
            north = math.radians(waypoint.parameters[4] - home[0]) * 6378137
            east = (
                math.radians(waypoint.parameters[5] - home[1])
                * 6378137
                * math.cos(math.radians(home[0]))
            )
            if not 2 <= math.hypot(east, north) <= limits.maximum_radius_m:
                raise ValueError(
                    f"Visual waypoint {index} must lie 2..{limits.maximum_radius_m:g} metres from home"
                )
            if math.dist(previous, (east, north)) < 0.8:
                raise ValueError(
                    f"Visual waypoint {index} is less than 0.8m from the previous point"
                )
            hold = waypoint.parameters[0]
            if not 0 <= hold <= 10:
                raise ValueError("Waypoint hold must lie within 0..10 simulation seconds")
            waypoints.append(VisualWaypoint((east, north), hold))
            previous = (east, north)
        route = cls(
            name,
            sha256,
            waypoints[0].offset_enu_m,
            waypoints[0].hold_sim_seconds,
            tuple(waypoints),
            limits.maximum_radius_m,
            limits.maximum_total_distance_m,
            limits.maximum_navigation_wall_seconds,
        )
        if route.planned_distance_m > limits.maximum_total_distance_m:
            raise ValueError("Visual route including return exceeds the configured distance budget")
        return route
