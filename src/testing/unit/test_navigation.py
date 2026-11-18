"""Measured free space, unknown space and unscripted static detours."""

import math
import time
import unittest

from common.interfaces.navigation import NavigationPose, RangeScan
from common.navigation.occupancy import GridConfiguration, ObservedGrid


def scan_at(x: float, y: float) -> RangeScan:
    """Synthetic ray/box intersection fixture; never used by the application."""
    ranges = []
    for index in range(720):
        angle = -math.pi + index * 2 * math.pi / 719
        dx, dy = math.cos(angle), math.sin(angle)
        entry, exit_distance = -math.inf, math.inf
        for origin, direction, lower, upper in ((x, dx, -2, 2), (y, dy, 6, 8)):
            if abs(direction) < 1e-12:
                if not lower <= origin <= upper:
                    entry = math.inf
            else:
                intersections = sorted(((lower - origin) / direction, (upper - origin) / direction))
                entry, exit_distance = (
                    max(entry, intersections[0]),
                    min(exit_distance, intersections[1]),
                )
        ranges.append(entry if 0 < entry < exit_distance and entry <= 20 else math.inf)
    return RangeScan(1, time.monotonic(), -math.pi, 2 * math.pi / 719, 20, tuple(ranges))


class NavigationGridTests(unittest.TestCase):
    def test_unknown_grid_does_not_offer_a_route(self) -> None:
        grid = ObservedGrid((0, 0), GridConfiguration())
        self.assertEqual(grid.plan((0, 0), (0, 14)), [])

    def test_scan_plans_observed_detour_without_box_input(self) -> None:
        grid = ObservedGrid((0, 0), GridConfiguration())
        grid.observe(
            scan_at(0, 0), NavigationPose((0, 0, 4), 0, time.monotonic(), "a", True, "OFFBOARD")
        )
        path = grid.plan((0, 0), (0, 14))
        self.assertGreater(len(path), 5)
        self.assertGreater(max(abs(point[0]) for point in path), 3)
        for point in path:
            self.assertFalse(-3.1 < point[0] < 3.1 and 4.9 < point[1] < 9.1)
        self.assertNotEqual(path[-1], (0, 14))  # Occluded goal is initially unknown.

    def test_invalid_rays_do_not_create_free_space(self) -> None:
        grid = ObservedGrid((0, 0), GridConfiguration())
        grid.observe(
            RangeScan(1, 1, 0, 0.1, 20, (math.nan,) * 720),
            NavigationPose((0, 0, 4), 0, 1, "a", True, "OFFBOARD"),
        )
        self.assertFalse(grid.free)

    def test_scan_clock_jump_requires_reset(self) -> None:
        grid = ObservedGrid((0, 0), GridConfiguration())
        pose = NavigationPose((0, 0, 4), 0, 1, "a", True, "OFFBOARD")
        grid.observe(scan_at(0, 0), pose)
        with self.assertRaisesRegex(RuntimeError, "backwards"):
            grid.observe(RangeScan(0, 1, 0, 0.1, 20, (math.inf,) * 720), pose)

    def test_only_observed_free_rays_clear_previous_endpoints(self) -> None:
        grid = ObservedGrid((0, 0), GridConfiguration())
        pose = NavigationPose((0, 0, 4), 0, 1, "a", True, "OFFBOARD")
        grid.observe(RangeScan(1, 1, 0, 0.1, 20, (5.0,)), pose)
        self.assertTrue(grid.occupied)
        grid.observe(RangeScan(2, 2, math.pi, 0.1, 20, (math.inf,)), pose)
        self.assertTrue(grid.occupied)  # Unseen is not free.
        grid.observe(RangeScan(3, 3, 0, 0.1, 20, (math.inf,)), pose)
        self.assertFalse(grid.occupied)

    def test_vehicle_can_leave_margin_through_observed_increasing_clearance(self) -> None:
        grid = ObservedGrid((0, 0), GridConfiguration())
        pose = NavigationPose((0, 4.8, 4), 0, 1, "a", True, "OFFBOARD")
        grid.observe(scan_at(0, 4.8), pose)
        self.assertNotIn(grid.cell((0, 4.8)), grid.traversable())
        path = grid.plan((0, 4.8), (0, 14))
        self.assertGreater(len(path), 1)
        self.assertLess(path[-1][1], path[0][1])
        for point in path:
            self.assertIn(grid.cell(point), grid.free)
        self.assertIn(grid.cell(path[-1]), grid.traversable())

    def test_grid_bounds_are_not_silently_expanded(self) -> None:
        with self.assertRaisesRegex(ValueError, "outside"):
            ObservedGrid((0, 0), GridConfiguration()).plan((0, 0), (40, 0))
