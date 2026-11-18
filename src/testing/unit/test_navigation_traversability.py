"""Inflated obstacles and unknown footprint edges remain impassable."""

import random
import unittest

from common.navigation.occupancy import GridConfiguration, ObservedGrid


class NavigationTraversabilityTests(unittest.TestCase):
    def test_randomized_bounded_maps_match_clearance_and_full_footprint_rules(self) -> None:
        random_source = random.Random(809)
        for inflation_m in (0.0, 0.4, 1.6):
            for trial in range(8):
                with self.subTest(inflation_m=inflation_m, trial=trial):
                    grid = ObservedGrid(
                        (0.13, -0.27), GridConfiguration(half_width_m=4.0, inflation_m=inflation_m)
                    )
                    cells = [(x, y) for x in range(-10, 11) for y in range(-10, 11)]
                    grid.free = {cell: 1.0 for cell in cells if random_source.random() < 0.9}
                    grid.occupied = set(random_source.sample(cells, 12))
                    radius = round(inflation_m / grid.config.resolution_m)
                    expected = {
                        (x, y)
                        for x, y in grid.free
                        if all(
                            (x + dx, y + dy) in grid.free for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                        )
                        and all(
                            (x - hx) ** 2 + (y - hy) ** 2 > radius**2 for hx, hy in grid.occupied
                        )
                    }
                    self.assertEqual(grid.traversable(), expected)

    def test_empty_obstacles_still_require_observed_footprint_at_grid_edge(self) -> None:
        grid = ObservedGrid((0, 0), GridConfiguration(half_width_m=0.8))
        grid.free = {(x, y): 1.0 for x in range(-2, 3) for y in range(-2, 3)}
        self.assertEqual(grid.traversable(), {(x, y) for x in range(-1, 2) for y in range(-1, 2)})
        grid.free.pop((0, 0))
        self.assertEqual(grid.traversable(), set())
