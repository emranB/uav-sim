"""Batched CPU raster keeps the scalar half-cell lattice, bounds and hit precedence."""

import math
import random

from common.interfaces.obstacles import ObstacleRay
from common.navigation.occupancy import GridConfiguration, ObservedGrid


def scalar_cells(grid: ObservedGrid, rays: tuple[ObstacleRay, ...]) -> tuple[set, set]:
    """Reference the declared sample lattice independently of the array indexing."""
    free, hits = set(), set()
    step_m = grid.config.resolution_m / 2
    for ray in rays:
        x, y = ray.origin_xy_m
        dx, dy = ray.direction_xy
        for step in range(math.ceil(ray.free_distance_m / step_m) + 1):
            distance = min(ray.free_distance_m, step * step_m)
            cell = grid.cell((x + distance * dx, y + distance * dy))
            if grid.inside(cell):
                free.add(cell)
        if ray.occupied_start_m is None:
            continue
        lateral_width = 2 * ray.lateral_uncertainty_m
        for step in range(math.ceil((ray.occupied_end_m - ray.occupied_start_m) / step_m) + 1):
            distance = min(ray.occupied_end_m, ray.occupied_start_m + step * step_m)
            center_x, center_y = x + distance * dx, y + distance * dy
            for offset in range(math.ceil(lateral_width / step_m) + 1):
                lateral = -ray.lateral_uncertainty_m + min(lateral_width, offset * step_m)
                cell = grid.cell((center_x - lateral * dy, center_y + lateral * dx))
                if grid.inside(cell):
                    hits.add(cell)
    return free, hits


def test_batched_raster_preserves_measured_cells_across_batch_boundaries() -> None:
    random_source = random.Random(146)
    rays = [
        ObstacleRay((0, 0), (1, 0), 0.2, 0.2, 0.2, 0),
        ObstacleRay((0.2, -0.2), (0, 1), 0.4, None, None),
    ]
    for index in range(513):
        angle = random_source.uniform(-math.pi, math.pi)
        near = random_source.uniform(0.8, 18)
        rays.append(
            ObstacleRay(
                (random_source.uniform(-4, 4), random_source.uniform(-4, 4)),
                (math.cos(angle), math.sin(angle)),
                max(0, near - 0.1),
                near if index % 3 else None,
                near + random_source.uniform(0, 12) if index % 3 else None,
                random_source.uniform(0, 1.2),
            )
        )
    for origin in [(0, 0), (0.034227628260850906, 0.027003303170204)]:
        grid = ObservedGrid(origin, GridConfiguration())
        expected_free, expected_hits = scalar_cells(grid, tuple(rays))
        grid.observe_rays(10, tuple(rays))
        assert grid.free == {cell: 10 for cell in expected_free}
        assert grid.occupied == expected_hits
        grid.observe_rays(19, ())
        assert grid.free == {}
        assert grid.occupied == expected_hits
