"""Bounded static 2D occupancy from horizontal measured range rays."""

import heapq
import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from common.interfaces.navigation import NavigationPose, RangeScan
from common.interfaces.obstacles import ObstacleRay

Cell = tuple[int, int]


@dataclass(frozen=True)
class GridConfiguration:
    """Fixed-altitude demo bounds; inflation includes the vehicle and tracking margin."""

    resolution_m: float = 0.4
    half_width_m: float = 18.0
    inflation_m: float = 1.6
    cell_lifetime_seconds: float = 8.0


class ObservedGrid:
    """Unknown cells never become traversable merely because no obstacle was recorded."""

    def __init__(self, origin_xy_m: tuple[float, float], config: GridConfiguration) -> None:
        self.origin = origin_xy_m
        self.config = config
        self.free: dict[Cell, float] = {}
        self.occupied: set[Cell] = set()
        self.timestamp_seconds = -1.0

    def cell(self, point: tuple[float, float]) -> Cell:
        return tuple(round((point[i] - self.origin[i]) / self.config.resolution_m) for i in (0, 1))

    def point(self, cell: Cell) -> tuple[float, float]:
        return tuple(self.origin[i] + cell[i] * self.config.resolution_m for i in (0, 1))

    def inside(self, cell: Cell) -> bool:
        limit = int(self.config.half_width_m / self.config.resolution_m)
        return max(abs(cell[0]), abs(cell[1])) <= limit

    def observe(self, scan: RangeScan, pose: NavigationPose) -> None:
        """Integrate timestamp-ordered static observations, rejecting clock jumps."""
        if scan.timestamp_seconds < self.timestamp_seconds:
            raise RuntimeError("Range sensor clock moved backwards; reset navigation")
        self.timestamp_seconds = scan.timestamp_seconds
        x, y, _ = pose.position_enu_m
        measured_hits: set[Cell] = set()
        measured_free: set[Cell] = set()
        for index, distance in enumerate(scan.ranges_m):
            if math.isnan(distance) or distance < 0.8:
                continue
            hit = math.isfinite(distance) and distance < scan.maximum_range_m - 0.1
            length = min(distance, scan.maximum_range_m, self.config.half_width_m * 2)
            angle = pose.yaw_enu_rad + scan.angle_min_rad + index * scan.angle_step_rad
            end = self.cell((x + length * math.cos(angle), y + length * math.sin(angle)))
            if hit and self.inside(end):
                measured_hits.add(end)
            # Half-cell steps avoid gaps between raster cells along a ray.
            steps = max(1, math.ceil(length / (self.config.resolution_m / 2)))
            for step in range(steps + (0 if hit else 1)):
                fraction = step / steps
                cell = self.cell(
                    (
                        x + length * fraction * math.cos(angle),
                        y + length * fraction * math.sin(angle),
                    )
                )
                if self.inside(cell):
                    measured_free.add(cell)
                    self.free[cell] = scan.timestamp_seconds
        # A current free ray can contradict an old endpoint displaced by estimate noise.
        # Hits from the same scan win; unseen cells never clear an occupied endpoint.
        self.occupied.difference_update(measured_free - measured_hits)
        self.occupied.update(measured_hits)
        oldest = scan.timestamp_seconds - self.config.cell_lifetime_seconds
        self.free = {cell: stamp for cell, stamp in self.free.items() if stamp >= oldest}

    def observe_rays(self, timestamp_seconds: float, rays: tuple[ObstacleRay, ...]) -> None:
        """Integrate only measured free prefixes and uncertainty-expanded occupied bands."""
        if timestamp_seconds < self.timestamp_seconds:
            raise RuntimeError("Obstacle measurement clock moved backwards; reset navigation")
        self.timestamp_seconds = timestamp_seconds
        resolution = self.config.resolution_m
        half_cell = resolution / 2
        limit = int(self.config.half_width_m / resolution)
        side_cells = 2 * limit + 1
        origin_x, origin_y = self.origin
        # Deduplicate within the fixed grid instead of retaining and sorting
        # every ray sample after all batches have been processed.
        free_mask = np.zeros(side_cells * side_cells, dtype=bool)
        occupied_mask = np.zeros(side_cells * side_cells, dtype=bool)

        def encode(x: np.ndarray, y: np.ndarray) -> np.ndarray:
            x, y = np.rint(x), np.rint(y)
            inside = (x >= -limit) & (x <= limit) & (y >= -limit) & (y <= limit)
            return (
                (y[inside].astype(np.int64) + limit) * side_cells
                + x[inside].astype(np.int64)
                + limit
            )

        def indices(counts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            ray_index = np.repeat(np.arange(len(counts)), counts)
            starts = np.cumsum(counts) - counts
            step = np.arange(int(counts.sum())) - np.repeat(starts, counts)
            return ray_index, step

        # Bound temporary arrays independently of the number of measured pixels.
        # Half-cell samples, arithmetic order and ties-to-even rounding stay
        # identical to the scalar raster; batching only removes Python loops.
        for offset in range(0, len(rays), 128):
            batch = rays[offset : offset + 128]
            values = np.array(
                [(*ray.origin_xy_m, *ray.direction_xy, ray.free_distance_m) for ray in batch],
                dtype=np.float64,
            )
            counts = np.ceil(values[:, 4] / half_cell).astype(np.int64) + 1
            ray_index, step = indices(counts)
            distance = np.minimum(values[ray_index, 4], step * half_cell)
            free_mask[
                encode(
                    (values[ray_index, 0] + distance * values[ray_index, 2] - origin_x)
                    / resolution,
                    (values[ray_index, 1] + distance * values[ray_index, 3] - origin_y)
                    / resolution,
                )
            ] = True
            occupied = [ray for ray in batch if ray.occupied_start_m is not None]
            if not occupied:
                continue
            values = np.array(
                [
                    (
                        *ray.origin_xy_m,
                        *ray.direction_xy,
                        ray.occupied_start_m,
                        ray.occupied_end_m,
                        ray.lateral_uncertainty_m,
                    )
                    for ray in occupied
                ],
                dtype=np.float64,
            )
            radial_counts = np.ceil((values[:, 5] - values[:, 4]) / half_cell).astype(np.int64) + 1
            lateral_counts = np.ceil(2 * values[:, 6] / half_cell).astype(np.int64) + 1
            ray_index, step = indices(radial_counts * lateral_counts)
            radial_step = step // lateral_counts[ray_index]
            lateral_step = step % lateral_counts[ray_index]
            distance = np.minimum(
                values[ray_index, 5], values[ray_index, 4] + radial_step * half_cell
            )
            lateral = -values[ray_index, 6] + np.minimum(
                2 * values[ray_index, 6],
                lateral_step * half_cell,
            )
            center_x = values[ray_index, 0] + distance * values[ray_index, 2]
            center_y = values[ray_index, 1] + distance * values[ray_index, 3]
            occupied_mask[
                encode(
                    (center_x - lateral * values[ray_index, 3] - origin_x) / resolution,
                    (center_y + lateral * values[ray_index, 2] - origin_y) / resolution,
                )
            ] = True

        def decode(mask: np.ndarray) -> set[Cell]:
            return {
                (code % side_cells - limit, code // side_cells - limit)
                for code in np.flatnonzero(mask).tolist()
            }

        free = decode(free_mask)
        hits = decode(occupied_mask)
        self.occupied.difference_update(free - hits)
        self.occupied.update(hits)
        self.free.update({cell: timestamp_seconds for cell in free})
        oldest = timestamp_seconds - self.config.cell_lifetime_seconds
        self.free = {cell: stamp for cell, stamp in self.free.items() if stamp >= oldest}

    def traversable(self) -> set[Cell]:
        """Require an observed footprint and inflate occupied endpoints conservatively."""
        radius = math.ceil(self.config.inflation_m / self.config.resolution_m)
        limit = int(self.config.half_width_m / self.config.resolution_m)
        side = 2 * limit + 1
        # Padding keeps shifts inside the bounded map; unobserved edge cells
        # remain false. This is the same circular inflation and 3x3 footprint.
        hits = np.zeros((side + 2 * radius, side + 2 * radius), dtype=bool)
        for x, y in self.occupied:
            hits[y + limit + radius, x + limit + radius] = True
        blocked = np.zeros((side, side), dtype=bool)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:
                    blocked |= hits[
                        radius + dy : radius + dy + side, radius + dx : radius + dx + side
                    ]
        free = np.zeros((side + 2, side + 2), dtype=bool)
        for x, y in self.free:
            free[y + limit + 1, x + limit + 1] = True
        allowed = ~blocked
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                allowed &= free[1 + dy : 1 + dy + side, 1 + dx : 1 + dx + side]
        ys, xs = np.nonzero(allowed)
        return set(zip((xs - limit).tolist(), (ys - limit).tolist()))

    def _escape_margin(self, start: Cell, allowed: set[Cell]) -> list[Cell]:
        """Leave a conservative margin only through measured, nondecreasing clearance.

        The margin is larger than the physical vehicle. A raster boundary or newly
        observed endpoint can put the current estimate inside it without a collision.
        Never escape from within the vehicle/braking envelope or through unknown space.
        """
        if start not in self.free or not self.occupied:
            return []

        def separation(cell: Cell) -> float:
            return min(math.dist(cell, hit) for hit in self.occupied) * self.config.resolution_m

        minimum_separation_m = 1.1  # 0.5m body + 0.32m braking + 0.28m half-cell diagonal.
        if separation(start) < minimum_separation_m:
            return []
        pending = deque([[start]])
        visited = {start}
        while pending:
            path = pending.popleft()
            current = path[-1]
            if current in allowed:
                return path
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbour = current[0] + dx, current[1] + dy
                if (
                    neighbour not in visited
                    and all(
                        (neighbour[0] + x, neighbour[1] + y) in self.free
                        for x in (-1, 0, 1)
                        for y in (-1, 0, 1)
                    )
                    and math.dist(neighbour, start) * self.config.resolution_m <= 3.2
                    and separation(neighbour) >= separation(current) - 1e-6
                ):
                    visited.add(neighbour)
                    pending.append([*path, neighbour])
        return []

    def plan(self, start_xy_m: tuple[float, float], goal_xy_m: tuple[float, float]) -> list:
        """Find an observed path or a strictly goal-progressing observed frontier.

        No preauthored detours or obstacle definitions enter this operation. Search is
        bounded by the configured grid. Four-neighbour edges cannot cut occupied corners.
        """
        start, goal = self.cell(start_xy_m), self.cell(goal_xy_m)
        if not self.inside(goal):
            raise ValueError("Goal is outside the bounded navigation grid")
        allowed = self.traversable()
        if start not in allowed:
            return [self.point(cell) for cell in self._escape_margin(start, allowed)]
        frontier = [(0.0, start)]
        cost = {start: 0.0}
        parent = {}
        while frontier:
            distance, current = heapq.heappop(frontier)
            if distance != cost[current]:
                continue
            if current == goal:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbour = current[0] + dx, current[1] + dy
                candidate = distance + 1
                if neighbour in allowed and candidate < cost.get(neighbour, math.inf):
                    cost[neighbour] = candidate
                    parent[neighbour] = current
                    heapq.heappush(frontier, (candidate, neighbour))
        if goal in cost:
            selected = goal
        else:
            selected = min(cost, key=lambda cell: (math.dist(cell, goal), cost[cell]))
            if math.dist(selected, goal) >= math.dist(start, goal) - 1:
                return []
        cells = [selected]
        while cells[-1] != start:
            cells.append(parent[cells[-1]])
        return [self.point(cell) for cell in reversed(cells)]
