"""Blender geometry for time-indexed recorded world poses."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import bpy


def recorded_trail_curve(
    name: str, positions_enu_m: Sequence[tuple[float, float, float]]
) -> bpy.types.Curve:
    """Build a polyline whose visible end factor measures actual 3D arc length.

    Callers set the end factor to observed cumulative distance divided by total
    retained distance. Blender's default subdivision mapping would reveal a future
    segment for unevenly spaced samples, so this contract explicitly selects length.
    """
    import bpy

    if not 2 <= len(positions_enu_m) <= 20000 or any(
        len(position) != 3 or any(not math.isfinite(value) for value in position)
        for position in positions_enu_m
    ):
        raise ValueError("A recorded trail requires 2..20000 finite ENU positions")
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_factor_mapping_end = "SPLINE"
    curve.bevel_depth = 0.012
    curve.bevel_resolution = 1
    spline = curve.splines.new("POLY")
    spline.points.add(len(positions_enu_m) - 1)
    for point, position in zip(spline.points, positions_enu_m):
        point.co = (*position, 1)
    return curve
