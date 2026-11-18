"""Offline GIS geometry checks; run in the optional geography image."""

import numpy as np
import pytest
from shapely.geometry import Polygon

from integrations.geography.buildings import extrude_polygon


def test_extruded_roof_preserves_courtyard_and_vertical_limits() -> None:
    polygon = Polygon(
        [(0, 0), (10, 0), (10, 10), (0, 10)], holes=[[(3, 3), (7, 3), (7, 7), (3, 7)]]
    )
    vertices, faces = extrude_polygon(polygon, -1, 12)
    roof_area = 0
    for face in faces:
        points = [vertices[i] for i in face]
        assert all(point[2] in {-1, 12} for point in points)
        if all(point[2] == 12 for point in points):
            triangle = Polygon([(p[0], p[1]) for p in points])
            assert polygon.covers(triangle)
            roof_area += triangle.area
    assert roof_area == pytest.approx(84)
    signed_volume = sum(
        np.dot(vertices[a], np.cross(vertices[b], vertices[c])) / 6 for a, b, c in faces
    )
    assert signed_volume == pytest.approx(84 * 13)


def test_invalid_height_is_rejected() -> None:
    with pytest.raises(ValueError, match="positive height"):
        extrude_polygon(Polygon([(0, 0), (1, 0), (0, 1)]), 5, 4)
