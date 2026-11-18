"""Prepared visual grids preserve the collision image's quantized heights and local axes."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest


def test_visual_mesh_preserves_grid_extents_heights_and_upward_faces(tmp_path: Path) -> None:
    rasterio = pytest.importorskip(
        "rasterio", reason="Requires the isolated terrain-preparation image"
    )
    # GIS dependencies are intentionally optional in ordinary mapping deployments.
    from integrations.vision.terrain_mesh import export_visual_mesh

    metadata = dict(
        schema_version=1, width_m=20, height_m=40, minimum_elevation_m=-1, maximum_elevation_m=9
    )
    (tmp_path / "terrain.json").write_text(json.dumps(metadata))
    coordinates = np.arange(257)
    pixels = np.rint((coordinates[:, None] + coordinates[None, :]) * 65535 / 512).astype(np.uint16)
    with rasterio.open(
        tmp_path / "heightmap.png",
        "w",
        driver="PNG",
        width=257,
        height=257,
        count=1,
        dtype="uint16",
    ) as output:
        output.write(pixels, 1)
    report = export_visual_mesh(tmp_path)
    lines = (tmp_path / "terrain.obj").read_text().splitlines()
    vertices = [list(map(float, line.split()[1:])) for line in lines if line.startswith("v ")]
    normals = [list(map(float, line.split()[1:])) for line in lines if line.startswith("vn ")]
    assert vertices[0] == [-10, 20, 0]
    assert vertices[-1] == [10, -20, 10]
    assert normals[0][0] < 0 and normals[0][1] > 0 and normals[0][2] > 0
    face = next(line for line in lines if line.startswith("f "))
    assert face == "f 1/1/1 258/258/258 2/2/2"
    assert report["triangles"] == 2 * 256 * 256
    assert (
        report["mesh_sha256"] == hashlib.sha256((tmp_path / "terrain.obj").read_bytes()).hexdigest()
    )
