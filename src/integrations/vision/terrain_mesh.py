"""Export a calibrated height grid as a portable visual mesh, outside simulator runtime."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rasterio


def export_visual_mesh(directory: Path) -> dict:
    """Preserve quantized heights, north-up grid, local metres and twenty-metre UV tiles."""
    metadata = json.loads((directory / "terrain.json").read_text())
    if metadata.get("schema_version") != 1:
        raise ValueError("Terrain mesh export requires metadata schema version 1")
    width, height = metadata["width_m"], metadata["height_m"]
    relief = metadata["maximum_elevation_m"] - metadata["minimum_elevation_m"]
    if not all(math.isfinite(value) and value > 0 for value in (width, height, relief)):
        raise ValueError("Terrain mesh dimensions must be finite positive metres")
    source = directory / "heightmap.png"
    with rasterio.open(source) as image:
        if image.count != 1 or image.dtypes[0] != "uint16" or image.width != image.height:
            raise ValueError("Terrain visual requires one square unsigned 16-bit height grid")
        if image.width not in {257, 513, 1025}:
            raise ValueError("Terrain visual grid must have 257, 513 or 1025 samples")
        heights = image.read(1).astype(np.float64) * relief / 65535
    count = heights.shape[0]
    uv_tile_m = float(metadata.get("uv_tile_m", 20))
    if not math.isfinite(uv_tile_m) or not 1 <= uv_tile_m <= 10000:
        raise ValueError("Terrain material tile must lie within1..10000m")
    slope_y, slope_x = np.gradient(heights, height / (count - 1), width / (count - 1))
    normals = np.stack((-slope_x, slope_y, np.ones_like(heights)), axis=2)
    normals /= np.linalg.norm(normals, axis=2, keepdims=True)
    path = directory / "terrain.obj"
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# Local terrain grid in metres; placement belongs to the model pose.\n")
        for row in range(count):
            for column in range(count):
                x = column * width / (count - 1) - width / 2
                y = height / 2 - row * height / (count - 1)
                stream.write(f"v {x:.9f} {y:.9f} {heights[row, column]:.9f}\n")
        for row in range(count):
            for column in range(count):
                stream.write(
                    f"vt {column * width / (count - 1) / uv_tile_m:.9f} {1 - row * height / (count - 1) / uv_tile_m:.9f}\n"
                )
        for row in range(count):
            for column in range(count):
                nx, ny, nz = normals[row, column]
                stream.write(f"vn {nx:.9f} {ny:.9f} {nz:.9f}\n")
        for row in range(count - 1):
            for column in range(count - 1):
                a = row * count + column + 1
                b, c, d = a + count, a + 1, a + count + 1
                stream.write(
                    f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\nf {c}/{c}/{c} {b}/{b}/{b} {d}/{d}/{d}\n"
                )
    with path.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    report = dict(
        schema_version=1,
        mesh_file="terrain.obj",
        mesh_sha256=digest,
        source_heightmap_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        samples=count,
        width_m=width,
        height_m=height,
        relief_m=relief,
        vertices=count * count,
        triangles=2 * (count - 1) ** 2,
        frame="Heightmap-local metres; north-up; model pose applies ENU offset and yaw",
    )
    (directory / "terrain-visual.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(export_visual_mesh(args.directory)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
