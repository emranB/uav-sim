"""Synthetic, geographically aligned terrain colours from measured elevation."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import rasterio
from rasterio.enums import Resampling

from integrations.vision.terrain_mesh import export_visual_mesh


def prepare_material(directory: Path) -> None:
    """Keep low flattened water visually distinct; this is not photographic imagery."""
    metadata = json.loads((directory / "terrain.json").read_text())
    texture_size = 8192
    with rasterio.open(directory / "heightmap.png") as source:
        heights = source.read(
            1, out_shape=(texture_size, texture_size), resampling=Resampling.bilinear
        ).astype(np.float32)
    heights = heights / 65535 * (metadata["maximum_elevation_m"] - metadata["minimum_elevation_m"])
    heights += metadata["minimum_elevation_m"]
    elevations = heights
    random = np.random.default_rng(27)
    # A repeated authored fine texture gives the simulated cameras resolvable
    # ground detail. It is not measured imagery or evidence about real-water SLAM.
    tile = random.integers(-65, 66, (64, 64), dtype=np.int16)
    grain = np.tile(tile, (texture_size // 64, texture_size // 64))
    with rasterio.open(
        directory / "texture.png",
        "w",
        driver="PNG",
        width=texture_size,
        height=texture_size,
        count=3,
        dtype="uint8",
    ) as output:
        for band, (water, land) in enumerate(zip([27, 85, 111], [143, 148, 145]), 1):
            colours = np.where(elevations <= 0.05, water, land).astype(np.int16)
            output.write(np.clip(colours + grain, 0, 255).astype(np.uint8), band)
    metadata.update(
        uv_tile_m=metadata["width_m"],
        texture_scope="Synthetic neutral land and height-threshold water colours; not aerial imagery",
        texture_algorithm="elevation-water-mask-authored-fine-grain-v2",
        texture_sha256=hashlib.sha256((directory / "texture.png").read_bytes()).hexdigest(),
    )
    (directory / "terrain.json").write_text(json.dumps(metadata, indent=2) + "\n")
    export_visual_mesh(directory)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    prepare_material(parser.parse_args().directory)
