"""Prepare cached elevation assets independently of the running simulation."""

import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path

import numpy as np
import rasterio
from pyproj import Proj, Transformer
from rasterio.enums import Resampling
from rasterio.windows import Window

from integrations.vision.terrain_mesh import export_visual_mesh


def prepare_terrain(source: Path, destination: Path, configuration: dict) -> dict:
    """Resample a metric DEM and retain explicit horizontal and vertical references."""
    size_m = configuration["extent_m"]
    samples = configuration["samples"]
    latitude, longitude = configuration["latitude_deg"], configuration["longitude_deg"]
    if (
        type(latitude) not in {int, float}
        or not math.isfinite(latitude)
        or not -90 <= latitude <= 90
        or type(longitude) not in {int, float}
        or not math.isfinite(longitude)
        or not -180 <= longitude <= 180
        or type(configuration["texture_seed"]) is not int
    ):
        raise ValueError("Terrain needs finite geographic coordinates and an integer texture seed")
    if type(samples) is not int or samples not in {257, 513, 1025}:
        raise ValueError("Terrain samples must be 257, 513 or 1025")
    if type(size_m) not in {int, float} or not math.isfinite(size_m) or not 100 <= size_m <= 10000:
        raise ValueError("Terrain extent must be 100..10000 metres")
    destination.mkdir(parents=True, exist_ok=True)
    with rasterio.open(source) as dataset:
        if dataset.crs is None or not dataset.crs.is_projected:
            raise ValueError("Terrain source requires a projected metric CRS")
        if dataset.crs.linear_units not in {"metre", "meter"}:
            raise ValueError("Terrain source horizontal units must be metres")
        east, north = Transformer.from_crs("EPSG:4326", dataset.crs, always_xy=True).transform(
            longitude,
            latitude,
        )
        if configuration.get("full_source", False):
            bounds = dataset.bounds
            width_m, height_m = bounds.right - bounds.left, bounds.top - bounds.bottom
            window = Window(0, 0, dataset.width, dataset.height)
            center_east, center_north = (
                (bounds.left + bounds.right) / 2,
                (bounds.top + bounds.bottom) / 2,
            )
        else:
            row, column = dataset.index(east, north)
            width_m = height_m = float(size_m)
            window = Window(
                column - size_m / dataset.res[0] / 2,
                row - size_m / dataset.res[1] / 2,
                size_m / dataset.res[0],
                size_m / dataset.res[1],
            )
            left, bottom, right, top = rasterio.windows.bounds(window, dataset.transform)
            center_east, center_north = (left + right) / 2, (bottom + top) / 2
        if (
            window.col_off < 0
            or window.row_off < 0
            or window.col_off + window.width > dataset.width
            or window.row_off + window.height > dataset.height
        ):
            raise ValueError("Terrain area extends outside the downloaded source")
        heights = dataset.read(
            1,
            window=window,
            out_shape=(samples, samples),
            masked=True,
            resampling=Resampling.bilinear,
        )
        if np.ma.getmaskarray(heights).any() or not np.isfinite(heights.data).all():
            raise ValueError("Terrain contains missing elevations; no silent gap filling")
        home_height = next(dataset.sample([(east, north)]))[0]
        if home_height == dataset.nodata or not np.isfinite(home_height):
            raise ValueError("Terrain home has no valid elevation")
        low, high = float(heights.min()), float(heights.max())
        if high - low < 0.001:
            raise ValueError("Selected terrain has no measurable relief")
        image = np.rint((heights.data - low) / (high - low) * 65535).astype(np.uint16)
        with rasterio.open(
            destination / "heightmap.png",
            "w",
            driver="PNG",
            width=samples,
            height=samples,
            count=1,
            dtype="uint16",
        ) as output:
            output.write(image, 1)
        # Synthetic material supplies visible texture, not geographic aerial imagery.
        random = np.random.default_rng(configuration["texture_seed"])
        # Broad intensity structure survives perspective changes and minification.
        # High-contrast pixel noise produces unstable ORB descriptors in motion.
        coarse = random.integers(65, 175, size=(128, 128))
        texture = np.repeat(np.repeat(coarse, 8, axis=0), 8, axis=1).astype(float)
        for _ in range(4):
            texture = (
                texture
                + np.roll(texture, 1, 0)
                + np.roll(texture, -1, 0)
                + np.roll(texture, 1, 1)
                + np.roll(texture, -1, 1)
            ) / 5
        texture = np.clip(texture + random.integers(-24, 25, size=texture.shape), 0, 255).astype(
            np.uint8
        )
        texture = np.repeat(texture[np.newaxis, :, :], 3, axis=0)
        texture[:, :, ::32] = np.array([125, 130, 110], np.uint8)[:, None, None]
        # Low-contrast grain supports nearby views; asymmetric outlines remain
        # recognizable after the camera climbs and that grain becomes subpixel.
        for _ in range(300):
            x, y = random.integers(0, 940, size=2)
            width, height = random.integers(20, 80, size=2)
            color = random.integers(0, 256, size=3, dtype=np.uint8)[:, None, None]
            texture[:, y : y + 3, x : x + width] = color
            texture[:, y + height : y + height + 3, x : x + width] = color
            texture[:, y : y + height, x : x + 3] = color
            texture[:, y : y + height, x + width : x + width + 3] = color
        for name, pixels in (
            ("texture.png", texture),
            (
                "normal.png",
                np.broadcast_to(np.array([128, 128, 255], np.uint8)[:, None, None], (3, 8, 8)),
            ),
        ):
            with rasterio.open(
                destination / name,
                "w",
                driver="PNG",
                width=pixels.shape[2],
                height=pixels.shape[1],
                count=3,
                dtype="uint8",
            ) as output:
                output.write(pixels)
        convergence = math.radians(
            Proj(dataset.crs).get_factors(longitude, latitude).meridian_convergence
        )
        delta_east, delta_north = center_east - east, center_north - north
        offset_east = math.cos(convergence) * delta_east - math.sin(convergence) * delta_north
        offset_north = math.sin(convergence) * delta_east + math.cos(convergence) * delta_north
        with source.open("rb") as stream:
            source_digest = hashlib.file_digest(stream, "sha256").hexdigest()
        metadata = {
            "schema_version": 1,
            "width_m": width_m,
            "height_m": height_m,
            "minimum_elevation_m": low,
            "maximum_elevation_m": high,
            "origin_elevation_m": float(home_height),
            "offset_enu_m": [offset_east, offset_north, low - float(home_height)],
            "grid_yaw_rad": convergence,
            "latitude_deg": latitude,
            "longitude_deg": longitude,
            "source_crs": dataset.crs.to_wkt(),
            "vertical_datum": "CGVD2013",
            "samples": samples,
            "texture_seed": configuration["texture_seed"],
            "texture_scope": "synthetic simulation material, not aerial imagery",
            "texture_algorithm": "multiscale-grain-v3",
            "texture_sha256": hashlib.sha256(
                (destination / "texture.png").read_bytes()
            ).hexdigest(),
            "water_scope": "source flattened water values, not bathymetry",
            "alignment_scope": "local UTM offsets; regional vertical reference retained",
            "source_sha256": source_digest,
            "source_file": source.name,
        }
    (destination / "terrain.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    export_visual_mesh(destination)
    (destination / "location.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": "Halifax Waterfront",
                "latitude_deg": latitude,
                "longitude_deg": longitude,
                "elevation_m": metadata["origin_elevation_m"],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return metadata


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--license", type=Path)
    parser.add_argument(
        "--configuration", type=Path, default=Path("configs/worlds/waterfront.json")
    )
    arguments = parser.parse_args()
    configuration = json.loads(arguments.configuration.read_text(encoding="utf-8"))
    license_path = arguments.license or arguments.source.parent / "halifax-data-license.pdf"
    if not license_path.is_file():
        parser.error("Supply the provincial data license using --license or beside the DEM")
    metadata = prepare_terrain(arguments.source, arguments.destination, configuration)
    shutil.copyfile(license_path, arguments.destination / "data-license.pdf")
    shutil.copyfile(
        Path("configs/worlds/waterfront-source.txt"),
        arguments.destination / "SOURCE.txt",
    )
    print(json.dumps(metadata))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
