"""Prepare simplified building solids from municipal outlines and measured heights.

This optional GIS entry point runs offline. Both visual and collision users consume
the same triangulated, local ENU mesh. Roof details and facade textures are not inferred.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rasterio
from pyproj import Transformer
from rasterio.features import geometry_mask
from rasterio.windows import from_bounds
from shapely.geometry import Polygon, mapping
from shapely.geometry.polygon import orient
from shapely.ops import transform, triangulate, unary_union


def extrude_polygon(polygon: Polygon, bottom_m: float, top_m: float) -> tuple[list, list]:
    """Triangulate a closed solid, retaining courtyard holes; coordinates are metres."""
    if not polygon.is_valid or not bottom_m < top_m:
        raise ValueError("Building requires a valid polygon and positive height")
    polygon = orient(polygon, sign=1.0)
    vertices, faces = [], []
    for ring in (polygon.exterior, *polygon.interiors):
        coordinates = list(ring.coords)
        for a, b in zip(coordinates, coordinates[1:]):
            offset = len(vertices)
            vertices.extend(
                [
                    (a[0], a[1], bottom_m),
                    (b[0], b[1], bottom_m),
                    (b[0], b[1], top_m),
                    (a[0], a[1], top_m),
                ]
            )
            faces.extend([(offset, offset + 1, offset + 2), (offset, offset + 2, offset + 3)])
    for triangle in triangulate(polygon):
        if not polygon.covers(triangle):
            continue
        coordinates = list(triangle.exterior.coords)[:3]
        offset = len(vertices)
        vertices.extend([(x, y, top_m) for x, y in coordinates])
        vertices.extend([(x, y, bottom_m) for x, y in coordinates])
        faces.extend([(offset, offset + 1, offset + 2), (offset + 5, offset + 4, offset + 3)])
    return vertices, faces


def prepare_buildings(
    polygons_path: Path,
    heights_path: Path,
    dem_path: Path,
    dsm_path: Path,
    terrain_path: Path,
    destination: Path,
) -> dict:
    """Project and merge outlines; resolve absent heights from measured DSM minus DEM."""
    payload = json.loads(polygons_path.read_text())
    height_rows = json.loads(heights_path.read_text())["features"]
    heights = {row["attributes"]["BL_ID"]: row["attributes"].get("HEIGHT_M") for row in height_rows}
    terrain = json.loads(terrain_path.read_text())
    if len(payload["features"]) > 2000:
        raise ValueError("Building selection exceeds the 2000-polygon preparation budget")
    destination.mkdir(parents=True, exist_ok=True)
    groups = {}
    with rasterio.open(dem_path) as dem, rasterio.open(dsm_path) as dsm:
        if dem.crs != dsm.crs or dem.crs.linear_units not in {"metre", "meter"}:
            raise ValueError("Ground and surface rasters require the same metric CRS")
        projection = Transformer.from_crs(4326, dem.crs, always_xy=True)
        origin = projection.transform(terrain["longitude_deg"], terrain["latitude_deg"])
        yaw = terrain["grid_yaw_rad"]

        def local(x, y, z=None):
            dx, dy = np.asarray(x) - origin[0], np.asarray(y) - origin[1]
            return math.cos(yaw) * dx - math.sin(yaw) * dy, math.sin(yaw) * dx + math.cos(yaw) * dy

        for feature in payload["features"]:
            attributes = feature["attributes"]
            geometry = Polygon()
            for ring in feature["geometry"]["rings"]:
                polygon = transform(projection.transform, Polygon(ring))
                if not polygon.is_valid:
                    polygon = polygon.buffer(0)
                if not polygon.is_empty and polygon.area >= 2:
                    # ArcGIS rings include courtyard holes and disconnected parts.
                    geometry = geometry.symmetric_difference(polygon)
            if not geometry.is_empty:
                identifier = attributes.get("BL_ID") or f"polygon-{attributes['OBJECTID']}"
                groups.setdefault(identifier, []).append(geometry)
        retained, excluded = [], []
        for identifier, outlines in sorted(groups.items()):
            geometry = unary_union(outlines)
            parts = [geometry] if geometry.geom_type == "Polygon" else list(geometry.geoms)
            for index, polygon in enumerate(parts):
                coordinates = list(polygon.exterior.coords)
                ground = np.array([v[0] for v in dem.sample(coordinates)])
                ground = ground[np.isfinite(ground) & (ground > -100) & (ground < 1000)]
                if not len(ground):
                    raise ValueError(f"{identifier}: ground elevations unavailable")
                base = float(np.median(ground))
                height = heights.get(identifier)
                source = "Halifax municipal HEIGHT_M"
                if not isinstance(height, (int, float)) or not math.isfinite(height) or height <= 0:
                    window = (
                        from_bounds(*polygon.bounds, transform=dsm.transform)
                        .round_offsets()
                        .round_lengths()
                    )
                    values = dsm.read(1, window=window, masked=True)
                    inside = geometry_mask(
                        [mapping(polygon)], values.shape, dsm.window_transform(window), invert=True
                    )
                    roof = values.data[inside & ~np.ma.getmaskarray(values)]
                    roof = roof[np.isfinite(roof) & (roof > base + 1) & (roof < base + 200)]
                    if not len(roof):
                        excluded.append(
                            {
                                "building_id": identifier,
                                "reason": "No municipal height or valid DSM roof samples",
                            }
                        )
                        continue
                    height = float(np.percentile(roof, 80) - base)
                    source = "2019 DSM roof percentile80 minus median boundary DEM"
                if not 1 <= height <= 200:
                    raise ValueError(f"{identifier}: building height outside 1..200 metres")
                polygon = transform(local, polygon)
                bottom = float(ground.min()) - terrain["origin_elevation_m"] - 1.0
                top = base - terrain["origin_elevation_m"] + height
                vertices, faces = extrude_polygon(polygon, bottom, top)
                name = f"{identifier}-{index}.obj"
                normals = np.zeros((len(vertices), 3), dtype=float)
                for face in faces:
                    a, b, c = (np.asarray(vertices[i]) for i in face)
                    normal = np.cross(b - a, c - a)
                    for i in face:
                        normals[i] += normal
                lengths = np.linalg.norm(normals, axis=1)
                if np.any(lengths < 1e-10):
                    raise ValueError(f"{identifier}: prepared mesh has a degenerate surface normal")
                normals /= lengths[:, None]
                lines = [f"o {identifier}-{index}"]
                lines += ["v " + " ".join(f"{v:.5f}" for v in point) for point in vertices]
                lines += ["vn " + " ".join(f"{v:.8f}" for v in point) for point in normals]
                lines += ["f " + " ".join(f"{i + 1}//{i + 1}" for i in face) for face in faces]
                encoded = ("\n".join(lines) + "\n").encode()
                (destination / name).write_bytes(encoded)
                retained.append(
                    {
                        "building_id": identifier,
                        "file": name,
                        "sha256": hashlib.sha256(encoded).hexdigest(),
                        "height_m": height,
                        "height_source": source,
                        "bottom_enu_m": bottom,
                        "top_enu_m": top,
                        "outline_enu_m": list(polygon.exterior.coords),
                        "triangles": len(faces),
                    }
                )
    result = {
        "schema_version": 1,
        "frame": "local ENU metres; same terrain origin and yaw",
        "scope": "Municipal outlines extruded to measured height; simplified flat roofs",
        "origin": {
            key: terrain[key]
            for key in ("latitude_deg", "longitude_deg", "origin_elevation_m", "grid_yaw_rad")
        },
        "sources_sha256": {
            str(p.name): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (polygons_path, heights_path)
        },
        "buildings": retained,
        "excluded": excluded,
    }
    for path in (dem_path, dsm_path, terrain_path):
        with path.open("rb") as stream:
            result["sources_sha256"][path.name] = hashlib.file_digest(stream, "sha256").hexdigest()
    result["attribution"] = (
        "Contains information licensed under the Open Government Licence—Halifax; terrain and surface data provided by the Department of Service Nova Scotia."
    )
    (destination / "buildings.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("polygons", "heights", "dem", "dsm", "terrain", "destination"):
        parser.add_argument(name, type=Path)
    arguments = parser.parse_args()
    result = prepare_buildings(
        arguments.polygons,
        arguments.heights,
        arguments.dem,
        arguments.dsm,
        arguments.terrain,
        arguments.destination,
    )
    print(json.dumps({"buildings": len(result["buildings"]), "excluded": result["excluded"]}))


if __name__ == "__main__":
    main()
