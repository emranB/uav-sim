"""Load a prepared terrain asset without GIS dependencies or network access."""

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def add_terrain(world: ET.Element, directory: Path) -> None:
    """Pair the collision height grid and prepared visual mesh in one ENU model frame."""
    metadata = json.loads((directory / "terrain.json").read_text(encoding="utf-8"))
    if metadata.get("schema_version") != 1:
        raise ValueError("Terrain metadata requires schema version 1")
    offset = metadata["offset_enu_m"]
    width, height = metadata["width_m"], metadata["height_m"]
    relief = metadata["maximum_elevation_m"] - metadata["minimum_elevation_m"]
    if len(offset) != 3 or not all(
        math.isfinite(value) for value in [width, height, relief, *offset]
    ):
        raise ValueError("Terrain dimensions and offsets must be finite")
    if min(width, height, relief) <= 0:
        raise ValueError("Terrain dimensions must be positive")
    for name in ("heightmap.png", "texture.png", "normal.png"):
        if not (directory / name).is_file():
            raise ValueError(f"Prepared terrain is missing {name}; run terrain preparation")
    for filename in ("terrain.obj", "terrain-visual.json"):
        if not (directory / filename).is_file():
            raise ValueError(
                f"Prepared terrain is missing {filename}; run the terrain mesh exporter"
            )
    visual_manifest = json.loads((directory / "terrain-visual.json").read_text())
    if visual_manifest.get("schema_version") != 1:
        raise ValueError("Terrain visual mesh requires schema version 1; export terrain mesh")
    mesh_path = directory / "terrain.obj"
    for filename, expected in (
        (directory / "heightmap.png", visual_manifest.get("source_heightmap_sha256")),
        (mesh_path, visual_manifest.get("mesh_sha256")),
    ):
        with filename.open("rb") as stream:
            actual = hashlib.file_digest(stream, "sha256").hexdigest()
        if actual != expected:
            raise ValueError(
                "Terrain visual mesh does not match its prepared source; export terrain mesh"
            )
    if [visual_manifest.get(key) for key in ("width_m", "height_m", "relief_m")] != [
        width,
        height,
        relief,
    ]:
        raise ValueError("Terrain visual mesh scale differs from collision metadata")
    ground = world.find("model[@name='ground_plane']")
    if ground is not None:
        world.remove(ground)
    model = ET.SubElement(world, "model", name="halifax_waterfront_terrain")
    yaw = metadata["grid_yaw_rad"]
    if not math.isfinite(yaw):
        raise ValueError("Terrain grid orientation must be finite")
    # Ogre2 Terra reads descriptor position but ignores the parent model transform.
    # A sampled mesh obeys the same model translation/yaw as the physical height grid.
    ET.SubElement(model, "pose").text = f"{offset[0]} {offset[1]} {offset[2]} 0 0 {yaw}"
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="terrain")
    collision = ET.SubElement(link, "collision", name="terrain_collision")
    heightmap = ET.SubElement(ET.SubElement(collision, "geometry"), "heightmap")
    ET.SubElement(heightmap, "uri").text = (directory / "heightmap.png").resolve().as_uri()
    ET.SubElement(heightmap, "size").text = f"{width} {height} {relief}"
    ET.SubElement(heightmap, "pos").text = "0 0 0"
    visual = ET.SubElement(link, "visual", name="terrain_visual")
    mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
    ET.SubElement(mesh, "uri").text = mesh_path.resolve().as_uri()
    material = ET.SubElement(visual, "material")
    # PBR albedo is multiplied by the base colour; an omitted colour renders black.
    ET.SubElement(material, "ambient").text = "1 1 1 1"
    ET.SubElement(material, "diffuse").text = "1 1 1 1"
    metal = ET.SubElement(ET.SubElement(material, "pbr"), "metal")
    ET.SubElement(metal, "albedo_map").text = (directory / "texture.png").resolve().as_uri()
    ET.SubElement(metal, "normal_map").text = (directory / "normal.png").resolve().as_uri()
    ET.SubElement(metal, "metalness").text = "0"
    ET.SubElement(metal, "roughness").text = "1"
