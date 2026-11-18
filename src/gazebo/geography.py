"""Load prepared geographic building solids without GIS runtime dependencies."""

import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path


def add_buildings(world: ET.Element, directory: Path, terrain_origin: dict | None = None) -> None:
    """Use each verified ENU solid for both visible and physical geometry."""
    manifest = json.loads((directory / "buildings.json").read_text())
    buildings = manifest.get("buildings", [])
    if terrain_origin is not None and any(
        not math.isclose(manifest["origin"][key], terrain_origin[key], abs_tol=1e-9, rel_tol=0)
        for key in ("latitude_deg", "longitude_deg", "origin_elevation_m", "grid_yaw_rad")
    ):
        raise ValueError("Prepared buildings and terrain use different geographic origins")
    if manifest.get("schema_version") != 1 or not 1 <= len(buildings) <= 500:
        raise ValueError("Geographic buildings require schema1 and 1..500 prepared solids")
    total_bytes = 0
    for building in buildings:
        name = building["file"]
        if not re.fullmatch(r"[A-Za-z0-9_-]+\.obj", name):
            raise ValueError("Building mesh must be a plain OBJ filename")
        path = directory / name
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"Prepared building mesh unavailable: {name}")
        total_bytes += path.stat().st_size
        if total_bytes > 16 * 1024**2:
            raise ValueError("Building meshes exceed the 16MiB runtime budget")
        if hashlib.sha256(path.read_bytes()).hexdigest() != building["sha256"]:
            raise ValueError(f"Building mesh checksum differs: {name}")
        model = ET.SubElement(world, "model", name="halifax_" + path.stem)
        ET.SubElement(model, "static").text = "true"
        link = ET.SubElement(model, "link", name="building")
        for kind in ("collision", "visual"):
            element = ET.SubElement(link, kind, name=kind)
            mesh = ET.SubElement(ET.SubElement(element, "geometry"), "mesh")
            ET.SubElement(mesh, "uri").text = path.resolve().as_uri()
            if kind == "visual":
                material = ET.SubElement(element, "material")
                ET.SubElement(material, "ambient").text = "0.52 0.57 0.60 1"
                ET.SubElement(material, "diffuse").text = "0.52 0.57 0.60 1"
        sensor = ET.SubElement(link, "sensor", name="contacts", type="contact")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "20"
        contact = ET.SubElement(sensor, "contact")
        ET.SubElement(contact, "collision").text = "collision"
        ET.SubElement(contact, "topic").text = "/geography/contacts"
