"""Verified visual-only pad details; physical geometry and ENU placement stay unchanged."""

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path


def add_pad_details(reference: ET.Element, directory: Path | None = None) -> None:
    """Load bounded authored meshes without requiring Blender in the simulator runtime."""
    directory = directory or Path(__file__).parent / "models/presentation"
    manifest = json.loads((directory / "manifest.json").read_text())
    if (
        manifest.get("schema_version") != 1
        or manifest.get("collision_geometry") is not False
        or manifest.get("coordinate_frame") != "ENU metres; Z up"
    ):
        raise ValueError("Pad details require versioned visual-only ENU assets")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or not 1 <= len(assets) <= 4:
        raise ValueError("Pad details exceed their asset budget")
    link = reference.find("link")
    if link is None:
        raise ValueError("Pad details need the existing reference visual link")
    for asset in assets:
        name = asset["file"]
        path = directory / name
        if (
            Path(name).name != name
            or path.is_symlink()
            or path.suffix != ".obj"
            or path.stat().st_size > 256 * 1024
            or not 1 <= asset["triangles"] <= 5000
        ):
            raise ValueError("Pad detail mesh is outside its bounded package")
        if hashlib.sha256(path.read_bytes()).hexdigest() != asset["sha256"]:
            raise ValueError(f"Pad detail asset hash changed: {name}")
        visual = ET.SubElement(link, "visual", name=f"presentation_{path.stem}")
        ET.SubElement(visual, "cast_shadows").text = "false"
        mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = path.resolve().as_uri()
        material = ET.SubElement(visual, "material")
        for kind in ("ambient", "diffuse"):
            ET.SubElement(material, kind).text = " ".join(map(str, asset["colour_rgba"]))
