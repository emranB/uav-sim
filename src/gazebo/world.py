"""Apply a configured geographic reference to the default physical world."""

import json
import random
import xml.etree.ElementTree as ET
from pathlib import Path

from common.configuration.world import WorldLocation
from gazebo.geography import add_buildings
from gazebo.presentation_assets import add_pad_details
from gazebo.terrain import add_terrain


def prepare_world(
    template: Path,
    destination: Path,
    location: WorldLocation,
    scene_name: str = "default",
    stereo_enabled: bool = False,
    terrain_directory: Path | None = None,
    shadows_enabled: bool = True,
    real_time_factor: float = 1.0,
    visual_polish: bool = False,
    buildings_directory: Path | None = None,
) -> Path:
    if scene_name not in {"default", "inspection", "waterfront"}:
        raise ValueError("Scene must be default, inspection or waterfront")
    tree = ET.parse(template)
    coordinates = tree.find("world/spherical_coordinates")
    if coordinates is None:
        raise ValueError("World template has no geographic reference")
    for key, value in (
        ("latitude_deg", location.latitude_deg),
        ("longitude_deg", location.longitude_deg),
        ("elevation", location.elevation_m),
    ):
        coordinates.find(key).text = str(value)
    world = tree.find("world")
    if world is None:
        raise ValueError("World template has no world element")
    physics = world.find("physics")
    if physics is None:
        physics = ET.SubElement(world, "physics", name="simulation", type="ignored")
    factor = physics.find("real_time_factor")
    if factor is None:
        factor = ET.SubElement(physics, "real_time_factor")
    factor.text = str(real_time_factor)
    scene = world.find("scene")
    if scene is not None:
        shadows = scene.find("shadows")
        if shadows is not None:
            shadows.text = str(shadows_enabled).lower()
        for name, color in (("background", "0.07 0.10 0.16 1"), ("ambient", "0.55 0.60 0.65 1")):
            element = scene.find(name)
            if element is not None:
                element.text = color
    if not shadows_enabled:
        for light in world.findall("light"):
            casts = light.find("cast_shadows")
            if casts is not None:
                casts.text = "false"
    material = world.find("model[@name='ground_plane']/link/visual/material")
    if material is not None:
        for name in ("ambient", "diffuse"):
            material.find(name).text = "0.17 0.23 0.29 1"
    reference = ET.parse(Path(__file__).parent / "models/flight_reference/model.sdf").find("model")
    if reference is None:
        raise ValueError("Flight reference asset is missing its model")
    if stereo_enabled:
        add_landing_fiducials(reference)
    if visual_polish:
        add_pad_details(reference)
    world.append(reference)
    if scene_name == "inspection":
        building = ET.parse(Path(__file__).parent / "models/inspection_building/model.sdf").find(
            "model"
        )
        if building is None:
            raise ValueError("Inspection building is missing its model")
        world.append(building)
    if scene_name != "waterfront":
        add_reference_grid(world)
    if scene_name == "waterfront":
        if terrain_directory is None:
            raise ValueError("Waterfront requires an explicit prepared terrain directory")
        add_terrain(world, terrain_directory)
        if buildings_directory is not None:
            add_buildings(
                world,
                buildings_directory,
                json.loads((terrain_directory / "terrain.json").read_text()),
            )
    if stereo_enabled:
        # A world with explicit plugins no longer receives the default plugin set.
        configuration = Path(__file__).resolve().parents[2] / "configs/worlds/server.config"
        present = {plugin.get("name") for plugin in world.findall("plugin")}
        for default_plugin in ET.parse(configuration).findall("plugins/plugin"):
            if default_plugin.get("name") not in present:
                default_plugin.attrib.pop("entity_name", None)
                default_plugin.attrib.pop("entity_type", None)
                world.append(default_plugin)
        plugin = ET.SubElement(
            world, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors"
        )
        ET.SubElement(plugin, "render_engine").text = "ogre2"
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return destination


def add_landing_fiducials(reference: ET.Element) -> None:
    """Add visual-only, varied corners where the plain landing pad fills close views."""
    link = reference.find("link")
    if link is None:
        raise ValueError("Landing reference has no visual link")
    pattern = random.Random(41)
    for row in range(-5, 6):
        for column in range(-5, 6):
            x, y = column * 0.24, row * 0.24
            if x * x + y * y > 1.35**2:
                continue
            if (abs(abs(x) - 0.45) < 0.17 and abs(y) < 0.78) or (abs(y) < 0.17 and abs(x) < 0.6):
                continue
            width = pattern.uniform(0.09, 0.15)
            height = pattern.uniform(0.09, 0.15)
            for layer, scale, shade in (
                (0, 1, "0.85 0.9 0.94 1"),
                (1, 0.65, "0.02 0.04 0.06 1"),
                (2, 0.2, "0.85 0.9 0.94 1"),
            ):
                visual = ET.SubElement(link, "visual", name=f"fiducial_{row}_{column}_{layer}")
                ET.SubElement(visual, "pose").text = f"{x} {y} {0.023 + layer * 0.002} 0 0 0"
                box = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
                ET.SubElement(box, "size").text = f"{width * scale} {height * scale} 0.001"
                material = ET.SubElement(visual, "material")
                ET.SubElement(material, "ambient").text = shade
                ET.SubElement(material, "diffuse").text = shade


def add_reference_grid(world: ET.Element) -> None:
    """Add visual-only two-metre cells; no collision or physics geometry is introduced."""
    model = ET.SubElement(world, "model", name="reference_grid")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="grid")
    for coordinate in range(-50, 51, 2):
        color = "0.30 0.42 0.52 1" if coordinate % 10 == 0 else "0.23 0.31 0.38 1"
        for axis in ("x", "y"):
            visual = ET.SubElement(link, "visual", name=f"{axis}_{coordinate}")
            ET.SubElement(visual, "pose").text = (
                f"0 {coordinate} 0.002 0 0 0" if axis == "x" else f"{coordinate} 0 0.002 0 0 0"
            )
            geometry = ET.SubElement(visual, "geometry")
            box = ET.SubElement(geometry, "box")
            ET.SubElement(box, "size").text = (
                "100 0.025 0.005" if axis == "x" else "0.025 100 0.005"
            )
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color
