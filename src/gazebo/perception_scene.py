"""A self-contained, stationary stereo scene for renderer feasibility checks."""

import random
import xml.etree.ElementTree as ET
from pathlib import Path


def _box(
    world: ET.Element,
    name: str,
    pose: str,
    size: str,
    color: str,
    *,
    collision: bool = True,
) -> None:
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = pose
    link = ET.SubElement(model, "link", name="body")
    for kind in ("visual", "collision") if collision else ("visual",):
        shape = ET.SubElement(link, kind, name=kind)
        box = ET.SubElement(ET.SubElement(shape, "geometry"), "box")
        ET.SubElement(box, "size").text = size
        if kind == "visual":
            material = ET.SubElement(shape, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color


def write_stereo_scene(
    destination: Path, detailed: bool = False, rate_hz: int | None = None
) -> Path:
    """Write two RGB sensors and distinct surfaces; no flight or navigation logic."""
    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name="perception_probe")
    for filename, name in (
        ("physics", "Physics"),
        ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("sensors", "Sensors"),
    ):
        plugin = ET.SubElement(
            world,
            "plugin",
            filename=f"gz-sim-{filename}-system",
            name=f"gz::sim::systems::{name}",
        )
        if name == "Sensors":
            ET.SubElement(plugin, "render_engine").text = "ogre2"
    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.7 0.7 0.7 1"
    ET.SubElement(scene, "background").text = "0.1 0.15 0.22 1"
    light = ET.SubElement(world, "light", type="directional", name="sun")
    ET.SubElement(light, "direction").text = "-1 0 -1"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    _box(world, "ground", "6 0 -0.1 0 0 0", "30 30 0.2", "0.22 0.3 0.35 1")
    _box(world, "building", "8 0 2 0 0 0", "2 4 4", "0.65 0.7 0.75 1")
    for row in range(4):
        for column in range(5):
            color = "0.08 0.2 0.3 1" if (row + column) % 2 else "0.9 0.6 0.15 1"
            _box(
                world,
                f"facade_{row}_{column}",
                f"6.98 {-1.5 + column * 0.7} {0.5 + row * 0.9} 0 0 0",
                "0.03 0.35 0.4",
                color,
                collision=False,
            )
    _box(world, "foreground", "4 -1.2 0.6 0 0 0.3", "0.5 0.5 1.2", "0.2 0.7 0.5 1")
    if detailed:
        random_source = random.Random(27)
        wall = world.find("model[@name='building']/link")
        for index in range(500):
            visual = ET.SubElement(wall, "visual", name=f"texture_{index}")
            y_m, z_m = random_source.uniform(-1.95, 1.95), random_source.uniform(-1.95, 1.95)
            ET.SubElement(visual, "pose").text = f"-1.04 {y_m} {z_m} 0 0 0"
            box = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
            size = random_source.uniform(0.025, 0.08)
            ET.SubElement(box, "size").text = f"0.01 {size} {size}"
            shade = random_source.choice(("0.02 0.03 0.04 1", "0.95 0.95 0.95 1"))
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "ambient").text = shade
            ET.SubElement(material, "diffuse").text = shade
    rig = ET.SubElement(world, "model", name="inspection_rig")
    ET.SubElement(rig, "static").text = "true"
    for side, y_m in (("left", 0.08), ("right", -0.08)):
        link = ET.SubElement(rig, "link", name=side)
        ET.SubElement(link, "pose").text = f"0 {y_m} 1.5 0 0 0"
        sensor = ET.SubElement(link, "sensor", name=f"{side}_camera", type="camera")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = str(rate_hz or (10 if detailed else 5))
        ET.SubElement(sensor, "topic").text = f"/perception_probe/{side}"
        camera = ET.SubElement(sensor, "camera")
        ET.SubElement(camera, "horizontal_fov").text = "1.0"
        image = ET.SubElement(camera, "image")
        for key, value in (
            ("width", "640" if detailed else "320"),
            ("height", "480" if detailed else "240"),
            ("format", "R8G8B8"),
        ):
            ET.SubElement(image, key).text = value
        clip = ET.SubElement(camera, "clip")
        ET.SubElement(clip, "near").text = "0.1"
        ET.SubElement(clip, "far").text = "30"
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(sdf, space="  ")
    ET.ElementTree(sdf).write(destination, encoding="utf-8", xml_declaration=True)
    return destination
