"""Opt-in range sensor and physical obstacle assets; never a planner input."""

import copy
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from gazebo.navigation_texture import write_navigation_texture


def _apply_texture(material: ET.Element, path: Path) -> None:
    metal = ET.SubElement(ET.SubElement(material, "pbr"), "metal")
    ET.SubElement(metal, "albedo_map").text = str(path.resolve())
    ET.SubElement(metal, "roughness").text = "0.9"
    ET.SubElement(metal, "metalness").text = "0"


def add_inspection_buildings(world: ET.Element, instances: list[dict]) -> None:
    """Place up to three scaled copies of the retained inspection-building asset.

    Both its collision box and visual markers use the declared scale. These are
    actual simulator geometry, never planner inputs or presentation-only scenery.
    """
    if not isinstance(instances, list) or len(instances) > 3:
        raise ValueError("Navigation scene supports at most three inspection buildings")
    asset = Path(__file__).parent / "models/inspection_building/model.sdf"
    template = ET.parse(asset).find("model")
    if template is None:
        raise ValueError("Inspection building asset has no model")
    for index, instance in enumerate(instances):
        if not isinstance(instance, dict):
            raise ValueError(f"Building {index}: configuration must be an object")
        center, scale = instance.get("center_enu_m"), instance.get("scale_xyz")
        for name, values in (("center_enu_m", center), ("scale_xyz", scale)):
            if (
                not isinstance(values, list)
                or len(values) != 3
                or any(type(v) not in {int, float} or not math.isfinite(v) for v in values)
            ):
                raise ValueError(f"Building {index}: {name} requires three finite values")
        if any(not 0.1 <= value <= 1 for value in scale):
            raise ValueError(f"Building {index}: asset scales must be in 0.1..1")
        if any(abs(center[i]) + 6 * scale[i] > 18 for i in (0, 1)) or center[2] < 6 * scale[2]:
            raise ValueError(f"Building {index}: geometry exceeds the world bounds or ground")
        model = copy.deepcopy(template)
        model.set("name", f"inspection_building_{index}")
        for size in model.findall(".//geometry/box/size"):
            size.text = " ".join(str(float(v) * scale[i]) for i, v in enumerate(size.text.split()))
        for pose in model.findall(".//pose"):
            values = [float(v) for v in pose.text.split()]
            pose.text = " ".join(str(v * scale[i] if i < 3 else v) for i, v in enumerate(values))
        model.find("pose").text = " ".join(map(str, center)) + " 0 0 0"
        world.append(model)


def add_visual_odometry_ground(world: ET.Element, seed: int, directory: Path) -> None:
    """Add unique visual-only ground tiles for downward stereo feature support."""
    if type(seed) is not int or not 0 <= seed < 2**31 - 16:
        raise ValueError("Visual odometry ground seed is outside the supported range")
    model = ET.SubElement(world, "model", name="visual_odometry_ground")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="ground_texture")
    for row in range(4):
        for column in range(4):
            index = row * 4 + column
            path = write_navigation_texture(
                directory / f"vo-ground-{seed + index}.png", seed + index
            )
            visual = ET.SubElement(link, "visual", name=f"tile_{index}")
            ET.SubElement(visual, "pose").text = f"{6 * column - 9} {6 * row - 9} 0.005 0 0 0"
            box = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
            ET.SubElement(box, "size").text = "6 6 0.002"
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "ambient").text = "1 1 1 1"
            ET.SubElement(material, "diffuse").text = "1 1 1 1"
            _apply_texture(material, path)


def add_navigation_sensor(model: ET.Element) -> None:
    """Mount a 720-ray horizontal sensor above the rotors, without collision geometry."""
    link = ET.SubElement(model, "link", name="navigation_scan")
    ET.SubElement(link, "pose").text = "0 0 0.35 0 0 0"
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").text = "0.001"
    inertia = ET.SubElement(inertial, "inertia")
    for key in ("ixx", "iyy", "izz"):
        ET.SubElement(inertia, key).text = "0.000001"
    joint = ET.SubElement(model, "joint", name="navigation_scan_joint", type="fixed")
    ET.SubElement(joint, "parent").text = "base_link"
    ET.SubElement(joint, "child").text = "navigation_scan"
    sensor = ET.SubElement(link, "sensor", name="navigation_lidar", type="gpu_lidar")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "10"
    lidar = ET.SubElement(sensor, "lidar")
    horizontal = ET.SubElement(ET.SubElement(lidar, "scan"), "horizontal")
    for key, value in (
        ("samples", "720"),
        ("resolution", "1"),
        ("min_angle", "-3.141592653589793"),
        ("max_angle", "3.141592653589793"),
    ):
        ET.SubElement(horizontal, key).text = value
    limits = ET.SubElement(lidar, "range")
    for key, value in (("min", "0.8"), ("max", "20"), ("resolution", "0.01")):
        ET.SubElement(limits, key).text = value


def add_navigation_obstacles(world: ET.Element, path: Path) -> None:
    """Load declarative static geometry exclusively in the simulator/evaluation boundary."""
    scenario = json.loads(path.read_text())
    add_inspection_buildings(world, scenario.get("inspection_buildings", []))
    geographic = scenario.get("geographic_buildings") is True and any(
        model.get("name", "").startswith("halifax_")
        and model.find("link/collision/geometry/mesh") is not None
        for model in world.findall("model")
    )
    minimum = 0 if geographic else 1
    if scenario.get("schema_version") != 1 or not minimum <= len(scenario["obstacles"]) <= 20:
        raise ValueError("Navigation requires prepared geographic buildings or1..20 obstacles")
    if "ground_texture_seed" in scenario:
        add_visual_odometry_ground(
            world,
            scenario["ground_texture_seed"],
            Path(os.environ.get("OUTPUT_DIR", "outputs")) / "worlds",
        )
    texture = None
    if "texture_seed" in scenario:
        seed = scenario["texture_seed"]
        texture = write_navigation_texture(
            Path(os.environ.get("OUTPUT_DIR", "outputs"))
            / "worlds"
            / f"navigation-texture-{seed}.png",
            seed,
        )
        scale = scenario.get("lighting_scale", 1.0)
        if type(scale) not in {int, float} or not 0.2 <= scale <= 1.2:
            raise ValueError("Navigation lighting scale must be in 0.2..1.2")
        for color in [world.find("scene/ambient"), *world.findall("light/diffuse")]:
            if color is not None and color.text:
                values = [float(v) for v in color.text.split()]
                color.text = " ".join(str(v * scale if i < 3 else v) for i, v in enumerate(values))
    # Gazebo 8.15 defaults to NED earth axes even in an ENU world. The pinned
    # PX4 bridge additionally expects a legacy left-handed wire convention.
    # ENU body measurements are translated at gazebo.magnetometer, without truth.
    plugin = world.find("plugin[@name='gz::sim::systems::Magnetometer']")
    if plugin is None:
        raise ValueError("Navigation requires the magnetometer world system")
    ET.SubElement(plugin, "use_earth_frame_ned").text = "false"
    ET.SubElement(plugin, "use_units_gauss").text = "true"
    if world.find("plugin[@name='gz::sim::systems::Contact']") is None:
        ET.SubElement(
            world, "plugin", filename="gz-sim-contact-system", name="gz::sim::systems::Contact"
        )
    for index, obstacle in enumerate(scenario["obstacles"]):
        for key in ("center_enu_m", "size_m"):
            values = obstacle.get(key)
            if (
                not isinstance(values, list)
                or len(values) != 3
                or any(type(v) not in {int, float} or not math.isfinite(v) for v in values)
            ):
                raise ValueError(f"Obstacle {index}: {key} requires three finite metres")
        if any(not 0.2 <= value <= 40 for value in obstacle["size_m"]):
            raise ValueError(f"Obstacle {index}: sizes must be in 0.2..40m")
        model = ET.SubElement(world, "model", name=f"avoidance_obstacle_{index}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = " ".join(map(str, obstacle["center_enu_m"])) + " 0 0 0"
        link = ET.SubElement(model, "link", name="body")
        for kind in ("visual", "collision"):
            element = ET.SubElement(link, kind, name=kind)
            box = ET.SubElement(ET.SubElement(element, "geometry"), "box")
            ET.SubElement(box, "size").text = " ".join(map(str, obstacle["size_m"]))
            if kind == "visual":
                material = ET.SubElement(element, "material")
                for key in ("ambient", "diffuse"):
                    ET.SubElement(material, key).text = "1 1 1 1" if texture else "0.85 0.35 0.12 1"
                if texture:
                    _apply_texture(material, texture)
        sensor = ET.SubElement(link, "sensor", name="contacts", type="contact")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "50"
        contact = ET.SubElement(sensor, "contact")
        ET.SubElement(contact, "collision").text = "collision"
        ET.SubElement(contact, "topic").text = f"/avoidance/contacts/{index}"
