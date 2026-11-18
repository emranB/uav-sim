"""Visual launch references for observation in the optional flat navigation world."""

import copy
import xml.etree.ElementTree as ET

from common.configuration.fleet import VehicleDefinition


def add_navigation_references(world: ET.Element, vehicles: tuple[VehicleDefinition, ...]) -> None:
    """Copy the existing visual-only fiducials to each declared flat-ground launch site."""
    reference = world.find("model[@name='flight_reference']")
    if reference is None or reference.findall(".//collision"):
        raise ValueError("Navigation launch references require the visual-only flight reference")
    for vehicle in vehicles:
        east, north, _ = vehicle.spawn_enu_m
        if not vehicle.enabled or (east == 0 and north == 0):
            continue
        model = copy.deepcopy(reference)
        model.set("name", f"navigation_reference_{vehicle.vehicle_id}")
        pose = model.find("pose")
        if pose is None:
            pose = ET.SubElement(model, "pose")
        pose.text = f"{east} {north} 0 0 0 0"
        world.append(model)


def add_launch_platforms(world: ET.Element, vehicles: tuple[VehicleDefinition, ...]) -> None:
    """Add declared simulation fixtures with a level0m top beneath the visual pads.

    These authored launch platforms are not municipal building or terrain data.
    Their top matches the flat launch-reference convention; the surrounding measured
    terrain remains unchanged. The fixture height is explicit in its geometry.
    """
    for vehicle in vehicles:
        if not vehicle.enabled:
            continue
        east, north, _ = vehicle.spawn_enu_m
        model = ET.SubElement(world, "model", name=f"demo_launch_platform_{vehicle.vehicle_id}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{east} {north} -0.15 0 0 0"
        link = ET.SubElement(model, "link", name="platform")
        for kind in ("collision", "visual"):
            element = ET.SubElement(link, kind, name=kind)
            cylinder = ET.SubElement(ET.SubElement(element, "geometry"), "cylinder")
            ET.SubElement(cylinder, "radius").text = "1.65"
            ET.SubElement(cylinder, "length").text = "0.3"
            if kind == "visual":
                material = ET.SubElement(element, "material")
                for key in ("ambient", "diffuse"):
                    ET.SubElement(material, key).text = "0.13 0.20 0.24 1"
