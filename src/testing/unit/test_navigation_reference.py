"""Launch texture is explicit visual geometry, independent of obstacle/planner inputs."""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from common.configuration.fleet import load_fleet
from gazebo.navigation_reference import add_navigation_references


class NavigationReferenceTests(unittest.TestCase):
    def test_each_offset_launch_gets_texture_without_collision_geometry(self) -> None:
        world = ET.Element("world")
        reference = ET.SubElement(world, "model", name="flight_reference")
        ET.SubElement(reference, "static").text = "true"
        ET.SubElement(ET.SubElement(reference, "link", name="guides"), "visual", name="pattern")
        fleet = load_fleet(Path(__file__).resolve().parents[3] / "configs/vehicles/fleet.json")
        add_navigation_references(world, fleet)
        second = world.find("model[@name='navigation_reference_vehicle_2']")
        self.assertEqual(second.findtext("pose"), "6 0 0 0 0 0")
        self.assertIsNotNone(second.find("link/visual[@name='pattern']"))
        self.assertIsNone(second.find(".//collision"))
        self.assertIsNone(world.find("model[@name='navigation_reference_vehicle_1']"))

    def test_missing_reference_fails_explicitly(self) -> None:
        with self.assertRaisesRegex(ValueError, "visual-only"):
            add_navigation_references(ET.Element("world"), ())
