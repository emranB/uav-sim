"""Scene generation preserves the default and gives demo assets real geometry."""

import json
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from common.configuration.missions import parse_plan
from common.configuration.world import WorldLocation
from gazebo.perception_scene import write_stereo_scene
from gazebo.world import prepare_world


class PerceptionSceneTests(unittest.TestCase):
    def test_baseline_missions_are_supported_and_clear_the_static_building(self) -> None:
        directory = Path(__file__).resolve().parents[3]
        homes = []
        for vehicle_id in ("vehicle_1", "vehicle_2"):
            path = directory / "configs/missions" / f"inspection-{vehicle_id}.plan"
            mission = json.loads(path.read_text(encoding="utf-8"))
            items = parse_plan(mission)
            self.assertEqual(items[0].command, 22)
            self.assertEqual(items[-1].command, 21)
            self.assertEqual(items[0].parameters[4:6], items[-1].parameters[4:6])
            homes.append(items[0].parameters[4:6])
            positions = []
            for item in items:
                north_m = math.radians(item.parameters[4] - 44.6488) * 6378137
                east_m = (
                    math.radians(item.parameters[5] + 63.5752)
                    * 6378137
                    * math.cos(math.radians(44.6488))
                )
                positions.append((east_m, north_m))
            for start, end in zip(positions, positions[1:]):
                for sample in range(101):
                    x = start[0] + (end[0] - start[0]) * sample / 100
                    y = start[1] + (end[1] - start[1]) * sample / 100
                    self.assertFalse(-7 <= x <= 7 and 23 <= y <= 37)
        self.assertNotEqual(homes[0], homes[1])

    def test_stereo_scene_has_distinct_origins_and_matching_capture_settings(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            world = ET.parse(write_stereo_scene(Path(directory) / "scene.sdf"))
        sensors = world.findall("world/model/link/sensor")
        self.assertEqual(len(sensors), 2)
        self.assertNotEqual(sensors[0].findtext("topic"), sensors[1].findtext("topic"))
        self.assertEqual(sensors[0].findtext("update_rate"), sensors[1].findtext("update_rate"))
        poses = world.findall("world/model[@name='inspection_rig']/link/pose")
        self.assertNotEqual(poses[0].text, poses[1].text)
        self.assertIsNotNone(world.find("world/model[@name='building']/link/collision"))

    def test_inspection_collision_geometry_is_opt_in(self) -> None:
        template = """<sdf><world name="default"><spherical_coordinates>
        <latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation>
        </spherical_coordinates></world></sdf>"""
        location = WorldLocation("Test", 44, -63, 20)
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "template.sdf"
            source.write_text(template, encoding="utf-8")
            target = Path(directory) / "world.sdf"
            prepare_world(source, target, location)
            self.assertIsNone(ET.parse(target).find("world/model[@name='inspection_building']"))
            prepare_world(source, target, location, "inspection")
            self.assertIsNotNone(
                ET.parse(target).find("world/model[@name='inspection_building']/link/collision")
            )
            with self.assertRaises(ValueError):
                prepare_world(source, target, location, "unknown")
            prepare_world(source, target, location, "inspection", stereo_enabled=True)
            plugins = {item.attrib["name"] for item in ET.parse(target).findall("world/plugin")}
            self.assertTrue(
                {
                    "gz::sim::systems::Physics",
                    "gz::sim::systems::UserCommands",
                    "gz::sim::systems::Imu",
                    "gz::sim::systems::NavSat",
                    "gz::sim::systems::Sensors",
                }.issubset(plugins)
            )
