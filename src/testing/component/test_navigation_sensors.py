"""Measured sensor frame/freshness and physical fixture contracts, no flight required."""

import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.magnetometer_pb2 import Magnetometer

from gazebo.magnetometer import MagnetometerCompatibility
from gazebo.navigation_scene import add_navigation_obstacles, add_navigation_sensor
from gazebo.ranges import GazeboRanges


class NavigationSensorTests(unittest.TestCase):
    def test_ranges_ignore_world_pose_and_reject_stale_inputs(self) -> None:
        with patch("gazebo.ranges.Node", return_value=Mock()):
            sensor = GazeboRanges("x500_0")
            message = LaserScan(vertical_count=1, range_max=20, angle_min=-3.14, angle_step=0.01)
            message.ranges.extend([5.0] * 720)
            message.header.stamp.sec = 7
            message.world_pose.position.x = 9000
            sensor.receive(message)
            self.assertEqual(sensor.latest().ranges_m, (5.0,) * 720)
            with patch("gazebo.ranges.time.monotonic", return_value=time.monotonic() + 5):
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    sensor.latest()
            sensor.set_enabled(False)
            with self.assertRaisesRegex(RuntimeError, "disabled"):
                sensor.latest()
            sensor.set_enabled(True)
            with self.assertRaisesRegex(RuntimeError, "unavailable"):
                sensor.latest()
            sensor.close()

    def test_magnetometer_wire_conversion_yields_frd_in_pinned_bridge(self) -> None:
        node = Mock()
        sensor = MagnetometerCompatibility(node, "x500_0")
        message = Magnetometer()
        message.field_tesla.x, message.field_tesla.y, message.field_tesla.z = 1, 2, 3
        sensor.receive(message)
        converted = sensor.publisher.publish.call_args.args[0]
        bridge_frd = (-converted.field_tesla.y, -converted.field_tesla.x, converted.field_tesla.z)
        self.assertEqual(bridge_frd, (1, -2, -3))
        self.assertEqual(message.field_tesla.x, 1)

    def test_scene_and_contact_topic_are_independent_of_planner(self) -> None:
        world = ET.Element("world")
        ET.SubElement(world, "plugin", name="gz::sim::systems::Magnetometer")
        add_navigation_obstacles(world, Path("configs/navigation/single.json"))
        self.assertEqual(world.findtext("model/link/sensor/contact/topic"), "/avoidance/contacts/0")
        model = ET.Element("model")
        add_navigation_sensor(model)
        self.assertEqual(model.findtext("link/sensor/lidar/scan/horizontal/samples"), "720")
        self.assertIsNone(model.find("link/collision"))

    def test_fusion_uses_bounded_measured_scan_near_camera_time(self) -> None:
        with patch("gazebo.ranges.Node", return_value=Mock()):
            sensor = GazeboRanges("x500_0")
            for index in range(20):
                message = LaserScan(
                    vertical_count=1, range_max=30, angle_min=-3.14, angle_step=0.01
                )
                message.ranges.extend([float(index + 1)] * 720)
                message.header.stamp.sec = index // 10
                message.header.stamp.nsec = (index % 10) * 100_000_000
                sensor.receive(message)
            self.assertEqual(sensor.latest().timestamp_seconds, 1.9)
            aligned = sensor.nearest(1.5, 0.01)
            self.assertEqual(aligned.timestamp_seconds, 1.5)
            self.assertEqual(aligned.ranges_m[0], 16.0)
            with self.assertRaisesRegex(RuntimeError, "No fresh range"):
                sensor.nearest(0.1, 0.01)
            sensor.set_enabled(False)
            with self.assertRaisesRegex(RuntimeError, "disabled"):
                sensor.nearest(1.5, 0.1)
            sensor.close()
