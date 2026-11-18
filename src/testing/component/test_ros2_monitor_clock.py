"""Actual Jazzy discovery timer progresses while the simulation clock has no updates."""

import importlib
import time
import unittest
from unittest.mock import patch

try:
    rclpy = importlib.import_module("rclpy")
except ModuleNotFoundError as error:
    rclpy = None
    ROS_UNAVAILABLE = str(error)
else:
    ROS_UNAVAILABLE = ""
    from rclpy.executors import SingleThreadedExecutor

    from integrations.ros2.monitor import TelemetryMonitor


@unittest.skipIf(rclpy is None, "Actual Jazzy environment unavailable: " + ROS_UNAVAILABLE)
class RosMonitorClockTests(unittest.TestCase):
    def test_discovery_continues_with_paused_simulation_time(self) -> None:
        rclpy.init(domain_id=91)
        try:
            with (
                patch.dict("os.environ", {"UAV_DDS_TIME_SOURCE": "simulation"}),
                patch("integrations.ros2.monitor.load_fleet", return_value=()) as discover,
            ):
                monitor = TelemetryMonitor()
                executor = SingleThreadedExecutor()
                executor.add_node(monitor)
                try:
                    self.assertTrue(monitor.get_parameter("use_sim_time").value)
                    deadline = time.monotonic() + 1.5
                    while discover.call_count == 0 and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=0.05)
                    self.assertGreater(discover.call_count, 0)
                    self.assertEqual(monitor.get_clock().now().nanoseconds, 0)
                finally:
                    executor.remove_node(monitor)
                    executor.shutdown()
                    monitor.destroy_node()
        finally:
            rclpy.try_shutdown()
