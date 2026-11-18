"""Validate ROS simulation clock reception and fresh PX4 DDS telemetry."""

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main() -> int:
    rclpy.init()
    node = Node("simulation_validation")
    clocks = []
    subscription = node.create_subscription(
        Clock,
        "/clock",
        lambda message: clocks.append(message.clock.sec),
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + 15
    report = {"check": "ros2-dds-and-clock", "passed": False, "timestamp": time.time()}
    try:
        while not clocks and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        state = json.loads(Path("/runtime/ros2.json").read_text())
        report.update({"clock_received": bool(clocks), "position": state})
        report["passed"] = bool(clocks) and time.time() - state["received_wall_time"] < 3
        report["passed"] = report["passed"] and state["xy_valid"] and state["z_valid"]
    except (OSError, ValueError, KeyError) as error:
        report["error"] = str(error)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    destination = Path("/workspace/outputs/reports/ros2-validation.json")
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
