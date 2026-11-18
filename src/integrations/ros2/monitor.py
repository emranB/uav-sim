"""Read-only ROS telemetry monitor with topic discovery and compatible sensor QoS."""

import json
import os
import time
from pathlib import Path

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from common.configuration.fleet import load_fleet


class TelemetryMonitor(Node):
    def __init__(self) -> None:
        time_source = os.environ.get("UAV_DDS_TIME_SOURCE", "simulation")
        if time_source not in {"simulation", "agent"}:
            raise ValueError("DDS time source must be simulation or agent")
        super().__init__(
            "simulation_telemetry_monitor",
            parameter_overrides=[
                Parameter("use_sim_time", value=time_source == "simulation"),
            ],
        )
        self.timestamp_clock = (
            "px4-simulation" if time_source == "simulation" else "agent-synchronized"
        )
        self.subscriptions_by_topic = {}
        # Discovery/recovery must continue while the simulation clock is paused or reset.
        self.discovery_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.discover_timer = self.create_timer(1.0, self.discover, clock=self.discovery_clock)

    def discover(self) -> None:
        active = {vehicle.vehicle_id for vehicle in load_fleet() if vehicle.enabled}
        for topic in list(self.subscriptions_by_topic):
            if topic.split("/")[1] not in active:
                self.destroy_subscription(self.subscriptions_by_topic.pop(topic))
        for topic, types in self.get_topic_names_and_types():
            if (
                topic.split("/")[1] in active
                and "/fmu/out/vehicle_local_position" in topic
                and "px4_msgs/msg/VehicleLocalPosition" in types
                and topic not in self.subscriptions_by_topic
            ):
                self.subscriptions_by_topic[topic] = self.create_subscription(
                    VehicleLocalPosition,
                    topic,
                    lambda message, vehicle_id=topic.split("/")[1]: self.received(
                        message, vehicle_id
                    ),
                    qos_profile_sensor_data,
                )

    def received(self, message: VehicleLocalPosition, vehicle_id: str) -> None:
        payload = {
            "vehicle_id": vehicle_id,
            "source": "ros2-dds",
            "received_wall_time": time.time(),
            "timestamp_us": message.timestamp,
            "timestamp_clock": self.timestamp_clock,
            "position_ned_m": [message.x, message.y, message.z],
            "xy_valid": message.xy_valid,
            "z_valid": message.z_valid,
        }
        directory = Path("/runtime") if vehicle_id == "vehicle_1" else Path("/runtime") / vehicle_id
        directory.mkdir(parents=True, exist_ok=True)
        temporary = directory / "ros2.json.tmp"
        temporary.write_text(json.dumps(payload), encoding="utf-8")
        temporary.replace(directory / "ros2.json")


def main() -> None:
    rclpy.init()
    monitor = TelemetryMonitor()
    try:
        rclpy.spin(monitor)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        monitor.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
