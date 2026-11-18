"""Explicit Gazebo 8.15 to pinned PX4 1.17 magnetometer compatibility boundary."""

from gz.msgs10.magnetometer_pb2 import Magnetometer
from gz.transport13 import Node


class MagnetometerCompatibility:
    """Translate measured FLU gauss into the legacy bridge's expected wire convention.

    The world plugin is configured for ENU earth axes and gauss. PX4 1.17's
    GZBridge applies (-wire_y, -wire_x, wire_z), so publish (FLU_y, -FLU_x, -FLU_z)
    to obtain the intended FRD body measurement. No truth pose or yaw is consulted.
    """

    def __init__(self, node: Node, model_name: str) -> None:
        self.node = node
        prefix = f"/world/default/model/{model_name}/link/base_link/sensor/"
        self.topic = prefix + "navigation_magnetometer_raw/magnetometer"
        self.publisher = node.advertise(prefix + "magnetometer_sensor/magnetometer", Magnetometer)
        if not node.subscribe(Magnetometer, self.topic, self.receive):
            raise RuntimeError("Cannot subscribe to measured navigation magnetometer")

    def receive(self, message: Magnetometer) -> None:
        converted = Magnetometer()
        converted.CopyFrom(message)
        converted.field_tesla.x = message.field_tesla.y
        converted.field_tesla.y = -message.field_tesla.x
        converted.field_tesla.z = -message.field_tesla.z
        self.publisher.publish(converted)

    def close(self) -> None:
        self.node.unsubscribe(self.topic)
