from dataclasses import dataclass


@dataclass(frozen=True)
class Vehicle:
    vehicle_id: str
    model: str = "x500"
    mavlink_system_id: int = 1
    ros_namespace: str = "/vehicle_1"


class VehicleRegistry:
    def __init__(self):
        self._vehicles = {}

    def add(self, vehicle: Vehicle):
        if not vehicle.vehicle_id or not 1 <= vehicle.mavlink_system_id <= 255:
            raise ValueError("A nonempty ID and MAVLink system ID in 1..255 are required")
        for current in self._vehicles.values():
            if (
                current.vehicle_id == vehicle.vehicle_id
                or current.mavlink_system_id == vehicle.mavlink_system_id
                or current.ros_namespace == vehicle.ros_namespace
            ):
                raise ValueError(
                    "Vehicle IDs, MAVLink system IDs and ROS namespaces must be unique"
                )
        self._vehicles[vehicle.vehicle_id] = vehicle

    def get(self, vehicle_id: str) -> Vehicle:
        return self._vehicles[vehicle_id]

    def all(self) -> tuple[Vehicle, ...]:
        return tuple(self._vehicles.values())
