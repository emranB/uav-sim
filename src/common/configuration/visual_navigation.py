"""Dedicated loopback endpoints for independent SITL visual localization sessions."""

from dataclasses import dataclass

from common.configuration.fleet import VehicleDefinition


@dataclass(frozen=True)
class VisualEndpoints:
    """One controller's sensor input, publisher, health and fault channels."""

    controller_port: int
    publisher_port: int
    status_port: int
    vision_fault_port: int
    gps_fault_port: int

    @classmethod
    def for_vehicle(cls, vehicle: VehicleDefinition) -> "VisualEndpoints":
        """Allocate by validated PX4 instance, preserving the original Atlas fixture."""
        if not 0 <= vehicle.instance <= 8 or vehicle.system_id != vehicle.instance + 1:
            raise ValueError(f"{vehicle.vehicle_id}: invalid visual controller identity")
        base = 14610 if vehicle.instance == 0 else 17000 + 10 * vehicle.instance
        return cls(*(base + offset for offset in range(5)))


def validate_visual_endpoints(vehicles: tuple[VehicleDefinition, ...]) -> None:
    """Reject visual channels that collide with any configured application link."""
    occupied = {
        port
        for vehicle in vehicles
        for port in (
            vehicle.vehicle_port,
            vehicle.application_port,
        )
    }
    for vehicle in vehicles:
        if not vehicle.enabled:
            continue
        ports = set(vars(VisualEndpoints.for_vehicle(vehicle)).values())
        if occupied & ports:
            raise ValueError(f"{vehicle.vehicle_id}: visual endpoint conflicts with fleet links")
        occupied.update(ports)
