"""Explicit vehicle identity and endpoint allocation for the local simulation fleet."""

import json
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class VehicleDefinition:
    vehicle_id: str
    label: str
    instance: int
    system_id: int
    vehicle_port: int
    application_port: int
    spawn_enu_m: tuple[float, float, float]
    enabled: bool = True

    @property
    def model_name(self) -> str:
        return f"x500_{self.instance}"

    @property
    def runtime_directory(self) -> Path:
        return (
            Path("/runtime")
            if self.vehicle_id == "vehicle_1"
            else Path("/runtime") / self.vehicle_id
        )

    def output_directory(self, root: Path) -> Path:
        return root if self.vehicle_id == "vehicle_1" else root / "vehicles" / self.vehicle_id


def load_fleet(path: Path | None = None) -> tuple[VehicleDefinition, ...]:
    if path is None:
        runtime_path = Path(os.environ.get("UAV_FLEET_PATH", "/workspace/outputs/fleet.json"))
        path = runtime_path if runtime_path.exists() else Path("configs/vehicles/fleet.json")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload["schema_version"] != 1 or not payload["vehicles"]:
        raise ValueError("Fleet requires schema version 1 and at least one vehicle")
    vehicles = tuple(
        VehicleDefinition(**{**item, "spawn_enu_m": tuple(item["spawn_enu_m"])})
        for item in payload["vehicles"]
    )
    for vehicle in vehicles:
        if type(vehicle.enabled) is not bool:
            raise ValueError("Vehicle enabled must be true or false")
        if not re.fullmatch(r"[a-z][a-z0-9_]*", vehicle.vehicle_id):
            raise ValueError("Vehicle IDs must be safe lowercase identifiers")
        if (
            type(vehicle.system_id) is not int
            or type(vehicle.instance) is not int
            or not 1 <= vehicle.system_id <= 239
            or not 0 <= vehicle.instance <= 8
            or vehicle.system_id != vehicle.instance + 1
        ):
            raise ValueError("Vehicle system ID or PX4 instance is outside the supported range")
        if (
            not isinstance(vehicle.label, str)
            or not vehicle.label.strip()
            or len(vehicle.spawn_enu_m) != 3
            or any(
                type(value) not in {int, float} or not math.isfinite(value)
                for value in vehicle.spawn_enu_m
            )
        ):
            raise ValueError("Vehicles need a display name and three finite ENU spawn coordinates")
    if not any(vehicle.enabled for vehicle in vehicles):
        raise ValueError("Fleet requires at least one enabled vehicle")
    active_labels = [vehicle.label.casefold() for vehicle in vehicles if vehicle.enabled]
    if len(active_labels) != len(set(active_labels)):
        raise ValueError("Active vehicle display names must be unique")
    for attribute in ("vehicle_id", "instance", "system_id"):
        scope = (
            vehicles
            if attribute == "vehicle_id"
            else tuple(vehicle for vehicle in vehicles if vehicle.enabled)
        )
        if len({getattr(vehicle, attribute) for vehicle in scope}) != len(scope):
            raise ValueError(f"Duplicate fleet {attribute}")
    ports = [
        port
        for vehicle in vehicles
        if vehicle.enabled
        for port in (vehicle.vehicle_port, vehicle.application_port)
    ]
    if len(set(ports)) != len(ports) or any(
        type(port) is not int or not 1024 <= port <= 65535 for port in ports
    ):
        raise ValueError("Fleet MAVLink ports must be unique and in 1024..65535")
    return vehicles
