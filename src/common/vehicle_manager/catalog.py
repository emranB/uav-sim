"""Persist vehicle definitions and coordinate explicit runtime registration and retirement."""

import json
import os
import shutil
import time
from collections.abc import Callable
from dataclasses import asdict, replace
from pathlib import Path
from typing import Protocol

from common.configuration.fleet import VehicleDefinition, load_fleet
from common.interfaces.vehicles import VehicleAdapter


class CatalogVehicle(Protocol):
    vehicle: VehicleAdapter


class VehicleCatalog:
    def __init__(
        self,
        directory: Path,
        services: dict[str, CatalogVehicle],
        create: Callable[[VehicleDefinition], CatalogVehicle],
        retire: Callable[[VehicleDefinition, CatalogVehicle], None],
        rename: Callable[[str, str], None],
        discard: Callable[[VehicleDefinition, CatalogVehicle], None] | None = None,
    ) -> None:
        self.path = Path(os.environ.get("UAV_FLEET_PATH", str(directory / "fleet.json")))
        seed = self.path if self.path.exists() else Path("configs/vehicles/fleet.json")
        self._definitions = list(load_fleet(seed))
        self._services = services
        self._create = create
        self._retire = retire
        self._rename = rename
        self._discard = discard or (lambda definition, service: service.vehicle.close())

    def execute(self, operation: str, body: dict) -> dict:
        if operation == "vehicles/add":
            return self._add(body)
        identifier = body.get("vehicle_id")
        if identifier not in self._services:
            raise ValueError("Choose an active vehicle")
        index = next(
            index
            for index, vehicle in enumerate(self._definitions)
            if vehicle.vehicle_id == identifier
        )
        definition = self._definitions[index]
        if operation == "vehicles/rename":
            label = self._validate_label(body.get("label"))
            self._validate_unique_label(label, identifier)
            candidate = list(self._definitions)
            candidate[index] = replace(definition, label=label)
            self._persist(candidate)
            self._rename(identifier, label)
        elif operation == "vehicles/remove":
            if len(self._services) == 1:
                raise ValueError("Keep at least one active vehicle")
            service = self._services[identifier]
            if service.vehicle.snapshot()["armed"]:
                raise RuntimeError("Land and disarm the selected vehicle before removing it")
            candidate = list(self._definitions)
            candidate[index] = replace(definition, enabled=False)
            self._persist(candidate)
            # Retirement must acknowledge process shutdown and model removal before
            # the entry disappears. A failure retains the entry for a cleanup retry.
            self._retire(definition, service)
            del self._services[identifier]
        else:
            raise ValueError("Unknown vehicle catalog operation")
        return {"accepted": True, "vehicle_id": identifier, "operation": operation}

    @staticmethod
    def _validate_label(label: object) -> str:
        if (
            not isinstance(label, str)
            or not 1 <= len(label.strip()) <= 40
            or not label.isprintable()
            or "<" in label
            or ">" in label
        ):
            raise ValueError("Vehicle name must contain 1..40 characters")
        return label.strip()

    def _validate_unique_label(self, label: str, excluding: str | None = None) -> None:
        if any(
            vehicle.enabled
            and vehicle.vehicle_id != excluding
            and vehicle.label.casefold() == label.casefold()
            for vehicle in self._definitions
        ):
            raise ValueError("Choose a distinct display name for each active vehicle")

    def _add(self, body: dict) -> dict:
        label = self._validate_label(body.get("label"))
        self._validate_unique_label(label)
        if body.get("model", "x500") != "x500":
            raise ValueError("Only the validated X500 quadcopter is available")
        occupied = {vehicle.instance for vehicle in self._definitions if vehicle.enabled}
        instance = next((candidate for candidate in range(9) if candidate not in occupied), None)
        if instance is None:
            raise ValueError("This deployment supports up to nine active PX4 vehicles")
        sequence = 1
        identifiers = {vehicle.vehicle_id for vehicle in self._definitions}
        while f"vehicle_{sequence}" in identifiers:
            sequence += 1
        definition = VehicleDefinition(
            f"vehicle_{sequence}",
            label,
            instance,
            instance + 1,
            14540 + instance,
            14640 + instance,
            (6.0 * instance, 0.0, 0.1),
        )
        service = self._create(definition)
        try:
            self._persist([*self._definitions, definition])
        except (ValueError, RuntimeError, OSError):
            self._discard(definition, service)
            raise
        self._services[definition.vehicle_id] = service
        return {
            "accepted": True,
            "vehicle_id": definition.vehicle_id,
            "operation": "vehicles/add",
            "state": "starting",
        }

    def _persist(self, definitions: list[VehicleDefinition]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        try:
            temporary.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "vehicles": [asdict(vehicle) for vehicle in definitions],
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
            load_fleet(temporary)
            if self.path.exists():
                history = self.path.parent / "configuration-history"
                history.mkdir(exist_ok=True)
                shutil.copyfile(self.path, history / f"fleet-{time.time_ns()}.json")
            temporary.replace(self.path)
        finally:
            temporary.unlink(missing_ok=True)
        self._definitions = definitions
