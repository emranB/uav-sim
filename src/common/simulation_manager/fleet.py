"""Resolve explicit command targets and retain each vehicle's independent result."""

import threading
from typing import TYPE_CHECKING, Protocol

from common.interfaces.capabilities import CapabilityUnavailable

if TYPE_CHECKING:
    from common.vehicle_manager.catalog import VehicleCatalog


class VehicleService(Protocol):
    paused: bool

    def execute(self, operation: str, body: dict) -> dict: ...
    def flight_actions(self) -> dict: ...


class FleetOperations:
    def __init__(
        self, services: dict[str, VehicleService], catalog: "VehicleCatalog | None" = None
    ) -> None:
        self.services = services
        self.catalog = catalog
        self._lock = threading.Lock()
        self._maintenance = threading.Event()

    def enter_maintenance(self) -> None:
        """Close command admission only after any short fleet dispatch has completed."""
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("Wait for the current fleet command before Master Reset")
        try:
            if self._maintenance.is_set():
                raise RuntimeError("Master Reset is already in progress")
            self._maintenance.set()
        finally:
            self._lock.release()

    def leave_maintenance(self) -> None:
        """Reopen command admission after verified completion or explicit failure."""
        self._maintenance.clear()

    def execute(self, operation: str, body: dict) -> dict:
        if self._maintenance.is_set() and operation not in {"stop", "flight/land"}:
            raise RuntimeError("Master Reset owns runtime recovery; Stop/Land remain available")
        if operation.startswith("flight/") and not ({"target_ids", "vehicle_id"} & body.keys()):
            raise ValueError("Flight shortcuts require explicit vehicle targets")
        if (operation == "perception/check" or operation.startswith("navigation/")) and not (
            {"target_ids", "vehicle_id"} & body.keys()
        ):
            raise ValueError("Perception checks require explicit vehicle targets")
        if operation.startswith("viewer/") or operation in {
            "recordings/compare",
            "perception/check",
            "stop",
            "flight/land",
        }:
            # Recovery must reach a peer while reset waits for fresh readiness.
            # Each vehicle still owns its command authority and operation lock.
            return self._execute(operation, body)
        # Serialize short navigation admissions without rejecting independent
        # vehicles whose HTTP requests arrive together. Flight ownership stays
        # with each session's authority lock after this dispatch returns.
        acquired = (
            self._lock.acquire(timeout=1.0)
            if operation.startswith("navigation/")
            else self._lock.acquire(blocking=False)
        )
        if not acquired:
            raise RuntimeError("Another fleet command is still in progress")
        try:
            if self._maintenance.is_set():
                raise RuntimeError("Master Reset owns runtime recovery")
            return self._execute(operation, body)
        finally:
            self._lock.release()

    def _execute(self, operation: str, body: dict) -> dict:
        if operation in {"pause", "resume"}:
            active = [
                identifier
                for identifier, service in self.services.items()
                if getattr(service, "navigation_active", False) is True
            ]
            if active:
                raise RuntimeError(
                    f"Shared-world {operation} conflicts with active navigation: {', '.join(active)}"
                )
        if operation in {"vehicles/add", "vehicles/remove", "vehicles/rename"}:
            if self.catalog is None:
                raise RuntimeError("Runtime vehicle catalog is unavailable")
            return self.catalog.execute(operation, body)
        body = dict(body)
        targets = body.pop("target_ids", [body.pop("vehicle_id", next(iter(self.services)))])
        if (
            not isinstance(targets, list)
            or not targets
            or any(not isinstance(target, str) or target not in self.services for target in targets)
            or len(set(targets)) != len(targets)
        ):
            raise ValueError("Choose one or more unique configured vehicle identifiers")
        if operation.startswith("navigation/") and len(targets) != 1:
            raise ValueError("Navigation requires exactly one explicit vehicle target")
        if operation.startswith("flight/"):
            action = operation.removeprefix("flight/")
            if action not in {"start", "land"}:
                raise ValueError("Choose a supported mission-start or landing shortcut")
            availability = {
                target: self.services[target].flight_actions()[action] for target in targets
            }
            blocked = {
                target: row["reason"] for target, row in availability.items() if not row["enabled"]
            }
            if blocked:
                return {
                    "accepted": False,
                    "preflight_passed": False,
                    "results": {
                        target: {
                            "accepted": False,
                            "error": blocked.get(
                                target,
                                "No command sent: another selected vehicle failed group preflight",
                            ),
                        }
                        for target in targets
                    },
                }
            results = {}
            start_options = {}
            if action == "start" and "stereo_horizontal_reference" in body:
                reference = body["stereo_horizontal_reference"]
                if type(reference) is not bool:
                    raise ValueError("Stereo horizontal reference must be true or false")
                start_options["stereo_horizontal_reference"] = reference
            for target in targets:
                try:
                    results[target] = self.services[target].execute(
                        "flight/start" if action == "start" else "stop", dict(start_options)
                    )
                except (ValueError, RuntimeError, OSError) as error:
                    results[target] = {"accepted": False, "error": str(error)}
            return {
                "accepted": all(row.get("accepted") for row in results.values()),
                "preflight_passed": True,
                "results": results,
            }
        if len(targets) == 1:
            result = self.services[targets[0]].execute(operation, body)
        else:
            if operation.startswith("viewer/"):
                raise ValueError("Camera view requires one focused vehicle")
            results = {}
            for target in targets:
                try:
                    results[target] = self.services[target].execute(operation, body)
                except CapabilityUnavailable as error:
                    results[target] = error.result()
                except (ValueError, RuntimeError, OSError) as error:
                    results[target] = {"accepted": False, "error": str(error)}
            result = {
                "accepted": all(item.get("accepted") for item in results.values()),
                "results": results,
            }
            if not result["accepted"]:
                result["error"] = "; ".join(
                    f"{target}: {item['error']}"
                    for target, item in results.items()
                    if "error" in item
                )
        if result.get("accepted") and operation in {"pause", "resume", "start", "stop", "reset"}:
            # Physics pause belongs to the shared world, rather than one vehicle.
            for service in self.services.values():
                service.paused = operation == "pause"
        return result
