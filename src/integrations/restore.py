"""Concrete safe UAV restore through application and declared process owners."""

import json
import os
import time
from pathlib import Path
from typing import TYPE_CHECKING

from common.configuration.preferences import DEFAULT_PREFERENCES, DesktopPreferences
from common.lifecycle.readiness import wait_for_reset_readiness
from common.lifecycle.windows import current_service_window
from common.logging.live_map import read_live_map
from integrations.runtime_services import RuntimeServiceOwners

if TYPE_CHECKING:
    from common.simulation_manager.fleet import FleetOperations
    from common.simulation_manager.server import SimulationService


class WorkspaceRestoreRuntime:
    """Keep the healthy control API owner alive while restarting declared UAV children."""

    def __init__(
        self,
        services: dict[str, "SimulationService"],
        fleet: "FleetOperations",
        output_directory: Path,
    ) -> None:
        self.services, self.fleet, self.output_directory = services, fleet, output_directory
        roles = tuple(filter(None, os.environ.get("UAV_RESTORE_SERVICES", "").split(",")))
        self.owners = RuntimeServiceOwners(roles)

    def describe(self) -> dict:
        required = {"gazebo", "dds-agent", "ros2-monitor", "ros2-clock"}
        if os.environ.get("UAV_STEREO") == "true":
            required.add("camera-streams")
        if os.environ.get("UAV_LIVE_MAPPING") == "true":
            required.add("live-mapping")
        if os.environ.get("UAV_NAVIGATION_STEREO") == "true":
            required.add("navigation-stereo")
        if any(service.viewer is not None for service in self.services.values()):
            required.update({"gazebo-viewer", "qgroundcontrol"})
        reason = None
        try:
            missing = required - set(self.owners.roles)
            if missing:
                raise RuntimeError(
                    "Configure owning restore supervisors: " + ", ".join(sorted(missing))
                )
            for role in self.owners.roles:
                self.owners.state(role)
        except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
            reason = str(error)
        return {
            "available": reason is None,
            "reason": reason,
            "vehicle_ids": list(self.services),
            "services": list(self.owners.roles),
            "scope": "UAV process children only; control API and owner supervisors stay alive",
        }

    def enter(self) -> None:
        self.fleet.enter_maintenance()

    def leave(self) -> None:
        self.fleet.leave_maintenance()

    def state(self, vehicle_id: str) -> dict:
        service = self.services[vehicle_id]
        navigation = service.navigation.snapshot() if service.navigation else {}
        return {
            **service.vehicle.snapshot(),
            "navigation_active": navigation.get("active", False),
            "landing_acknowledged": navigation.get("landing_acknowledged"),
        }

    def prepare(self, vehicle_ids: tuple[str, ...]) -> None:
        # Shared paused physics must advance before heartbeat/landing checks can succeed.
        for identity in vehicle_ids:
            service = self.services[identity]
            service.world.start()
            service.paused = False
            service.vehicle.configure_link(0, 0, 0)
            service.vehicle.wait_connected(10)

    def land(self, vehicle_id: str) -> dict:
        service = self.services[vehicle_id]
        state = self.state(vehicle_id)
        if state["navigation_active"]:
            result = service.navigation.cancel()
            return {**result, "landing_acknowledgement_pending": True}
        if state.get("armed") is True:
            return service.execute("stop", {})
        return {"accepted": True, "already_disarmed": state.get("armed") is False}

    def _safe(self) -> None:
        deadline = time.monotonic() + 5
        while True:
            fresh = True
            for identifier in self.services:
                state = self.state(identifier)
                age = state.get("heartbeat_age_seconds")
                if (
                    state.get("connected") is not True
                    or state.get("armed") is not False
                    or state["navigation_active"]
                    or type(age) not in {int, float}
                    or age < 0
                ):
                    raise RuntimeError(
                        f"{identifier}: restart refused without fresh disarmed state"
                    )
                fresh = fresh and age <= 1.5
            if fresh:
                return
            if time.monotonic() >= deadline:
                raise TimeoutError("Fresh disarmed heartbeat quorum did not arrive before restart")
            # At reduced simulation speed the next heartbeat can be two wall seconds
            # away. Wait for fresh evidence; never extend its 1.5-second age limit.
            time.sleep(0.05)

    def restart(self, vehicle_ids: tuple[str, ...]) -> dict:
        self._safe()
        previous = {identity: self.state(identity)["run_id"] for identity in vehicle_ids}
        for identity in vehicle_ids:
            service = self.services[identity]
            if service.features:
                service.features.reset()
            if service.navigation:
                service.navigation.reset()
        restarted_at = time.time()
        generations = {}
        if "dds-agent" in self.owners.roles:
            generations["dds-agent"] = self.owners.restart("dds-agent", self._safe)
        # Fresh clients must discover a stable agent after the world clock restarts.
        generations["gazebo"] = self.owners.restart("gazebo", self._safe)
        # Gazebo's generation marker makes existing PX4 fleet owners replace their children.
        controllers = {}
        for identity in vehicle_ids:
            controllers[identity] = wait_for_reset_readiness(
                identity,
                self.services[identity].vehicle.snapshot,
                previous[identity],
                45,
            )
        for role in self.owners.roles:
            if role not in generations:
                generations[role] = self.owners.restart(role, self._safe)
        deadline = time.monotonic() + 30
        pending = []
        while time.monotonic() < deadline:
            pending = []
            for role, window_name in (
                ("gazebo-viewer", "Gazebo.window"),
                ("qgroundcontrol", "QGroundControl.window"),
            ):
                if role in generations:
                    window = Path("/runtime") / window_name
                    owned = current_service_window(window, self.owners.state(role), restarted_at)
                    if not owned:
                        pending.append(f"{role}: fresh window belonging to its new service child")
            for identity in vehicle_ids:
                service = self.services[identity]
                if "camera-streams" in generations and not service.camera_status.snapshot().get(
                    "ready"
                ):
                    pending.append(f"{identity}: fresh camera frames")
                if "live-mapping" in generations:
                    mapping = read_live_map(identity)
                    if (
                        mapping.get("controller_run_id") != controllers[identity]
                        or mapping.get("received_wall_time", 0) < restarted_at
                        or mapping.get("stale", True)
                    ):
                        pending.append(f"{identity}: new mapping publication")
                if "ros2-monitor" in generations:
                    root = (
                        Path("/runtime") if identity == "vehicle_1" else Path("/runtime") / identity
                    )
                    try:
                        ros = json.loads((root / "ros2.json").read_text())
                        if (
                            ros.get("received_wall_time", 0) < restarted_at
                            or not 0 <= time.time() - ros.get("received_wall_time", 0) < 3
                        ):
                            pending.append(f"{identity}: fresh ROS observation")
                    except (OSError, ValueError):
                        pending.append(f"{identity}: ROS observation unavailable")
            if not pending:
                break
            time.sleep(0.1)
        if pending:
            raise TimeoutError("Restore dependencies unavailable: " + "; ".join(pending))
        for role, generation in generations.items():
            if self.owners.state(role)["generation"] != generation:
                raise RuntimeError(
                    f"{role}: process generation changed after restart acknowledgement"
                )
        return {
            "controller_generations": controllers,
            "service_generations": generations,
            "mapping_tracking_not_claimed": True,
        }

    def defaults(self) -> dict:
        self._safe()
        preferences = DesktopPreferences(self.output_directory / "preferences/desktop.json")
        preferences.save(DEFAULT_PREFERENCES)
        # Native view owners have restarted. Camera placement remains an acknowledged adapter operation.
        camera = None
        camera_identity = os.environ.get("UAV_DEFAULT_VIEW_VEHICLE_ID", "vehicle_1")
        if any(service.viewer is not None for service in self.services.values()):
            if camera_identity not in self.services:
                raise ValueError("Configured default camera vehicle is unavailable")
            camera = self.services[camera_identity].execute(
                "viewer/camera",
                {"preset": "chase", "follow": True},
            )
        return {
            "preferences": dict(DEFAULT_PREFERENCES),
            "target_ids": [],
            "panel": "deployment startup panel",
            "camera": camera,
            "saved_records_preserved": True,
        }
