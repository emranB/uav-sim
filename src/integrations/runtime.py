"""Composition and resource ownership for the concrete multi-vehicle simulation."""

import os
import threading
import time
from contextlib import ExitStack
from dataclasses import asdict, replace

from common.configuration.fleet import VehicleDefinition, load_fleet
from common.configuration.offline_replay import OfflineReplayLimits
from common.configuration.runtime import RuntimeSettings
from common.configuration.visual_missions import VisualMissionLimits
from common.configuration.visual_navigation import VisualEndpoints, validate_visual_endpoints
from common.configuration.world import WorldLocation
from common.interfaces.replay import ReplayCaptureLimits
from common.lifecycle.restore import MasterReset
from common.lifecycle.sensor_health import observed_health
from common.logging.replay_capture import ReplayCapture
from common.logging.replay_export import ReplayExportJobs
from common.logging.replay_store import ReplayCaptureStore
from common.navigation.session import NavigationSession
from common.simulation_manager.fleet import FleetOperations
from common.simulation_manager.server import SimulationService
from common.simulation_manager.workspace import WorkspaceFeatures
from common.vehicle_manager.catalog import VehicleCatalog
from gazebo.scene import GazeboScene
from gazebo.simulation import GazeboSimulation
from gazebo.transport import GazeboControlConnection
from gazebo.viewer_control import GazeboViewer
from integrations.blender.queue import FileReplayRenderer
from integrations.mavlink.connection import SimulatedVehicleConnection
from integrations.px4.runtime import PX4Runtime
from integrations.px4.visual_aiding import VisualAidingGuard
from integrations.px4.visual_navigation import StereoHorizontalNavigation
from integrations.replay import WorkspaceReplaySource
from integrations.restore import WorkspaceRestoreRuntime


class SimulationWorkspace:
    def __init__(self, settings: RuntimeSettings) -> None:
        self.settings = settings
        self.services: dict[str, SimulationService] = {}
        self.replay: ReplayCapture | None = None
        self.restore: MasterReset | None = None
        self._operation_lock = threading.Lock()
        self._viewer_lock = threading.Lock()
        self._scene = GazeboScene()
        self._world_connection = GazeboControlConnection()
        self._viewer_connection = None if settings.headless else GazeboControlConnection()
        self._visual_aiding: dict[str, VisualAidingGuard] = {}
        self._visual_navigation = os.environ.get("UAV_VISUAL_ODOMETRY_PROBE") == "true"
        try:
            fleet = load_fleet()
            if self._visual_navigation and os.environ.get("GZ_PARTITION") != "uav-visual-odometry":
                raise RuntimeError("Visual navigation requires the isolated visual partition")
            if self._visual_navigation:
                validate_visual_endpoints(fleet)
            for definition in fleet:
                if definition.enabled:
                    self.services[definition.vehicle_id] = self._create_vehicle(definition)
            self.catalog = VehicleCatalog(
                settings.output_directory,
                self.services,
                self._create_vehicle,
                self._retire_vehicle,
                self._rename_vehicle,
                self._discard_vehicle,
            )
            replay_store = ReplayCaptureStore(
                settings.output_directory / "replay",
                ReplayCaptureLimits(
                    maximum_seconds=float(os.environ.get("UAV_REPLAY_CAPTURE_SECONDS", "600")),
                    maximum_bytes=int(
                        os.environ.get("UAV_REPLAY_CAPTURE_BYTES", str(64 * 1024**2))
                    ),
                ),
            )
            renderer = FileReplayRenderer(settings.output_directory / "replay-jobs")
            render_trigger = os.environ.get("UAV_REPLAY_RENDER_TRIGGER", "stop")
            if render_trigger not in {"stop", "keep"}:
                raise ValueError("Replay render trigger must be stop or keep")
            offline_limits = OfflineReplayLimits.from_environment(os.environ)
            self.replay = ReplayCapture(
                WorkspaceReplaySource(
                    self.services, settings.output_directory / "worlds/default.sdf"
                ),
                replay_store,
                ReplayExportJobs(
                    replay_store,
                    settings.output_directory / "replay-jobs",
                    renderer,
                    maximum_wall_seconds=offline_limits.maximum_wall_seconds,
                    maximum_output_bytes=offline_limits.maximum_output_bytes,
                    maximum_store_bytes=offline_limits.maximum_store_bytes,
                ),
                render_on_stop=render_trigger == "stop",
            )
            self.fleet = FleetOperations(self.services, self.catalog)
            self.restore = MasterReset(
                WorkspaceRestoreRuntime(self.services, self.fleet, settings.output_directory),
                settings.output_directory / "restores",
                self.replay,
            )
        except (ValueError, RuntimeError, OSError):
            self.close()
            raise

    def _create_vehicle(self, definition: VehicleDefinition) -> SimulationService:
        directory = definition.output_directory(self.settings.output_directory)
        vehicle = SimulatedVehicleConnection(
            directory / "telemetry",
            PX4Runtime(definition.runtime_directory),
            definition.vehicle_id,
            definition.system_id,
            definition.vehicle_port,
            definition.application_port,
            heartbeat_timeout_seconds=self.settings.heartbeat_timeout_seconds,
        )
        with ExitStack() as resources:
            resources.callback(vehicle.close)
            visual_aiding = None
            if self._visual_navigation:
                visual_aiding = VisualAidingGuard(
                    VisualEndpoints.for_vehicle(definition).status_port,
                    vehicle_id=definition.vehicle_id,
                    controller_generation=PX4Runtime(definition.runtime_directory).generation,
                )
                resources.callback(visual_aiding.close)
            configuration = {
                "vehicle_id": definition.vehicle_id,
                "label": definition.label,
                "flight_controller": "px4_sitl",
                "model": "x500",
                "mavlink_system_id": definition.system_id,
                "model_name": definition.model_name,
                "ros_namespace": f"/{definition.vehicle_id}",
                "display_launch_enu_m": list(definition.spawn_enu_m),
            }
            world_configuration = WorldLocation.configured().as_dict()
            scene_name = os.environ.get("UAV_SCENE", "default")
            if scene_name not in {"default", "inspection", "waterfront"}:
                raise ValueError("Scene must be default, inspection or waterfront")
            if scene_name != "default":
                world_configuration["scene_name"] = scene_name
            features = WorkspaceFeatures(
                vehicle,
                directory,
                world_configuration,
                configuration,
                definition.runtime_directory,
                self.settings.output_directory,
            )
            viewer = (
                GazeboViewer(
                    self._scene,
                    definition.model_name,
                    definition.vehicle_id,
                    self._viewer_connection,
                )
                if self._viewer_connection
                else None
            )
            service = SimulationService(
                replace(self.settings, output_directory=directory),
                vehicle,
                GazeboSimulation(
                    definition.model_name, definition.spawn_enu_m, self._world_connection
                ),
                self._scene,
                features=features,
                viewer=viewer,
                vehicle_id=definition.vehicle_id,
                shared_output_directory=self.settings.output_directory,
                visual_mission_limits=VisualMissionLimits.from_environment(os.environ),
                visual_mission_obstacle_mode=os.environ.get(
                    "UAV_VISUAL_MISSION_OBSTACLE_MODE", "camera"
                ),
                visual_mission_name=(
                    os.environ.get("UAV_VISUAL_MISSION_NAME") or None
                    if self._visual_navigation
                    else None
                ),
            )
            service.operation_lock = (
                threading.Lock() if self._visual_navigation else self._operation_lock
            )
            service.viewer_lock = self._viewer_lock
            if os.environ.get("UAV_NAVIGATION", "false") == "true":
                from common.navigation.occupancy import GridConfiguration
                from gazebo.ranges import GazeboRanges

                stereo_source = None
                if os.environ.get("UAV_NAVIGATION_STEREO", "false") == "true":
                    from integrations.vision.navigation_source import FileStereoObstacles

                    stereo_source = FileStereoObstacles(
                        definition.vehicle_id, definition.runtime_directory
                    )
                service.navigation = NavigationSession(
                    definition.vehicle_id,
                    vehicle,
                    GazeboRanges(definition.model_name),
                    service.operation_lock,
                    directory / "navigation",
                    stereo=stereo_source,
                    localization_guard=visual_aiding,
                    localization_snapshot=visual_aiding.snapshot if visual_aiding else None,
                    stereo_reference_factory=(
                        (lambda: StereoHorizontalNavigation(vehicle, visual_aiding))
                        if visual_aiding
                        else None
                    ),
                    grid_configuration=GridConfiguration(
                        half_width_m=max(
                            18.0, VisualMissionLimits.from_environment(os.environ).maximum_radius_m
                        )
                    ),
                )
            self._scene.register_vehicle(definition.model_name, definition.vehicle_id)
            if visual_aiding is not None:
                self._visual_aiding[definition.vehicle_id] = visual_aiding
            resources.pop_all()
            return service

    def _retire_vehicle(self, definition: VehicleDefinition, service: SimulationService) -> None:
        PX4Runtime(definition.runtime_directory).wait_stopped()
        service.world.remove()
        self._discard_vehicle(definition, service)

    def _discard_vehicle(self, definition: VehicleDefinition, service: SimulationService) -> None:
        if service.navigation:
            service.navigation.close()
        service.vehicle.close()
        visual_aiding = self._visual_aiding.pop(definition.vehicle_id, None)
        if visual_aiding is not None:
            visual_aiding.close()
        self._scene.remove_vehicle(definition.model_name, definition.vehicle_id)

    def _rename_vehicle(self, identifier: str, label: str) -> None:
        self.services[identifier].features.configuration["label"] = label

    def health_snapshot(self, identifier: str) -> list[dict]:
        """Translate measured owners at composition; panels receive no protocol objects."""
        service = self.services[identifier]
        observation = WorkspaceReplaySource(self.services).observe(identifier)
        return [
            asdict(health)
            for health in observed_health(
                observation,
                service.vehicle.snapshot()["connected"],
                time.monotonic(),
                visual_supported=service.navigation is not None
                and service.navigation.localization_snapshot is not None,
            )
        ]

    def close(self) -> None:
        if self.restore is not None:
            self.restore.close()
        if self.replay is not None:
            self.replay.close()
        for service in list(self.services.values()):
            if service.navigation:
                service.navigation.close()
            service.vehicle.close()
        self._world_connection.close()
        if self._viewer_connection:
            self._viewer_connection.close()
        for visual_aiding in self._visual_aiding.values():
            visual_aiding.close()
        self._visual_aiding.clear()
        self._scene.close()
