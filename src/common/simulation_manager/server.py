"""Local API for the real, isolated PX4/Gazebo simulation."""

import json
import logging
import math
import os
import signal
import threading
import time
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import TYPE_CHECKING
from urllib.parse import parse_qs, urlsplit

from common.configuration.missions import parse_plan
from common.configuration.runtime import RuntimeSettings
from common.configuration.visual_missions import VisualMissionLimits
from common.interfaces import SimulationAdapter
from common.interfaces.capabilities import CapabilityUnavailable
from common.interfaces.scene import SceneSource, ViewerAdapter
from common.interfaces.vehicles import VehicleAdapter
from common.lifecycle.readiness import wait_for_reset_readiness
from common.logging.camera_status import CameraStatus
from common.logging.live_map import read_live_map
from common.logging.storage import StorageCapacityError, ensure_run_capacity
from common.logging.structured import recording_handler
from common.navigation.mission import VisualMission
from common.simulation_manager.fleet import FleetOperations
from common.simulation_manager.flight_actions import flight_action_status
from common.simulation_manager.perception import vehicle_perception
from common.simulation_manager.workspace import WorkspaceFeatures

if TYPE_CHECKING:
    from common.lifecycle.restore import MasterReset
    from common.logging.replay_capture import ReplayCapture
    from common.vehicle_manager.catalog import VehicleCatalog


class SimulationService:
    def __init__(
        self,
        settings: RuntimeSettings,
        vehicle: VehicleAdapter,
        world: SimulationAdapter,
        scene: SceneSource,
        reset_views: Callable[[], None] | None = None,
        features: WorkspaceFeatures | None = None,
        viewer: ViewerAdapter | None = None,
        vehicle_id: str = "vehicle_1",
        shared_output_directory: Path | None = None,
        visual_mission_name: str | None = None,
        visual_mission_limits: VisualMissionLimits | None = None,
        visual_mission_obstacle_mode: str = "camera",
    ) -> None:
        self.settings = settings
        self.vehicle = vehicle
        self.world = world
        self.scene_source = scene
        self.reset_views = reset_views
        self.features = features
        self.viewer = viewer
        self.vehicle_id = vehicle_id
        self.visual_mission_name = visual_mission_name
        self.visual_mission_limits = visual_mission_limits or VisualMissionLimits()
        if visual_mission_obstacle_mode not in {"camera", "range", "fused"}:
            raise ValueError("Visual mission obstacle mode must be camera, range or fused")
        self.visual_mission_obstacle_mode = visual_mission_obstacle_mode
        self.operation_lock = threading.Lock()
        self.viewer_lock = threading.Lock()
        self.recording_lock = threading.Lock()
        self.perception = vehicle_perception(
            os.environ.get("UAV_STEREO", "false") == "true",
            os.environ.get("UAV_LIVE_MAPPING", "false") == "true",
            os.environ.get("UAV_NAVIGATION", "false") == "true",
        )
        self.camera_status = CameraStatus(
            shared_output_directory or settings.output_directory,
            vehicle_id,
        )
        self.navigation = None
        self.paused = False
        self.vehicle.start()

    @property
    def navigation_active(self) -> bool:
        """Whether this vehicle currently owns a bounded navigation operation."""
        return self.navigation is not None and self.navigation.snapshot().get("active") is True

    def flight_actions(self) -> dict:
        """Expose the owning service's current shortcut capabilities and readiness."""
        snapshot = self.vehicle.snapshot()
        status = flight_action_status(snapshot, self.navigation_active)
        if self.visual_mission_name and self.navigation and self.navigation.localization_guard:
            reason = (
                "Wait for the vehicle connection"
                if not snapshot["connected"]
                else "Land and disarm before starting a mission"
                if snapshot["armed"]
                else "Cancel the active navigation run first"
                if self.navigation_active
                else "Wait for PX4 preflight checks"
                if not snapshot.get("preflight_ready")
                else "Restore recording readiness"
                if not (snapshot.get("recording") or {}).get("available", True)
                else ""
            )
            if not reason:
                try:
                    self.navigation.localization_guard(snapshot["run_id"])
                    self._visual_mission()
                except (OSError, ValueError, RuntimeError) as error:
                    reason = str(error)
            status["start"] = {
                "supported": True,
                "enabled": not reason,
                "reason": reason,
                "mission_name": self.visual_mission_name,
            }
        return status

    def _visual_mission(self) -> VisualMission:
        if self.features is None or not self.visual_mission_name:
            raise RuntimeError("Select a saved visual mission before starting")
        plan, digest = self.features.missions.load_with_digest(self.visual_mission_name)
        return VisualMission.from_plan(
            self.visual_mission_name, digest, plan, self.visual_mission_limits
        )

    def execute(self, operation: str, body: dict) -> dict:
        if operation == "flight/start":
            if self.visual_mission_name:
                mission = self._visual_mission()
                return self.execute(
                    "navigation/mission",
                    {
                        "name": mission.name,
                        "expected_sha256": mission.sha256,
                        "stereo_horizontal_reference": body.get(
                            "stereo_horizontal_reference", False
                        ),
                    },
                )
            return self.execute("start", {})
        if operation.startswith("navigation/"):
            if self.navigation is None:
                raise RuntimeError("Navigation unavailable: enable the explicit avoidance overlay")
            if operation == "navigation/start":
                return self.navigation.start(body)
            if operation == "navigation/mission":
                if self.features is None or self.navigation.localization_guard is None:
                    raise RuntimeError(
                        "Saved visual missions require the visual navigation profile"
                    )
                name = body.get("name")
                plan, digest = self.features.missions.load_with_digest(name)
                if body.get("expected_sha256") != digest:
                    raise ValueError("Saved mission hash changed; select the frozen mission again")
                mission = VisualMission.from_plan(name, digest, plan, self.visual_mission_limits)
                return self.navigation.start(
                    {
                        "obstacle_mode": self.visual_mission_obstacle_mode,
                        "return_to_start": True,
                        "goal_offset_enu_m": list(mission.goal_offset_enu_m),
                        "stereo_horizontal_reference": body.get(
                            "stereo_horizontal_reference", False
                        ),
                    },
                    mission=mission,
                )
            if operation == "navigation/cameras":
                return self.navigation.configure_camera_input(body.get("settings"))
            if operation == "navigation/ranges":
                self.navigation.ranges.set_enabled(body.get("enabled"))
                return {
                    "accepted": True,
                    "vehicle_id": self.vehicle_id,
                    "range_subscription_enabled": body["enabled"],
                }
            if operation == "navigation/cancel":
                return self.navigation.cancel()
            raise ValueError("Unknown navigation operation")
        if (
            operation == "stop"
            and self.navigation
            and self.navigation.snapshot().get("state")
            in {
                "starting",
                "taking_off",
                "navigating",
                "waiting_for_fresh_observation",
                "blocked",
                "landing",
            }
        ):
            return self.navigation.cancel()
        if operation == "perception/check":
            capability = self.perception.require(body.get("capability_id"), self.vehicle_id)
            if capability.key in {"obstacle_avoidance", "path_planning"}:
                if not self.navigation:
                    raise RuntimeError("Navigation adapter is unavailable")
                ready = self.navigation.sensing_ready(body.get("obstacle_mode", "range"))["ready"]
            elif capability.key == "stereo_capture":
                ready = self.camera_status.snapshot()["ready"]
            else:
                mapping = read_live_map(self.vehicle_id)
                ready = mapping["ready"]
                if capability.key in {"stereo_depth", "surface_geometry"}:
                    ready = not mapping["stale"] and mapping.get("valid_depth_fraction", 0) > 0
            return {
                "accepted": True,
                "implemented": capability.implemented,
                "ready": ready,
            }
        if operation in {"viewer/camera", "viewer/follow"}:
            return self._camera(body, operation)
        if operation == "sensors/gnss" and self.navigation_active:
            if self.features is None:
                raise RuntimeError("GNSS input control is unavailable")
            # Navigation retains arm/setpoint authority. Its declared simulation
            # fault input has its own owner lock and acknowledged parameter path.
            return self.features.execute(operation, body)
        if operation == "recordings/compare":
            if self.features is None:
                raise RuntimeError("Recording comparison is unavailable")
            if not self.recording_lock.acquire(blocking=False):
                raise RuntimeError("Another recording comparison is in progress")
            try:
                return self.features.execute(operation, body)
            finally:
                self.recording_lock.release()
        if not self.operation_lock.acquire(blocking=False):
            raise RuntimeError("Another simulation operation is in progress")
        try:
            if operation == "pause":
                self.world.pause()
                self.paused = True
            elif operation == "resume":
                self.world.start()
                self.paused = False
            elif operation == "start":
                ensure_run_capacity(self.settings.output_directory)
                self.world.start()
                self.paused = False
                self.vehicle.wait_connected(10)
                if not self.vehicle.snapshot()["armed"]:
                    if not self.vehicle.download()["mission"]["items"]:
                        raise RuntimeError("Upload a mission before starting the vehicle")
                    if not self.vehicle.snapshot().get("preflight_ready", True):
                        raise RuntimeError(
                            "PX4 preflight checks are not ready; wait before starting"
                        )
                    self.vehicle.arm()
                    self.vehicle.start_mission()
            elif operation == "stop":
                self.world.start()
                self.paused = False
                self.vehicle.wait_connected(10)
                if self.vehicle.snapshot()["armed"]:
                    self.vehicle.land()
            elif operation == "reset":
                if self.vehicle.snapshot()["armed"]:
                    raise RuntimeError("Land and wait for disarm before resetting the simulator")
                if self.features:
                    self.features.reset()
                self.world.start()
                self.paused = False
                self.vehicle.wait_connected(10)
                if self.vehicle.snapshot()["armed"]:
                    raise RuntimeError("Land and wait for disarm before resetting the simulator")
                if self.navigation:
                    self.navigation.reset()
                self.world.reset()
                self.paused = False
                self.scene_source.begin_run()
                previous_generation = self.vehicle.snapshot().get("run_id")
                self.vehicle.restart()
                generation = wait_for_reset_readiness(
                    self.vehicle_id,
                    self.vehicle.snapshot,
                    previous_generation,
                )
                if self.reset_views:
                    self.reset_views()
                return {
                    "accepted": True,
                    "operation": operation,
                    "vehicle_id": self.vehicle_id,
                    "controller_run_id": generation,
                    "reset_complete": True,
                }
            elif operation == "mission":
                if self.vehicle.snapshot()["armed"]:
                    raise RuntimeError("Land before replacing the vehicle mission")
                self.vehicle.upload(parse_plan(body))
                destination = self.settings.output_directory / "missions"
                destination.mkdir(parents=True, exist_ok=True)
                (destination / f"mission-{time.time_ns()}.plan").write_text(
                    json.dumps(body, indent=2),
                    encoding="utf-8",
                )
            elif self.features:
                return self.features.execute(operation, body)
            else:
                raise ValueError(f"Unknown operation: {operation}")
            return {"accepted": True, "operation": operation, "vehicle_id": self.vehicle_id}
        finally:
            self.operation_lock.release()

    def scene(self) -> dict:
        return self.scene_source.snapshot()

    def _camera(self, body: dict, operation: str = "viewer/camera") -> dict:
        if not self.viewer_lock.acquire(blocking=False):
            raise RuntimeError("Another camera operation is in progress")
        try:
            vehicle_id = self.vehicle_id
            logging.info(
                "Camera request vehicle=%s preset=%s follow=%s",
                vehicle_id,
                body.get("preset"),
                body.get("follow"),
            )
            poses = self.scene_source.snapshot()["poses"]
            pose = next((pose for pose in poses if pose["vehicle_id"] == vehicle_id), None)
            if self.viewer is None or pose is None:
                raise RuntimeError("Camera controls require the selected vehicle and viewer")
            if "look_at_enu_m" in body:
                focus = body["look_at_enu_m"]
                if (
                    body.get("follow") is not False
                    or not isinstance(focus, list)
                    or len(focus) != 3
                    or any(
                        type(v) not in {int, float} or not math.isfinite(v) or abs(v) > 1000
                        for v in focus
                    )
                ):
                    raise ValueError(
                        "Overview requires a finite ENU focus within1000m and Follow off"
                    )
                self.viewer.set_camera(body.get("preset", "overview"), False, tuple(focus))
                return {"accepted": True, "operation": operation, "vehicle_id": vehicle_id}
            if operation == "viewer/follow":
                self.viewer.set_follow(body.get("follow"), tuple(pose["position_enu_m"]))
            elif "offset_enu_m" in body:
                self.viewer.set_camera(
                    "custom",
                    body.get("follow", True),
                    tuple(pose["position_enu_m"]),
                    body["offset_enu_m"],
                )
            else:
                self.viewer.set_camera(
                    body.get("preset", "chase"),
                    body.get("follow", True),
                    tuple(pose["position_enu_m"]),
                )
            return {"accepted": True, "operation": operation, "vehicle_id": vehicle_id}
        finally:
            self.viewer_lock.release()


class SimulationServer(ThreadingHTTPServer):
    def __init__(
        self,
        service: SimulationService,
        services: dict | None = None,
        catalog: "VehicleCatalog | None" = None,
        replay: "ReplayCapture | None" = None,
        health_snapshot: Callable[[str], list[dict]] | None = None,
        fleet: FleetOperations | None = None,
        restore: "MasterReset | None" = None,
    ) -> None:
        self.service = service
        self.services = services or {service.vehicle_id: service}
        self.fleet = fleet or FleetOperations(self.services, catalog)
        self.restore = restore
        self.replay = replay
        self.health_snapshot = health_snapshot
        super().__init__(("0.0.0.0", 8080), Handler)


class Handler(BaseHTTPRequestHandler):
    server: SimulationServer

    def respond(self, status: int, body: dict) -> None:
        if status >= 400 and "error" in body:
            logging.warning("%s failed: %s", self.path, body["error"])
        encoded = json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self) -> None:
        resource = urlsplit(self.path)
        if resource.path == "/restore":
            if self.server.restore is None:
                self.respond(409, {"error": "Master Reset is unavailable"})
            else:
                self.respond(
                    200,
                    {
                        "restore": self.server.restore.snapshot(),
                        "availability": self.server.restore.availability(),
                    },
                )
            return
        if resource.path == "/replay":
            if self.server.replay is None:
                self.respond(409, {"error": "Replay capture is unavailable"})
            else:
                self.respond(200, self.server.replay.inventory())
            return
        if resource.path == "/navigation/fleet":
            items = []
            for identity, item in list(self.server.services.items()):
                telemetry = item.vehicle.snapshot()
                navigation = item.navigation.snapshot() if item.navigation else {}
                fields = (
                    "state",
                    "active",
                    "run_id",
                    "controller_run_id",
                    "leg",
                    "current_waypoint",
                    "saved_mission",
                    "waypoint_events",
                    "home_local_enu_m",
                    "goal_offset_enu_m",
                    "visual_localization",
                    "error",
                    "outcome",
                )
                items.append(
                    {
                        "vehicle_id": identity,
                        "label": item.features.configuration["label"],
                        "controller_generation": telemetry.get("run_id"),
                        "display_launch_enu_m": item.features.configuration.get(
                            "display_launch_enu_m", [0, 0, 0]
                        ),
                        "connected": telemetry["connected"],
                        "armed": telemetry["armed"],
                        "recording": telemetry.get("recording"),
                        "health": self.server.health_snapshot(identity)
                        if self.server.health_snapshot is not None
                        else [],
                        "navigation": {key: navigation.get(key) for key in fields},
                    }
                )
            self.respond(
                200,
                {
                    "items": items,
                    "scene_id": Path(
                        os.environ.get(
                            "UAV_AVOIDANCE_SCENARIO",
                            "configs/navigation/stereo/visual-odometry-probe.json",
                        )
                    ).stem,
                },
            )
            return
        if resource.path == "/vehicles":
            self.respond(
                200,
                {
                    "items": [
                        {
                            **item.features.configuration,
                            **item.vehicle.snapshot(),
                            "flight_actions": item.flight_actions(),
                            "health": self.server.health_snapshot(item.vehicle_id)
                            if self.server.health_snapshot is not None
                            else [],
                        }
                        for item in list(self.server.services.values())
                    ]
                },
            )
            return
        selected = parse_qs(resource.query).get("vehicle_id", [])
        if (
            resource.path.startswith("/perception") or resource.path.startswith("/navigation")
        ) and len(selected) != 1:
            self.respond(400, {"error": "Perception requests require one explicit vehicle_id"})
            return
        vehicle_id = (selected or [next(iter(self.server.services))])[0]
        if vehicle_id not in self.server.services:
            self.respond(400, {"error": "Unknown vehicle identifier"})
            return
        service = self.server.services[vehicle_id]
        if resource.path == "/navigation/obstacles":
            try:
                if service.navigation is None:
                    raise RuntimeError("Navigation sensing is not configured")
                mode = parse_qs(resource.query).get("obstacle_mode", ["range"])[0]
                self.respond(200, service.navigation.preview_obstacles(mode))
            except (ValueError, RuntimeError, OSError) as error:
                self.respond(409, {"error": str(error), "vehicle_id": vehicle_id})
            return
        if resource.path == "/navigation":
            self.respond(
                200,
                service.navigation.snapshot()
                if service.navigation
                else {
                    "vehicle_id": vehicle_id,
                    "available": False,
                    "state": "unavailable",
                    "error": "Enable the explicit simulation avoidance overlay",
                },
            )
            return
        if resource.path == "/perception/map":
            self.respond(200, read_live_map(vehicle_id))
            return
        if resource.path == "/perception":
            self.respond(
                200,
                {
                    **service.perception.snapshot(vehicle_id),
                    "camera": service.camera_status.snapshot(),
                },
            )
            return
        if resource.path == "/perception/image":
            try:
                side = parse_qs(resource.query).get("side", [""])[0]
                image = service.camera_status.image(side)
                self.send_response(200)
                self.send_header("Content-Type", "image/x-portable-pixmap")
                self.send_header("Content-Length", str(len(image)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(image)
            except (ValueError, OSError, RuntimeError) as error:
                self.respond(409, {"error": str(error), "vehicle_id": vehicle_id})
            return
        if resource.path.lstrip("/") in {
            "workspace",
            "missions",
            "experiments",
            "recordings",
            "logs",
        }:
            try:
                if service.features is None:
                    raise RuntimeError("Workspace features are unavailable")
                payload = service.features.read(resource.path.lstrip("/"), parse_qs(resource.query))
                if resource.path == "/workspace":
                    scene = service.scene()
                    payload["viewer"] = scene.get("viewer", {})
                    pose = next(
                        (pose for pose in scene["poses"] if pose["vehicle_id"] == vehicle_id), None
                    )
                    if pose:
                        payload["viewer"]["vehicle_position_enu_m"] = pose["position_enu_m"]
                self.respond(200, payload)
            except (ValueError, OSError, RuntimeError) as error:
                self.respond(409, {"error": str(error)})
            return
        if resource.path == "/telemetry":
            self.respond(
                200,
                {
                    **service.vehicle.snapshot(),
                    "simulation_paused": service.paused,
                    "scene_epoch": service.scene()["epoch"],
                    "health": self.server.health_snapshot(vehicle_id)
                    if self.server.health_snapshot is not None
                    else [],
                },
            )
        elif resource.path == "/mission":
            try:
                self.respond(200, service.vehicle.download())
            except (RuntimeError, TimeoutError) as error:
                self.respond(409, {"error": str(error)})
        elif resource.path == "/scene":
            self.respond(200, service.scene())
        elif resource.path in {"/health", "/capabilities"}:
            states = {vehicle_id: service.vehicle.snapshot()}
            if resource.path == "/health" and not resource.query:
                states = {
                    identifier: item.vehicle.snapshot()
                    for identifier, item in self.server.services.items()
                }
            recording_failures = {
                identifier: state["recording"].get("error") or "Recording unavailable"
                for identifier, state in states.items()
                if state.get("recording") is not None and not state["recording"]["available"]
            }
            connected = all(state["ready"] for state in states.values()) and not recording_failures
            try:
                ros_path = (
                    Path("/runtime/ros2.json")
                    if vehicle_id == "vehicle_1"
                    else Path(f"/runtime/{vehicle_id}/ros2.json")
                )
                ros_state = json.loads(ros_path.read_text())
                ros_connected = time.time() - ros_state["received_wall_time"] < 3
            except (OSError, ValueError, KeyError):
                ros_connected = False
            try:
                storage = {
                    "available": True,
                    **ensure_run_capacity(service.settings.output_directory),
                }
            except OSError as error:
                storage = {"available": False, "error": str(error)}
                connected = False
            if not storage["available"]:
                status = "storage-unavailable"
            elif recording_failures:
                status = "recording-unavailable"
            else:
                status = "ready" if connected else "waiting-for-px4"
            self.respond(
                200 if connected else 503,
                {
                    "status": status,
                    "storage": storage,
                    "recording_failures": recording_failures,
                    "mode": "simulation",
                    "real_simulator": True,
                    "requested_runtime": service.settings.requested_capabilities(),
                    "viewer_independent": True,
                    "rendering_validated": False,
                    "ros_connected": ros_connected,
                },
            )
        else:
            self.respond(404, {"error": "Unknown endpoint"})

    def do_POST(self) -> None:
        if self.headers.get("Origin"):
            self.respond(403, {"error": "Browser-origin control requests are not accepted"})
            return
        if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
            self.respond(415, {"error": "Control requests require application/json"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 <= size <= 1024 * 1024:
                raise ValueError("Request body exceeds 1 MiB")
            body = json.loads(self.rfile.read(size)) if size else {}
            if not isinstance(body, dict):
                raise ValueError("Expected a JSON object")
            operation = self.path.lstrip("/")
            if operation in {"restore/start", "restore/cancel"}:
                if self.server.restore is None:
                    raise RuntimeError("Master Reset is unavailable")
                result = (
                    self.server.restore.start()
                    if operation == "restore/start"
                    else self.server.restore.cancel()
                )
            elif operation.startswith("replay/"):
                if self.server.replay is None:
                    raise RuntimeError("Replay capture is unavailable")
                if (
                    self.server.restore is not None
                    and self.server.restore.snapshot()["active"]
                    and operation not in {"replay/stop", "replay/cancel"}
                ):
                    raise RuntimeError("Master Reset is finalizing offline replay work")
                result = self.server.replay.execute(operation, body)
            else:
                result = self.server.fleet.execute(operation, body)
            self.respond(200, result)
        except CapabilityUnavailable as error:
            self.respond(409, error.result())
        except (ValueError, KeyError, TypeError) as error:
            self.respond(400, {"error": str(error)})
        except (RuntimeError, TimeoutError) as error:
            self.respond(409, {"error": str(error)})
        except StorageCapacityError as error:
            self.respond(409, {"code": "STORAGE_CAPACITY", "error": str(error)})
        except OSError as error:
            self.respond(409, {"error": f"Cannot access the requested saved record: {error}"})

    def log_message(self, fmt: str, *args: object) -> None:
        logging.info(
            (fmt % args) + " source=%r", self.headers.get("X-Request-Source", "unspecified")[:80]
        )


def main() -> None:
    from integrations.runtime import SimulationWorkspace

    settings = RuntimeSettings.from_environment(os.environ)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(message)s",
        handlers=[
            logging.StreamHandler(),
            recording_handler(settings.output_directory / "logs", "application"),
        ],
    )
    workspace = SimulationWorkspace(settings)
    server = SimulationServer(
        next(iter(workspace.services.values())),
        workspace.services,
        workspace.catalog,
        replay=workspace.replay,
        health_snapshot=workspace.health_snapshot,
        fleet=workspace.fleet,
        restore=workspace.restore,
    )
    signal.signal(
        signal.SIGTERM, lambda signum, frame: threading.Thread(target=server.shutdown).start()
    )
    try:
        server.serve_forever()
    finally:
        server.server_close()
        workspace.close()


if __name__ == "__main__":
    main()
