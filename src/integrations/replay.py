"""Translate existing measured application snapshots into read-only replay evidence."""

import base64
import hashlib
import json
import os
import time
from dataclasses import fields
from pathlib import Path
from typing import TYPE_CHECKING

from common.interfaces.health import SystemHealthEvidence
from common.interfaces.replay import (
    ReplayAidingEvidence,
    ReplayFrameBinding,
    ReplayMissionReference,
    ReplayObservation,
    ReplayRecorderHealth,
    ReplayWaypointEvent,
)
from common.interfaces.scene import ScenePose
from integrations.mavlink.health import reported_sensor_health

if TYPE_CHECKING:
    from common.simulation_manager.server import SimulationService


class WorkspaceReplaySource:
    """Observe current owners without environment synchronization or command methods."""

    def __init__(
        self,
        services: dict[str, "SimulationService"],
        world_file: Path | None = None,
    ) -> None:
        self.services = services
        self.world_file = world_file

    def vehicle_ids(self) -> tuple[str, ...]:
        return tuple(self.services)

    def context(self) -> dict[str, object]:
        """Retain exact declared scene input; unresolved geometry stays explicit."""
        source_root = Path(__file__).resolve().parents[2]
        digest = hashlib.sha256()
        for path in sorted((source_root / "src").rglob("*.py")):
            digest.update(str(path.relative_to(source_root)).encode())
            digest.update(path.read_bytes())
        scene_path = (
            Path(os.environ["UAV_AVOIDANCE_SCENARIO"])
            if os.environ.get("UAV_AVOIDANCE_SCENARIO")
            else None
        )
        scene = None
        if scene_path and scene_path.is_file() and scene_path.stat().st_size <= 256 * 1024:
            encoded = scene_path.read_bytes()
            scene = {
                "path": str(scene_path),
                "sha256": hashlib.sha256(encoded).hexdigest(),
                "contents": encoded.decode("utf-8"),
            }
        world = None
        presentation = None
        if (
            self.world_file
            and self.world_file.is_file()
            and self.world_file.stat().st_size <= 1024 * 1024
        ):
            contents = self.world_file.read_bytes()
            world = {
                "sha256": hashlib.sha256(contents).hexdigest(),
                "contents": contents.decode("utf-8"),
                "external_assets_complete": False,
            }
            if b"presentation_pad_trim" in contents:
                package = source_root / "src/gazebo/models/presentation"
                presentation = {
                    name: (package / name).read_text(encoding="utf-8")
                    for name in ("manifest.json", "pad_trim.obj", "pad_labels.obj", "LICENSE.txt")
                    if (package / name).is_file() and (package / name).stat().st_size <= 256 * 1024
                }
        return {
            "source_python_sha256": digest.hexdigest(),
            "source_revision": os.environ.get("UAV_SOURCE_REVISION"),
            "frame": "estimated local ENU metres; scene ENU requires explicit launch binding",
            "wall_clock": "UTC receive time and process monotonic seconds",
            "simulation_clock": "PX4 pose simulation seconds; world epoch recorded per observation",
            "declared_scene_input": scene,
            "retained_world_sdf": world,
            "retained_presentation_assets": presentation,
            "retained_geographic_assets": self.geographic_assets(),
            "retained_geographic_images": self.geographic_images(),
            "complete_render_scene_retained": False,
            "render_unavailable_reason": (
                "Complete scene assets have not been retained in this package"
            ),
            "vehicles": {
                identifier: {
                    "configuration": dict(service.features.configuration),
                    "world": dict(service.features.world),
                    "native_revisions": service.features.versions(),
                    "camera_source_status": service.camera_status.snapshot(),
                }
                for identifier, service in list(self.services.items())
                if service.features
            },
        }

    def geographic_assets(self) -> dict:
        """Freeze bounded geographic mesh bytes at capture start, never at render time."""
        assets = {}
        paths = []
        if os.environ.get("UAV_TERRAIN_DIR"):
            paths.append(Path(os.environ["UAV_TERRAIN_DIR"]) / "terrain.obj")
        if os.environ.get("UAV_WATERFRONT_BUILDINGS_DIR"):
            directory = Path(os.environ["UAV_WATERFRONT_BUILDINGS_DIR"])
            manifest = json.loads((directory / "buildings.json").read_text())
            if len(manifest["buildings"]) > 500 or any(
                Path(row["file"]).name != row["file"] or not row["file"].endswith(".obj")
                for row in manifest["buildings"]
            ):
                raise ValueError("Geographic replay requires at most500 plain OBJ filenames")
            paths.extend(directory / row["file"] for row in manifest["buildings"])
        total = 0
        for path in paths:
            if path.is_symlink() or not path.is_file() or path.name in assets:
                raise ValueError(f"Geographic replay mesh unavailable or duplicated: {path.name}")
            total += path.stat().st_size
            if total > 32 * 1024**2:
                raise ValueError("Geographic replay assets exceed 32MiB")
            contents = path.read_text(encoding="utf-8")
            assets[path.name] = {
                "contents": contents,
                "sha256": hashlib.sha256(contents.encode()).hexdigest(),
            }
        return assets

    def geographic_images(self) -> dict:
        """Retain the terrain's actual albedo image with its immutable mesh."""
        if not os.environ.get("UAV_TERRAIN_DIR"):
            return {}
        path = Path(os.environ["UAV_TERRAIN_DIR"]) / "texture.png"
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 4 * 1024**2:
            raise ValueError("Terrain albedo must be a regular PNG smaller than4MiB")
        encoded = path.read_bytes()
        return {
            path.name: {
                "base64": base64.b64encode(encoded).decode("ascii"),
                "sha256": hashlib.sha256(encoded).hexdigest(),
            }
        }

    def observe(self, vehicle_id: str) -> ReplayObservation:
        """Keep estimated pose, measured aiding, diagnostic health and truth distinct."""
        service = self.services[vehicle_id]
        telemetry = service.vehicle.snapshot()
        generation = telemetry.get("run_id")
        navigation = service.navigation.snapshot() if service.navigation else {}
        localization = navigation.get("visual_localization") or {}
        native = localization.get("aiding") or {}
        if native.get("controller_run_id") != generation:
            native = {}
        aiding_values = {
            field.name: native.get(field.name) for field in fields(ReplayAidingEvidence)
        }
        aiding_values["received_monotonic"] = native.get("monotonic")
        if not native:
            gps = (telemetry.get("messages") or {}).get("GPS_RAW_INT") or {}
            aiding_values.update(
                received_monotonic=telemetry.get("gps_received_monotonic"),
                gps_fix_type=gps.get("fix_type"),
                satellites_used=gps.get("satellites_visible"),
            )
        pose_status = localization.get("pose") or {}
        if pose_status.get("controller_run_id") == generation:
            aiding_values["map_generation"] = pose_status.get("map_generation")
            aiding_values["visual_pose_received_monotonic"] = pose_status.get("monotonic")
            aiding_values["visual_pose_healthy"] = pose_status.get("healthy")
        recording = telemetry.get("recording") or {}
        capabilities = {
            item["key"]: item["implemented"]
            for item in service.perception.snapshot(vehicle_id)["items"]
        }
        camera_supported = capabilities.get("stereo_capture", False)
        camera = service.camera_status.snapshot() if camera_supported else {}
        camera_received = None
        if type(camera.get("received_wall_time")) in {int, float}:
            camera_age = time.time() - camera["received_wall_time"]
            if camera_age >= 0:
                camera_received = time.monotonic() - camera_age
        systems = list(reported_sensor_health(telemetry))
        systems.append(
            SystemHealthEvidence(
                "Camera",
                camera_supported,
                None,
                camera.get("ready"),
                camera_received,
                3.0,
                camera.get("error")
                or (
                    "Measured stereo frame receipt; recording health is separate"
                    if camera_supported
                    else "Integrated stereo capture pipeline is not configured"
                ),
            )
        )
        range_received, range_reason = None, "No range sensor is configured"
        if service.navigation is not None:
            try:
                range_received = service.navigation.ranges.latest().received_monotonic
                range_reason = "Measured per-vehicle range scan"
            except (OSError, RuntimeError, ValueError) as error:
                range_reason = str(error)
        systems.append(
            SystemHealthEvidence(
                "LiDAR",
                service.navigation is not None,
                None,
                True if range_received is not None else None,
                range_received,
                0.5,
                range_reason,
            )
        )
        recorder = ReplayRecorderHealth(
            available=recording.get("available") is True,
            path=recording.get("path"),
            bytes_written=recording.get("bytes", 0),
            error=recording.get("error"),
            phase="flight" if telemetry.get("armed") else "idle",
            link_packet_losses=(telemetry.get("communication") or {}).get("dropped_packets"),
            camera_pair_drops=camera.get("dropped_pairs"),
            # No packet sequence/camera loss counters are inferred from missing observations.
        )
        try:
            pose = service.vehicle.navigation_pose()
            pose_error = None
        except (ValueError, RuntimeError, OSError) as error:
            pose, pose_error = None, str(error)
        configuration = service.features.configuration if service.features else {}
        home = navigation.get("home_local_enu_m")
        launch = configuration.get("display_launch_enu_m")
        binding = None
        if (
            home is not None
            and launch is not None
            and navigation.get("controller_run_id") == generation
        ):
            binding = ReplayFrameBinding(str(generation), tuple(home), tuple(launch))
        mission = navigation.get("saved_mission")
        reference = None
        if mission and navigation.get("controller_run_id") == generation:
            reference = ReplayMissionReference(
                str(navigation.get("run_id")),
                mission.get("name"),
                mission.get("sha256"),
                navigation.get("state", "unknown"),
                navigation.get("outcome"),
                tuple(mission["goal_offset_enu_m"]) if mission.get("goal_offset_enu_m") else None,
                navigation.get("current_waypoint"),
                tuple(
                    ReplayWaypointEvent(
                        event["waypoint_id"],
                        event.get("simulation_timestamp_seconds"),
                        generation,
                    )
                    for event in navigation.get("waypoint_events", [])[-32:]
                    if event.get("controller_run_id") == generation
                ),
                leg=navigation.get("leg"),
            )
        scene = service.scene_source.snapshot()
        truth = next(
            (item for item in scene.get("poses", []) if item["vehicle_id"] == vehicle_id),
            None,
        )
        truth_received = scene.get("pose_received_monotonic", {}).get(vehicle_id)
        truth_fresh = truth_received is not None and 0 <= time.monotonic() - truth_received < 3
        return ReplayObservation(
            vehicle_id,
            configuration.get("label", vehicle_id),
            generation,
            scene.get("epoch", 0),
            pose,
            pose_error,
            recorder,
            ReplayAidingEvidence(**aiding_values),
            binding,
            reference,
            ScenePose(**truth) if truth and truth_fresh else None,
            tuple(systems),
            evaluation_truth_received_monotonic=truth_received if truth and truth_fresh else None,
        )
