"""Translate Gazebo ground-truth poses into viewer-neutral scene snapshots."""

import threading
import time
from dataclasses import asdict

from gz.msgs10.cameratrack_pb2 import CameraTrack
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node, SubscribeOptions

from common.configuration.fleet import load_fleet
from common.interfaces.scene import ScenePose


class GazeboScene:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.pose: ScenePose | None = None
        self.poses: dict[str, ScenePose] = {}
        self.pose_received_monotonic: dict[str, float] = {}
        self.model_vehicles = {
            vehicle.model_name: vehicle.vehicle_id for vehicle in load_fleet() if vehicle.enabled
        }
        self.last_received = 0.0
        self.epoch = 0
        self.camera_position: tuple | None = None
        self.camera_received_at = 0.0
        self.follow_target = ""
        self.node = Node()
        options = SubscribeOptions()
        options.msgs_per_sec = 30
        if not self.node.subscribe(Pose_V, "/world/default/pose/info", self.received, options):
            raise RuntimeError("Could not subscribe to Gazebo scene poses")
        self.node.subscribe(Pose, "/gui/camera/pose", self.received_camera, options)
        self.node.subscribe(CameraTrack, "/gui/currently_tracked", self.received_tracking, options)

    def received_camera(self, message: Pose) -> None:
        with self.lock:
            self.camera_position = (message.position.x, message.position.y, message.position.z)
            self.camera_received_at = time.monotonic()

    def received_tracking(self, message: CameraTrack) -> None:
        with self.lock:
            self.follow_target = message.follow_target.name

    def received(self, message: Pose_V) -> None:
        timestamp = message.header.stamp.sec + message.header.stamp.nsec / 1e9
        for pose in message.pose:
            with self.lock:
                vehicle_id = self.model_vehicles.get(pose.name)
            if vehicle_id is not None:
                snapshot = ScenePose(
                    vehicle_id,
                    timestamp,
                    (pose.position.x, pose.position.y, pose.position.z),
                    (
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                )
                with self.lock:
                    if self.model_vehicles.get(pose.name) != vehicle_id:
                        continue
                    if self.pose and timestamp < self.pose.simulation_time_seconds:
                        self.epoch += 1
                        self.poses.clear()
                        self.pose_received_monotonic.clear()
                    self.pose = snapshot
                    self.poses[snapshot.vehicle_id] = snapshot
                    self.last_received = time.monotonic()
                    self.pose_received_monotonic[snapshot.vehicle_id] = self.last_received

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "schema_version": 1,
                "frame": "ENU",
                "source": "gazebo-ground-truth",
                "epoch": self.epoch,
                "stale": not self.pose or time.monotonic() - self.last_received > 3,
                "poses": [asdict(pose) for pose in self.poses.values()],
                "pose_received_monotonic": dict(self.pose_received_monotonic),
                "viewer": {
                    "available": time.monotonic() - self.camera_received_at < 3,
                    "following": bool(self.follow_target),
                    "target": self.follow_target,
                    "camera_position_enu_m": self.camera_position,
                },
            }

    def begin_run(self) -> None:
        with self.lock:
            self.epoch += 1

    def register_vehicle(self, model_name: str, vehicle_id: str) -> None:
        with self.lock:
            self.model_vehicles[model_name] = vehicle_id

    def remove_vehicle(self, model_name: str, vehicle_id: str) -> None:
        with self.lock:
            self.model_vehicles.pop(model_name, None)
            self.poses.pop(vehicle_id, None)
            self.pose_received_monotonic.pop(vehicle_id, None)

    def close(self) -> None:
        for topic in ("/world/default/pose/info", "/gui/camera/pose", "/gui/currently_tracked"):
            self.node.unsubscribe(topic)
