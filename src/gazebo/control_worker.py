"""A dedicated persistent control connection, isolated from Python scene callbacks."""

import logging
import math
import time
from multiprocessing.connection import Connection
from typing import TYPE_CHECKING

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node

from gazebo.camera import CAMERA_OFFSETS

if TYPE_CHECKING:
    from google.protobuf.message import Message


def execute_control(node: Node, operation: str, values: dict | None = None) -> None:
    if operation == "camera":
        execute_camera(node, values or {})
        return
    if operation == "camera_follow":
        values = values or {}
        if not values.get("follow", True):
            send_request(node, "/gui/follow", StringMsg(data=""))
            return
        offset = values["offset"]
        send_request(node, "/gui/follow", StringMsg(data=values.get("model_name", "x500_0")))
        send_request(node, "/gui/follow/offset", Vector3d(x=offset[0], y=offset[1], z=offset[2]))
        return
    request: Message
    if operation in {"resume", "pause"}:
        service = "/world/default/control"
        request = WorldControl(pause=operation == "pause")
    elif operation == "restore_pose":
        service = "/world/default/set_pose"
        values = values or {}
        request = Pose(name=values.get("model_name", "x500_0"))
        position = values.get("position_enu_m", (0, 0, 0.1))
        request.position.x, request.position.y, request.position.z = position
        request.orientation.w = 1
    elif operation == "remove_model":
        service = "/world/default/remove"
        request = Entity(name=values["model_name"], type=Entity.MODEL)
    else:
        raise ValueError("Unknown Gazebo control operation")
    send_request(node, service, request)


def send_request(node: Node, service: str, request: "Message") -> None:
    for attempt in range(2):
        acknowledged, response = node.request(service, request, type(request), Boolean, 2500)
        if acknowledged:
            if not response.data:
                raise RuntimeError(f"Gazebo rejected {service}")
            return
        logging.warning("Gazebo acknowledgement timed out for %s, attempt %s", service, attempt + 1)
        time.sleep(0.05)
    raise TimeoutError(f"Gazebo did not acknowledge {service} after two attempts")


def execute_camera(node: Node, values: dict) -> None:
    offset = values.get("offset_enu_m") or CAMERA_OFFSETS[values["preset"]]
    position = values["position_enu_m"]
    send_request(node, "/gui/follow", StringMsg(data=""))
    # Let the render thread detach its previous follow target before moving.
    time.sleep(0.2)
    camera = GUICamera()
    camera.pose.position.x = position[0] + offset[0]
    camera.pose.position.y = position[1] + offset[1]
    camera.pose.position.z = position[2] + offset[2]
    yaw = math.atan2(-offset[1], -offset[0])
    pitch = math.atan2(offset[2], math.hypot(offset[0], offset[1]))
    camera.pose.orientation.x = -math.sin(pitch / 2) * math.sin(yaw / 2)
    camera.pose.orientation.y = math.sin(pitch / 2) * math.cos(yaw / 2)
    camera.pose.orientation.z = math.cos(pitch / 2) * math.sin(yaw / 2)
    camera.pose.orientation.w = math.cos(pitch / 2) * math.cos(yaw / 2)
    send_request(node, "/gui/move_to/pose", camera)


def run_worker(connection: Connection) -> None:
    # No Python subscriptions share this receiver. A scene callback waiting for the
    # interpreter must not delay service-response delivery on the same transport thread.
    node = Node()
    with connection:
        while True:
            try:
                operation = connection.recv()
            except EOFError:
                return
            if operation is None:
                return
            try:
                if isinstance(operation, dict):
                    execute_control(node, operation["operation"], operation["values"])
                else:
                    execute_control(node, operation)
                connection.send({"accepted": True})
            except (RuntimeError, ValueError, TimeoutError) as error:
                connection.send({"accepted": False, "error": str(error)})
