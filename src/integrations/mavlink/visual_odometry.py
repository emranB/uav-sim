"""Publish fresh stereo body poses to a dedicated, local PX4 SITL MAVLink link."""

import socket
import threading
import time

import numpy as np
from pymavlink.dialects.v20 import common as mavlink
from pymavlink.quaternion import QuaternionBase

from common.configuration.cameras import StereoConfiguration


def rotation_from_wxyz(quaternion: tuple[float, ...]) -> np.ndarray:
    """Convert a finite unit quaternion to a rotation matrix."""
    values = np.asarray(quaternion, dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("Expected a finite WXYZ quaternion")
    magnitude = float(np.linalg.norm(values))
    if abs(magnitude - 1) > 0.01:
        raise ValueError("Expected a normalized WXYZ quaternion")
    return QuaternionBase(values / magnitude).dcm


class LocalVisualFrame:
    """Anchor the optical map to onboard attitude at the first valid, stationary pose."""

    def __init__(
        self,
        settings: StereoConfiguration,
        map_position: list,
        map_orientation: list,
        initial_ned_attitude: tuple[float, ...],
    ) -> None:
        cosine, sine = np.cos(settings.mount_pitch_rad), np.sin(settings.mount_pitch_rad)
        # The SDF camera link uses FLU and looks along +X. Convert optical
        # right/down/forward to link FLU, apply mount pitch, then convert to FRD.
        link_from_optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        flu_from_link = np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]])
        frd_from_flu = np.diag([1, -1, -1])
        self.body_from_camera = frd_from_flu @ flu_from_link @ link_from_optical
        self.body_to_camera = np.array(
            [settings.mount_forward_m, -settings.baseline_m / 2, -settings.mount_height_m]
        )
        self.ned_from_map = np.eye(3)
        initial_body, initial_map_from_body = self.body_pose(map_position, map_orientation)
        self.ned_from_map = rotation_from_wxyz(initial_ned_attitude) @ initial_map_from_body.T
        self.origin = initial_body

    def body_pose(self, position: list, orientation: list) -> tuple[np.ndarray, np.ndarray]:
        """Return body centre in optical-map metres and map-from-body rotation."""
        camera_position = np.asarray(position, dtype=float)
        if camera_position.shape != (3,) or not np.isfinite(camera_position).all():
            raise ValueError("Visual map position must contain three finite metres")
        x, y, z, w = orientation
        map_from_body = rotation_from_wxyz((w, x, y, z)) @ self.body_from_camera.T
        return camera_position - map_from_body @ self.body_to_camera, map_from_body

    def to_ned(self, position: list, orientation: list) -> tuple[np.ndarray, np.ndarray]:
        """Return start-relative NED metres and the FRD-to-NED WXYZ quaternion."""
        body_position, map_from_body = self.body_pose(position, orientation)
        return (
            self.ned_from_map @ (body_position - self.origin),
            QuaternionBase(self.ned_from_map @ map_from_body).q,
        )


class VisualOdometryLink:
    """SITL sensor link with a disarmed-only default and no flight commands."""

    def __init__(
        self,
        system_id: int,
        *,
        allow_armed: bool = False,
        controller_port: int = 14610,
        publisher_port: int = 14611,
    ) -> None:
        self.system_id = system_id
        self.allow_armed = allow_armed
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(("127.0.0.1", publisher_port))
            self.socket.connect(("127.0.0.1", controller_port))
            self.socket.setblocking(False)
        except OSError:
            self.socket.close()
            raise
        self.protocol = mavlink.MAVLink(self, srcSystem=245, srcComponent=197)
        self.protocol.robust_parsing = True
        self.lock = threading.Lock()
        self.timesync_replies = 0
        self.attitude: tuple[float, ...] | None = None
        self.attitude_boot_ms: int | None = None
        self.attitude_received = 0.0
        self.disarmed = False
        self.heartbeat_received = 0.0
        self.last_timestamp_ns = -1

    def close(self) -> None:
        """Close the dedicated simulator UDP socket."""
        with self.lock:
            self.socket.close()

    def write(self, packet: bytes) -> int:
        """Send one MAVLink datagram to the dedicated local PX4 port."""
        return self.socket.send(packet)

    def poll(self, simulation_now_ns: int) -> None:
        """Read fresh onboard attitude and answer timesync with the SITL clock."""
        with self.lock:
            self._poll(simulation_now_ns)

    def _poll(self, simulation_now_ns: int) -> None:
        for _ in range(64):
            try:
                packet = self.socket.recv(4096)
            except BlockingIOError:
                break
            for message in self.protocol.parse_buffer(packet) or ():
                if message.get_srcSystem() != self.system_id or message.get_srcComponent() != 1:
                    continue
                if message.get_type() == "HEARTBEAT":
                    self.disarmed = not bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.heartbeat_received = time.monotonic()
                elif message.get_type() == "ATTITUDE_QUATERNION":
                    self.attitude = (message.q1, message.q2, message.q3, message.q4)
                    self.attitude_boot_ms = message.time_boot_ms
                    self.attitude_received = time.monotonic()
                elif message.get_type() == "TIMESYNC" and message.tc1 == 0:
                    # Drop a delayed request rather than skew the PX4 clock mapping.
                    if abs(message.ts1 - simulation_now_ns) <= 50_000_000:
                        self.protocol.timesync_send(simulation_now_ns, message.ts1)
                        self.timesync_replies += 1

    def publish(
        self,
        timestamp_ns: int,
        simulation_now_ns: int,
        position: np.ndarray,
        orientation: np.ndarray,
    ) -> None:
        """Send one distinct capture, after checking vehicle, timestamp and disarm."""
        # The heartbeat is 1 Hz in simulation time. The qualified profile
        # runs at 0.50 real-time factor, so allow wall-clock delivery jitter.
        if (not self.disarmed and not self.allow_armed) or (
            time.monotonic() - self.heartbeat_received > 4
        ):
            raise RuntimeError("SITL visual odometry requires a fresh permitted heartbeat")
        if (
            self.attitude_boot_ms is None
            or abs(simulation_now_ns / 1e6 - self.attitude_boot_ms) > 100
            or not 0 <= simulation_now_ns - timestamp_ns <= 150_000_000
            or timestamp_ns <= self.last_timestamp_ns
        ):
            raise RuntimeError("Visual capture timestamp or PX4 SITL clock is inconsistent")
        # The disarmed trial enables horizontal position only. Velocity and
        # angle covariance are unknown, with position uncertainty set in EKF2.
        unknown = [float("nan")] + [0.0] * 20
        with self.lock:
            self.protocol.odometry_send(
                timestamp_ns // 1000,
                mavlink.MAV_FRAME_LOCAL_NED,
                mavlink.MAV_FRAME_BODY_FRD,
                *position,
                orientation,
                *([float("nan")] * 6),
                unknown,
                unknown,
                estimator_type=mavlink.MAV_ESTIMATOR_TYPE_VISION,
            )
        self.last_timestamp_ns = timestamp_ns
