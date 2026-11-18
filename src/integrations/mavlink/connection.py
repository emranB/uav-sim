"""One receive owner for simulated-vehicle telemetry and acknowledged commands."""

import json
import math
import socket
import struct
import threading
import time
from collections import deque
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path

from pymavlink import mavutil

from common.configuration.missions import MissionItem
from common.interfaces.navigation import NavigationPose
from common.interfaces.vehicles import VehicleRuntime
from common.logging.storage import BoundedRecording
from common.logging.telemetry import TelemetryRecording
from common.navigation.pose_history import EstimatedPoseHistory
from integrations.mavlink.impairment import SimulatedLink


class SimulatedVehicleConnection:
    """Listen only on the isolated simulator network's local MAVLink API port."""

    def __init__(
        self,
        output_directory: Path,
        runtime: VehicleRuntime | None = None,
        vehicle_id: str = "vehicle_1",
        system_id: int = 1,
        vehicle_port: int = 14540,
        application_port: int = 14542,
        heartbeat_timeout_seconds: float = 5.0,
    ) -> None:
        if (
            type(heartbeat_timeout_seconds) not in {int, float}
            or not math.isfinite(heartbeat_timeout_seconds)
            or not 1 <= heartbeat_timeout_seconds <= 30
        ):
            raise ValueError("Heartbeat timeout must be finite and in 1..30 seconds")
        self.heartbeat_timeout_seconds = heartbeat_timeout_seconds
        self.vehicle_id = vehicle_id
        self.system_id = system_id
        self.application_port = application_port
        self.link = SimulatedLink(vehicle_port, application_port)
        try:
            self.connection = self._new_connection()
        except (OSError, ValueError):
            self.link.close()
            raise
        self.runtime = runtime
        self.run_id = runtime.generation() if runtime else ""
        self.restart_pending = False
        self.condition = threading.Condition()
        self.send_lock = threading.Lock()
        self.transaction_lock = threading.Lock()
        self.navigation_history = EstimatedPoseHistory()
        self.messages: dict[str, dict] = {}
        self.received_at: dict[str, float] = {}
        self.replies: deque = deque(maxlen=128)
        self.last_heartbeat = 0.0
        self.flight_mode = "Unknown"
        self.stop_event = threading.Event()
        self.output_directory = output_directory
        self.recording: TelemetryRecording | None = None
        self.thread = threading.Thread(target=self.receive, name="mavlink-receiver", daemon=True)

    def _new_connection(self):
        connection = mavutil.mavlink_connection(
            f"udpin:127.0.0.1:{self.application_port}", source_system=245, source_component=190
        )
        try:
            # The receiver has the same short-burst requirements as its proxy.
            # Buffering is bounded and never changes measurement freshness rules.
            connection.port.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        except OSError:
            connection.close()
            raise
        return connection

    def start(self) -> None:
        self.link.start()
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=3)
        self.connection.close()
        self.link.close()

    def receive(self) -> None:
        last_sent = 0.0
        next_generation_check = 0.0
        recording = TelemetryRecording(
            self.output_directory,
            self.vehicle_id,
            self.run_id,
            writer_factory=BoundedRecording,
        )
        self.recording = recording
        try:
            while not self.stop_event.is_set():
                generation = ""
                if self.runtime and time.monotonic() >= next_generation_check:
                    generation = self.runtime.generation()
                    next_generation_check = time.monotonic() + 0.1
                if generation and generation != self.run_id:
                    with self.send_lock:
                        self.connection.close()
                        self.connection = self._new_connection()
                    self.clear_runtime_state()
                    recording.close()
                    recording = TelemetryRecording(
                        self.output_directory,
                        self.vehicle_id,
                        generation,
                        writer_factory=BoundedRecording,
                    )
                    self.recording = recording
                    with self.condition:
                        self.run_id = generation
                        self.restart_pending = False
                        self.condition.notify_all()
                message = self.connection.recv_match(blocking=True, timeout=0.2)
                now = time.monotonic()
                if message is not None and message.get_srcSystem() == self.system_id:
                    kind = message.get_type()
                    payload = message.to_dict()
                    with self.condition:
                        self.messages[kind] = payload
                        self.received_at[kind] = now
                        if kind == "HEARTBEAT":
                            self.last_heartbeat = now
                            self.flight_mode = mavutil.mode_string_v10(message)
                            # pymavlink 2.4.49 does not decode PX4 main-mode 6.
                            if (message.custom_mode >> 16) & 0xFF == 6:
                                self.flight_mode = "OFFBOARD"
                        if kind in {
                            "COMMAND_ACK",
                            "MISSION_REQUEST",
                            "MISSION_REQUEST_INT",
                            "MISSION_ACK",
                            "MISSION_COUNT",
                            "MISSION_ITEM",
                            "MISSION_ITEM_INT",
                            "PARAM_VALUE",
                        } and getattr(message, "target_system", 0) in {0, 245}:
                            self.replies.append(message)
                        if kind == "LOCAL_POSITION_NED" and "ATTITUDE" in self.messages:
                            self.navigation_history.append(
                                self._estimated_navigation_pose(
                                    payload, self.messages["ATTITUDE"], now
                                )
                            )
                        self.condition.notify_all()
                    if kind == "HEARTBEAT":
                        recording.observe_armed(
                            bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        )
                    recording.write(
                        json.dumps(
                            {
                                "schema_version": 2,
                                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                                "vehicle_id": self.vehicle_id,
                                "controller_run_id": self.run_id,
                                "recording_phase": "flight" if recording.armed else "idle",
                                "source": "px4",
                                "message": payload,
                            }
                        )
                        + "\n"
                    )
                if now - last_sent > 1 and self.last_heartbeat:
                    with self.send_lock:
                        self.connection.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0,
                            0,
                            mavutil.mavlink.MAV_STATE_ACTIVE,
                        )
                    last_sent = now
        finally:
            recording.close()

    def snapshot(self) -> dict:
        with self.condition:
            messages = {key: dict(value) for key, value in self.messages.items()}
            flight_mode = self.flight_mode
            heartbeat_age = time.monotonic() - self.last_heartbeat if self.last_heartbeat else None
            position_age = time.monotonic() - self.received_at.get("GLOBAL_POSITION_INT", 0)
            gps_age = time.monotonic() - self.received_at.get("GPS_RAW_INT", 0)
            status_age = time.monotonic() - self.received_at.get("SYS_STATUS", 0)
            restart_pending = self.restart_pending
            run_id = self.run_id
            gps_received_monotonic = self.received_at.get("GPS_RAW_INT")
            system_status_received_monotonic = self.received_at.get("SYS_STATUS")
            heartbeat_received_monotonic = self.last_heartbeat or None
        heartbeat = messages.get("HEARTBEAT", {})
        position = messages.get("GLOBAL_POSITION_INT", {})
        gps = messages.get("GPS_RAW_INT", {})
        armed = bool(heartbeat.get("base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        preflight_ready = status_age < 5 and bool(
            messages.get("SYS_STATUS", {}).get("onboard_control_sensors_health", 0)
            & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        )
        connected = (
            heartbeat_age is not None
            and heartbeat_age < self.heartbeat_timeout_seconds
            and not restart_pending
        )
        return {
            "vehicle_id": self.vehicle_id,
            "run_id": run_id,
            "restart_pending": restart_pending,
            "connected": connected,
            "flight_mode": flight_mode,
            "recording": self.recording.snapshot() if self.recording is not None else None,
            "ready": connected
            and (self.recording is None or self.recording.error is None)
            and (armed or preflight_ready)
            and gps.get("fix_type", 0) >= 3
            and "LOCAL_POSITION_NED" in messages
            and bool(position.get("lat"))
            and position_age < 3
            and gps_age < 5,
            "heartbeat_age_seconds": heartbeat_age,
            "preflight_ready": preflight_ready,
            "armed": armed,
            "latitude_deg": position.get("lat", 0) / 1e7,
            "longitude_deg": position.get("lon", 0) / 1e7,
            "relative_altitude_m": position.get("relative_alt", 0) / 1000,
            "messages": messages,
            "gps_received_monotonic": gps_received_monotonic,
            "system_status_received_monotonic": system_status_received_monotonic,
            "heartbeat_received_monotonic": heartbeat_received_monotonic,
            "heartbeat_timeout_seconds": self.heartbeat_timeout_seconds,
            "communication": self.link.snapshot(),
        }

    def configure_link(self, latency_ms: int, loss_percent: int, seed: int) -> None:
        self.link.configure(latency_ms, loss_percent, seed)

    def set_gnss_satellites(self, satellites: int) -> None:
        if type(satellites) is not int or not 0 <= satellites <= 20:
            raise ValueError("Simulated satellites must be an integer in 0..20")
        self._set_parameter("SIM_GPS_USED", satellites, integer=True)

    def read_configuration(self) -> dict:
        return {
            "cruise_speed_m_s": self._read_parameter("MPC_XY_CRUISE"),
            "climb_speed_m_s": self._read_parameter("MPC_Z_VEL_MAX_UP"),
            "gnss_satellites": self._read_parameter("SIM_GPS_USED"),
        }

    def configure_limits(self, cruise_speed_m_s: float, climb_speed_m_s: float) -> dict:
        for value, maximum in ((cruise_speed_m_s, 15), (climb_speed_m_s, 5)):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("Vehicle speeds must be numeric")
            if not 0.5 <= value <= maximum:
                raise ValueError(f"Vehicle speed must be in 0.5..{maximum} m/s")
        self._set_parameter("MPC_XY_CRUISE", cruise_speed_m_s, integer=False)
        self._set_parameter("MPC_Z_VEL_MAX_UP", climb_speed_m_s, integer=False)
        return self.read_configuration()

    def _parameter_reply(self, name: str, timeout_seconds: float) -> float | int:
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            reply = self.wait_reply({"PARAM_VALUE"}, deadline - time.monotonic())
            if reply.param_id == name:
                if reply.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                    return struct.unpack("<i", struct.pack("<f", reply.param_value))[0]
                if reply.param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
                    return reply.param_value
                raise RuntimeError(f"Unsupported parameter encoding for {name}")
        raise TimeoutError(f"No parameter response for {name}")

    def _read_parameter(self, name: str) -> float | int:
        with self.transaction_lock:
            self.wait_connected(5)
            with self.condition:
                self.replies.clear()
            with self.send_lock:
                self.connection.mav.param_request_read_send(self.system_id, 1, name.encode(), -1)
            return self._parameter_reply(name, 5)

    def _set_parameter(self, name: str, value: float | int, integer: bool) -> None:
        encoded = struct.unpack("<f", struct.pack("<i", value))[0] if integer else float(value)
        parameter_type = (
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
            if integer
            else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        with self.transaction_lock:
            self.wait_connected(5)
            for attempt in range(3):
                with self.condition:
                    self.replies.clear()
                with self.send_lock:
                    self.connection.mav.param_set_send(
                        self.system_id, 1, name.encode(), encoded, parameter_type
                    )
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    try:
                        actual = self._parameter_reply(name, deadline - time.monotonic())
                    except TimeoutError:
                        break
                    # PARAM_VALUE has no request ID; an earlier read/notification can arrive late.
                    if math.isclose(actual, value, rel_tol=1e-5, abs_tol=1e-6):
                        return
            raise TimeoutError(f"No verified PX4 parameter response for {name}")

    def clear_runtime_state(self) -> None:
        with self.condition:
            self.navigation_history.clear()
            self.messages.clear()
            self.received_at.clear()
            self.replies.clear()
            self.last_heartbeat = 0
            self.flight_mode = "Unknown"

    def wait_connected(self, timeout_seconds: float) -> None:
        """Wait for a fresh heartbeat after a paused clock or transport interruption."""
        with self.condition:
            connected = self.condition.wait_for(
                lambda: (
                    self.last_heartbeat > 0
                    and time.monotonic() - self.last_heartbeat < 5
                    and not self.restart_pending
                ),
                timeout=timeout_seconds,
            )
        if not connected:
            raise TimeoutError(
                f"{self.vehicle_id} did not reconnect within {timeout_seconds:g} seconds"
            )

    def restart(self) -> None:
        if self.runtime is None:
            raise RuntimeError("This vehicle adapter has no process-restart capability")
        with self.condition:
            self.restart_pending = True
            self.clear_runtime_state()
        try:
            self.runtime.restart()
        except OSError:
            with self.condition:
                self.restart_pending = False
            raise

    def arm(self) -> None:
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (1.0,))

    def start_mission(self) -> None:
        self.command(mavutil.mavlink.MAV_CMD_MISSION_START)

    def land(self) -> None:
        self.command(mavutil.mavlink.MAV_CMD_NAV_LAND, (0, 0, 0, math.nan, math.nan, math.nan, 0))

    def wait_reply(
        self,
        kinds: set[str],
        timeout_seconds: float,
        command: int | None = None,
        sequence: int | None = None,
    ):
        deadline = time.monotonic() + timeout_seconds
        with self.condition:
            while True:
                for message in tuple(self.replies):
                    if (
                        message.get_type() in kinds
                        and (command is None or message.command == command)
                        and (sequence is None or getattr(message, "seq", None) == sequence)
                    ):
                        self.replies.remove(message)
                        return message
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"No acknowledgement for {sorted(kinds)}")
                self.condition.wait(remaining)

    def command(self, command: int, parameters: tuple[float, ...] = ()) -> None:
        """Return only after PX4 accepts a command; completion is tracked in telemetry."""
        with self.transaction_lock:
            if not self.snapshot()["connected"]:
                raise RuntimeError("Simulated vehicle is disconnected")
            with self.condition:
                self.replies.clear()
            with self.send_lock:
                self.connection.mav.command_long_send(
                    self.system_id,
                    1,
                    command,
                    0,
                    *(parameters + (0.0,) * (7 - len(parameters))),
                )
            reply = self.wait_reply({"COMMAND_ACK"}, 10, command)
            if reply.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise RuntimeError(f"PX4 rejected command {command}: result {reply.result}")

    def upload(self, items: tuple[MissionItem, ...]) -> None:
        with self.transaction_lock:
            if not self.snapshot()["connected"]:
                raise RuntimeError("Simulated vehicle is disconnected")
            with self.condition:
                self.replies.clear()
            with self.send_lock:
                self.connection.mav.mission_count_send(self.system_id, 1, len(items))
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                reply = self.wait_reply(
                    {"MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"}, 10
                )
                if reply.get_type() == "MISSION_ACK":
                    if reply.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        raise RuntimeError(f"Mission rejected: result {reply.type}")
                    return
                if not 0 <= reply.seq < len(items):
                    raise RuntimeError("PX4 requested an invalid mission sequence")
                item = items[reply.seq]
                p = item.parameters
                with self.send_lock:
                    if reply.get_type() == "MISSION_REQUEST_INT":
                        self.connection.mav.mission_item_int_send(
                            self.system_id,
                            1,
                            reply.seq,
                            2 if item.frame == 2 else 6,
                            item.command,
                            0,
                            1,
                            *p[:4],
                            int(p[4] * 1e7),
                            int(p[5] * 1e7),
                            p[6],
                        )
                    else:
                        self.connection.mav.mission_item_send(
                            self.system_id,
                            1,
                            reply.seq,
                            2 if item.frame == 2 else 3,
                            item.command,
                            0,
                            1,
                            *p,
                        )
            raise TimeoutError("Mission upload exceeded its deadline")

    def _request_mission_reply(
        self,
        send_request: Callable[[], None],
        kinds: set[str],
        deadline: float,
        sequence: int | None = None,
    ):
        """Retry lost mission responses within one wall-clock transaction deadline."""
        for attempt in range(3):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            with self.send_lock:
                send_request()
            try:
                return self.wait_reply(kinds, min(3, remaining), sequence=sequence)
            except TimeoutError:
                if self.stop_event.is_set():
                    raise RuntimeError(f"{self.vehicle_id}: mission read cancelled")
        raise TimeoutError(f"{self.vehicle_id}: mission read timed out at sequence {sequence}")

    def download(self) -> dict:
        """Read the actual vehicle mission, including one uploaded through QGC."""
        with self.transaction_lock:
            if not self.snapshot()["connected"]:
                raise RuntimeError("Simulated vehicle is disconnected")
            with self.condition:
                self.replies.clear()
            deadline = time.monotonic() + 30
            count = self._request_mission_reply(
                lambda: self.connection.mav.mission_request_list_send(self.system_id, 1),
                {"MISSION_COUNT"},
                deadline,
            ).count
            if not 0 <= count <= 500:
                raise RuntimeError("Vehicle mission exceeds the supported item limit")
            items = []
            for sequence in range(count):
                item = self._request_mission_reply(
                    lambda: self.connection.mav.mission_request_int_send(
                        self.system_id, 1, sequence
                    ),
                    {"MISSION_ITEM_INT", "MISSION_ITEM"},
                    deadline,
                    sequence,
                )
                scale = 1e7 if item.get_type() == "MISSION_ITEM_INT" else 1
                parameters = [
                    item.param1,
                    item.param2,
                    item.param3,
                    item.param4,
                    item.x / scale,
                    item.y / scale,
                    item.z,
                ]
                parameters = [value if math.isfinite(value) else None for value in parameters]
                items.append(
                    {
                        "type": "SimpleItem",
                        "command": item.command,
                        "frame": 3 if item.frame == 6 else item.frame,
                        "autoContinue": bool(item.autocontinue),
                        "doJumpId": sequence + 1,
                        "params": parameters,
                    }
                )
            with self.send_lock:
                self.connection.mav.mission_ack_send(
                    self.system_id, 1, mavutil.mavlink.MAV_MISSION_ACCEPTED
                )
            position = self.snapshot()
            home = position["messages"].get("HOME_POSITION", {})
            global_position = position["messages"].get("GLOBAL_POSITION_INT", {})
            planned_home = [
                home.get("latitude", position["latitude_deg"] * 1e7) / 1e7,
                home.get("longitude", position["longitude_deg"] * 1e7) / 1e7,
                home.get("altitude", global_position.get("alt", 0)) / 1000,
            ]
            return {
                "fileType": "Plan",
                "version": 1,
                "groundStation": "QGroundControl",
                "mission": {
                    "version": 2,
                    "firmwareType": 12,
                    "vehicleType": 2,
                    "cruiseSpeed": 5,
                    "hoverSpeed": 3,
                    "plannedHomePosition": planned_home,
                    "items": items,
                },
                "geoFence": {"version": 2, "circles": [], "polygons": []},
                "rallyPoints": {"version": 2, "points": []},
            }

    def navigation_pose(self) -> NavigationPose:
        """Return fresh PX4 local estimates converted once from NED/FRD to ENU/FLU."""
        with self.condition:
            now = time.monotonic()
            position = dict(self.messages.get("LOCAL_POSITION_NED", {}))
            attitude = dict(self.messages.get("ATTITUDE", {}))
            stamp = min(
                self.received_at.get("LOCAL_POSITION_NED", 0), self.received_at.get("ATTITUDE", 0)
            )
        state = self.snapshot()
        if not state["connected"] or now - stamp > 1 or not position or not attitude:
            raise RuntimeError(f"{self.vehicle_id}: fresh estimated local pose unavailable")
        if self.recording is not None and self.recording.error is not None:
            raise RuntimeError(f"{self.vehicle_id}: {self.recording.error}")
        if max(abs(attitude["roll"]), abs(attitude["pitch"])) > 0.25:
            raise RuntimeError(f"{self.vehicle_id}: tilt exceeds horizontal-scan demo limit")
        return self._estimated_navigation_pose(position, attitude, stamp)

    def _estimated_navigation_pose(
        self, position: dict, attitude: dict, received_monotonic: float
    ) -> NavigationPose:
        heartbeat = self.messages.get("HEARTBEAT", {})
        return NavigationPose(
            (position["y"], position["x"], -position["z"]),
            math.pi / 2 - attitude["yaw"],
            received_monotonic,
            self.run_id,
            bool(heartbeat.get("base_mode", 0) & 128),
            self.flight_mode,
            position["time_boot_ms"] / 1000,
            attitude["roll"],
            -attitude["pitch"],
        )

    def navigation_pose_at(self, timestamp_seconds: float) -> NavigationPose:
        """Align PX4 estimates, waiting at most 0.25 wall seconds for a future bracket.

        Alignment and history-gap limits remain unchanged; delayed transport delivery
        must not cause extrapolation beyond them. The condition releases the receive
        owner while waiting. Old measurements and actual gaps fail immediately.
        """
        deadline = time.monotonic() + 0.25
        with self.condition:
            while True:
                try:
                    return self.navigation_history.at(timestamp_seconds)
                except RuntimeError:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not self.navigation_history.needs_future_sample(
                        timestamp_seconds
                    ):
                        raise
                    self.condition.wait(timeout=remaining)

    def navigation_armed(self) -> bool:
        """Observe disarm independently of range suitability or local-pose tilt."""
        state = self.snapshot()
        if not state["connected"]:
            raise RuntimeError(f"{self.vehicle_id}: no fresh heartbeat to confirm disarm")
        return state["armed"]

    def send_navigation_velocity(self, velocity_enu_m_s: tuple[float, float, float]) -> None:
        """Convert one service-bounded ENU velocity into the existing MAVLink owner."""
        if len(velocity_enu_m_s) != 3 or not all(math.isfinite(v) for v in velocity_enu_m_s):
            raise ValueError("Navigation velocity requires three finite ENU m/s")
        if math.hypot(*velocity_enu_m_s[:2]) > 1.01 or abs(velocity_enu_m_s[2]) > 1.01:
            raise ValueError("Navigation velocity exceeds the 1m/s simulation limit")
        east, north, up = velocity_enu_m_s
        with self.send_lock:
            self.connection.mav.set_position_target_local_ned_send(
                0,
                self.system_id,
                1,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                3527,
                0,
                0,
                0,
                north,
                east,
                -up,
                0,
                0,
                0,
                0,
                0,
            )

    def begin_navigation(self) -> None:
        """Acknowledge Offboard mode and arming after the caller primes setpoints."""
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, (1, 6))
        self.arm()

    def end_navigation(self) -> None:
        """Return a disarmed controller to Hold so missing Offboard proof cannot block readiness."""
        if self.snapshot()["armed"]:
            raise RuntimeError("Cannot release navigation mode while armed")
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, (1, 4, 3))
