"""Bounded loopback status for the isolated visual position source and PX4 aiding."""

import errno
import json
import math
import socket
import threading
import time
from collections import deque
from collections.abc import Callable

STATUS_PORT = 14612
VISION_FAULT_PORT = 14613
GPS_FAULT_PORT = 14614
MAX_STATUS_BYTES = 4096


def send_status(channel: socket.socket, port: int, record: dict) -> bool:
    """Offer a status sample without waiting for a receiver or storage."""
    payload = json.dumps(record, allow_nan=False, separators=(",", ":")).encode()
    if len(payload) > MAX_STATUS_BYTES:
        raise ValueError("Visual status exceeds its datagram budget")
    try:
        return channel.sendto(payload, ("127.0.0.1", port)) == len(payload)
    except OSError as error:
        if error.errno in {errno.EAGAIN, errno.ENOBUFS}:
            return False
        raise


def receive_status(channel: socket.socket) -> dict | None:
    """Read at most one bounded local datagram."""
    try:
        encoded = channel.recv(MAX_STATUS_BYTES + 1)
    except BlockingIOError:
        return None
    if len(encoded) > MAX_STATUS_BYTES:
        return None
    try:
        message = json.loads(encoded)
    except (ValueError, UnicodeError):
        return None
    return message if isinstance(message, dict) else None


class VisualAidingGuard:
    """Keep only the newest pose and controller evidence in memory."""

    def __init__(
        self,
        port: int = STATUS_PORT,
        *,
        vehicle_id: str | None = None,
        controller_generation: Callable[[], str] | None = None,
    ) -> None:
        self.vehicle_id = vehicle_id
        self.controller_generation = controller_generation
        self.generation = ""
        self.channel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.channel.bind(("127.0.0.1", port))
            self.channel.setblocking(False)
        except OSError:
            self.channel.close()
            raise
        self.port = self.channel.getsockname()[1]
        self.lock = threading.Lock()
        self.latest: dict[str, dict] = {}
        self.poses: deque[dict] = deque(maxlen=192)
        self.stopping = threading.Event()
        self.receive_error: str | None = None
        self.receiver = threading.Thread(
            target=self._receive_loop,
            name=f"visual-aiding-{vehicle_id or port}",
            daemon=True,
        )
        self.receiver.start()

    def _receive_loop(self) -> None:
        # Freshness must not depend on how frequently a panel polls the API.
        while not self.stopping.is_set():
            try:
                self.snapshot()
            except (OSError, ValueError) as error:
                self.receive_error = str(error)
                return
            self.stopping.wait(0.01)

    def close(self) -> None:
        """Release the isolated status listener."""
        self.stopping.set()
        self.receiver.join(timeout=2)
        self.channel.close()

    def snapshot(self) -> dict[str, dict]:
        """Drain a bounded burst, retaining only each source's newest evidence."""
        if self.receive_error is not None:
            raise RuntimeError(f"Visual localization status receiver failed: {self.receive_error}")
        with self.lock:
            generation = self.controller_generation() if self.controller_generation else None
            if generation is not None and generation != self.generation:
                self.latest.clear()
                self.poses.clear()
                self.generation = generation
            for _ in range(64):
                message = receive_status(self.channel)
                if message is None:
                    break
                if self.vehicle_id is not None and message.get("vehicle_id") != self.vehicle_id:
                    continue
                if generation is not None and message.get("controller_run_id") != generation:
                    continue
                kind = message.get("kind")
                stamp = message.get("monotonic")
                if (
                    type(kind) is not str
                    or kind not in {"aiding", "pose"}
                    or type(stamp) not in {int, float}
                    or not math.isfinite(stamp)
                ):
                    continue
                if stamp >= self.latest.get(kind, {}).get("monotonic", -1):
                    self.latest[kind] = message
                    if kind == "pose" and message.get("healthy") and "horizontal_ned_m" in message:
                        if self.poses and (
                            message["controller_run_id"] != self.poses[-1]["controller_run_id"]
                            or message["map_generation"] != self.poses[-1]["map_generation"]
                        ):
                            self.poses.clear()
                        if (
                            not self.poses
                            or message["timestamp_ns"] > self.poses[-1]["timestamp_ns"]
                        ):
                            self.poses.append(message)
            return {kind: dict(record) for kind, record in self.latest.items()}

    def horizontal_enu_at(
        self,
        stamp_seconds: float,
        controller_run_id: str,
        map_generation: int,
        *,
        endpoint_seconds: float = 0.04,
    ) -> tuple[float, float]:
        """Interpolate measured stereo position within one controller and visual map."""
        self.snapshot()
        target = round(stamp_seconds * 1e9)
        with self.lock:
            if not self.poses or (
                self.poses[-1]["controller_run_id"],
                self.poses[-1]["map_generation"],
            ) != (controller_run_id, map_generation):
                raise RuntimeError("Continuous stereo position history unavailable")
            before = next(
                (row for row in reversed(self.poses) if row["timestamp_ns"] <= target), None
            )
            after = next((row for row in self.poses if row["timestamp_ns"] >= target), None)
            if before is None or after is None:
                nearest = before or after
                if abs(nearest["timestamp_ns"] - target) > endpoint_seconds * 1e9:
                    raise RuntimeError("Stereo position lies outside its capture-time history")
                north, east = nearest["horizontal_ned_m"]
            else:
                gap = after["timestamp_ns"] - before["timestamp_ns"]
                if gap > 120_000_000:
                    raise RuntimeError(
                        "Stereo position history gap exceeds 0.12 simulation seconds"
                    )
                fraction = (target - before["timestamp_ns"]) / gap if gap else 0
                north, east = (
                    first + fraction * (last - first)
                    for first, last in zip(before["horizontal_ned_m"], after["horizontal_ned_m"])
                )
            if not all(math.isfinite(value) for value in (east, north)):
                raise RuntimeError("Stereo position history contains invalid metres")
            return east, north

    def __call__(self, controller_run_id: str) -> tuple[int, int]:
        """Return map generation and PX4 horizontal reset counter after health checks."""
        state, stream = self.healthy_sources(controller_run_id)
        return int(stream["map_generation"]), int(state["xy_reset_counter"])

    def healthy_sources(self, controller_run_id: str) -> tuple[dict, dict]:
        """Return one coherent aiding/pose snapshot after health checks."""
        try:
            evidence = self.snapshot()
            state, stream = evidence["aiding"], evidence["pose"]
            now = time.monotonic()
            if (
                state["controller_run_id"] != controller_run_id
                or stream["controller_run_id"] != controller_run_id
            ):
                raise RuntimeError("Visual localization evidence belongs to another controller")
            aiding_age = now - state["monotonic"]
            pose_age = now - stream["monotonic"]
            if not 0 <= aiding_age <= 0.5:
                raise RuntimeError(f"PX4 visual aiding evidence is stale ({aiding_age:.3f} s)")
            if not 0 <= pose_age <= 0.25:
                raise RuntimeError(f"Visual pose capture is stale ({pose_age:.3f} s)")
            if not (
                stream["healthy"]
                and state["vision_xy_active"]
                and state["source_samples_fresh"]
                and state["xy_valid"]
                and state["z_valid"]
                and state["capture_stamp_valid"]
            ):
                raise RuntimeError(
                    "PX4 does not have healthy synchronized horizontal visual aiding"
                )
            return state, stream
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise RuntimeError(f"Visual localization evidence unavailable: {error}") from error
