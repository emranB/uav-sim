"""Translate only measured Gazebo scan ranges; never expose sensor world_pose."""

import math
import threading
import time
from collections import deque

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node

from common.interfaces.navigation import RangeScan
from gazebo.magnetometer import MagnetometerCompatibility


class GazeboRanges:
    """Latest scan and sixteen measured-time samples; no truth pose access."""

    def __init__(self, model_name: str) -> None:
        self.node = Node()
        self.magnetometer = MagnetometerCompatibility(self.node, model_name)
        self.topic = (
            f"/world/default/model/{model_name}/link/navigation_scan/sensor/navigation_lidar/scan"
        )
        self.lock = threading.Lock()
        self.scan: RangeScan | None = None
        self.history: deque[RangeScan] = deque(maxlen=16)
        self.enabled = True
        if not self.node.subscribe(LaserScan, self.topic, self.receive):
            raise RuntimeError(f"Cannot subscribe to navigation range sensor for {model_name}")

    def receive(self, message: LaserScan) -> None:
        if len(message.ranges) != 720 or message.vertical_count != 1:
            return
        # world_pose in this message is deliberately ignored. Planning uses PX4 estimates.
        scan = RangeScan(
            message.header.stamp.sec + message.header.stamp.nsec / 1e9,
            time.monotonic(),
            message.angle_min,
            message.angle_step,
            message.range_max,
            tuple(float(value) for value in message.ranges),
        )
        if not math.isfinite(scan.maximum_range_m) or scan.maximum_range_m <= 0:
            return
        with self.lock:
            if self.enabled:
                if self.history and scan.timestamp_seconds < self.history[-1].timestamp_seconds:
                    self.history.clear()
                self.scan = scan
                self.history.append(scan)

    def latest(self) -> RangeScan:
        with self.lock:
            scan = self.scan
            if not self.enabled:
                raise RuntimeError("Horizontal range subscription disabled")
        if scan is None or time.monotonic() - scan.received_monotonic > 1.5:
            raise RuntimeError("Horizontal range sensor is unavailable or stale")
        if sum(value >= 0.8 for value in scan.ranges_m) < 72:
            raise RuntimeError("Horizontal range sensor has insufficient valid rays")
        return scan

    def nearest(self, timestamp_seconds: float, maximum_skew_seconds: float) -> RangeScan:
        """Select fresh measured evidence near stereo time, retaining the latest health check."""
        if not math.isfinite(timestamp_seconds) or not 0 <= maximum_skew_seconds <= 0.5:
            raise ValueError("Invalid measured range alignment request")
        self.latest()
        with self.lock:
            if not self.enabled:
                raise RuntimeError("Horizontal range subscription disabled")
            candidates = [
                scan
                for scan in self.history
                if abs(scan.timestamp_seconds - timestamp_seconds) <= maximum_skew_seconds
                and time.monotonic() - scan.received_monotonic <= 1.5
            ]
        if not candidates:
            raise RuntimeError("No fresh range scan within the declared stereo time skew")
        selected = min(candidates, key=lambda scan: abs(scan.timestamp_seconds - timestamp_seconds))
        if sum(value >= 0.8 for value in selected.ranges_m) < 72:
            raise RuntimeError("Aligned horizontal range scan has insufficient valid rays")
        return selected

    def set_enabled(self, enabled: bool) -> None:
        """Simulation-only input-link interruption; leaves the magnetometer relay running."""
        if type(enabled) is not bool:
            raise ValueError("Range subscription enabled must be true or false")
        with self.lock:
            if self.enabled == enabled:
                return
            self.enabled = enabled
            self.scan = None
            self.history.clear()
        if enabled:
            if not self.node.subscribe(LaserScan, self.topic, self.receive):
                raise RuntimeError("Cannot restore navigation range subscription")
        else:
            self.node.unsubscribe(self.topic)

    def close(self) -> None:
        self.node.unsubscribe(self.topic)
        self.magnetometer.close()
