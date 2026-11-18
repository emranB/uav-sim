"""Deliver one fresh calibrated Gazebo stereo pair at a time without recording I/O."""

import threading
import time
from dataclasses import dataclass
from functools import partial

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

from common.configuration.cameras import StereoConfiguration
from common.configuration.fleet import VehicleDefinition


@dataclass(frozen=True)
class StereoPair:
    """RGB PPM images with a common simulation capture stamp and monotonic receipt time."""

    timestamp_ns: int
    received_monotonic: float
    sequence: int
    left: bytes
    right: bytes


class LatestStereoPair:
    """Match exact timestamps and discard queued work when a newer pair is complete."""

    def __init__(
        self,
        node: Node,
        vehicle: VehicleDefinition,
        configuration: StereoConfiguration,
        *,
        model_name: str | None = None,
    ) -> None:
        self.node = node
        self.width = configuration.width
        self.height = configuration.height
        self.header = f"P6\n{self.width} {self.height}\n255\n".encode()
        self.condition = threading.Condition()
        self.pending: dict[int, dict[str, bytes]] = {}
        self.latest: StereoPair | None = None
        self.completed = 0
        self.topics: list[str] = []
        try:
            for side in ("left", "right"):
                topic = (
                    f"/world/default/model/{model_name or vehicle.model_name}/link/stereo_{side}"
                    "/sensor/camera/image"
                )
                if not node.subscribe(Image, topic, partial(self.receive, side)):
                    raise RuntimeError(f"{vehicle.vehicle_id}: cannot subscribe to {side} camera")
                self.topics.append(topic)
        except RuntimeError:
            self.close()
            raise

    def close(self) -> None:
        """Unsubscribe from the selected vehicle's camera topics."""
        for topic in self.topics:
            self.node.unsubscribe(topic)

    def receive(self, side: str, image: Image) -> None:
        """Accept one calibrated image and publish a pair only after matching its peer."""
        if (image.width, image.height, image.step) != (
            self.width,
            self.height,
            3 * self.width,
        ) or len(image.data) != 3 * self.width * self.height:
            return
        stamp = image.header.stamp
        timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nsec
        with self.condition:
            if self.latest is not None and timestamp_ns <= self.latest.timestamp_ns:
                return
            pair = self.pending.setdefault(timestamp_ns, {})
            pair[side] = self.header + image.data
            if len(pair) == 2:
                self.completed += 1
                self.latest = StereoPair(
                    timestamp_ns, time.monotonic(), self.completed, pair["left"], pair["right"]
                )
                self.pending = {
                    stamp: pending
                    for stamp, pending in self.pending.items()
                    if stamp > timestamp_ns
                }
                self.condition.notify()
            while len(self.pending) > 4:
                del self.pending[min(self.pending)]

    def wait_after(self, previous_timestamp_ns: int, timeout_seconds: float) -> StereoPair:
        """Return the latest pair newer than the caller's last input or raise on timeout."""
        with self.condition:
            if not self.condition.wait_for(
                lambda: (
                    self.latest is not None and self.latest.timestamp_ns > previous_timestamp_ns
                ),
                timeout_seconds,
            ):
                raise TimeoutError("No advancing stereo image pair")
            assert self.latest is not None
            return self.latest
