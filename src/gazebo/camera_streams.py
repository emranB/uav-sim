"""Observe per-vehicle rendered cameras with bounded recording and explicit freshness."""

import argparse
import json
import logging
import os
import signal
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from functools import partial
from pathlib import Path

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

from common.configuration.cameras import StereoConfiguration
from common.configuration.fleet import VehicleDefinition, load_fleet
from common.logging.camera_status import CameraStatus
from common.logging.storage import ensure_capacity, write_atomic
from common.logging.structured import recording_handler


class CameraRecordingClock:
    """Publish simulation time and its reset epoch as one immutable snapshot."""

    def __init__(self) -> None:
        self._snapshot: tuple[int, int] | None = None

    def observe(self, timestamp_ns: int) -> None:
        previous = self._snapshot
        epoch = previous[0] if previous else 0
        if previous is not None and timestamp_ns < previous[1]:
            epoch += 1
        self._snapshot = epoch, timestamp_ns

    def snapshot(self) -> tuple[int, int] | None:
        """Return (clock epoch, simulation nanoseconds), or None before the first clock."""
        return self._snapshot


class VehicleCameras:
    """Own one vehicle's subscriptions and a bounded ordered pair handoff."""

    def __init__(
        self,
        node: Node,
        definition: VehicleDefinition,
        root: Path,
        minimum_timestamp_ns: int = 0,
        controller_run_id: str | None = None,
    ) -> None:
        self.node, self.definition, self.root = node, definition, root
        self.configuration = StereoConfiguration.configured()
        self.recording_limit = int(os.environ.get("UAV_RECORD_LIMIT_PAIRS", "1"))
        if not 1 <= self.recording_limit <= 1000:
            raise ValueError("Camera recording limit must be in 1..1000 pairs")
        self.lock = threading.Lock()
        self.pending: dict[int, dict[str, bytes]] = {}
        self.completed_pairs: dict[int, tuple[dict[str, bytes], float]] = {}
        self.dropped_pairs = 0
        self.pairs: list[dict] = []
        self.archive = ThreadPoolExecutor(max_workers=1, thread_name_prefix="stereo-archive")
        self.archive_future: Future | None = None
        self.preview_timestamps: list[int] = []
        self.available_frames: list[dict] = []
        self.last_timestamp_ns = -1
        self.minimum_timestamp_ns = minimum_timestamp_ns
        self.controller_run_id = controller_run_id
        self.received_wall_time = 0.0
        self.error = "Waiting for rendered stereo images"
        self.directory = definition.output_directory(root) / "perception" / f"run-{time.time_ns()}"
        ensure_capacity(
            root,
            self.recording_limit
            * 2
            * (self.configuration.width * self.configuration.height * 3 + 64),
        )
        self.directory.mkdir(parents=True)
        self.status_path = definition.runtime_directory / "stereo.json"
        self.status_path.parent.mkdir(parents=True, exist_ok=True)
        self.preview_directory = self.status_path.parent / "stereo-preview"
        self.preview_directory.mkdir(exist_ok=True)
        self.topics = []
        try:
            for side in ("left", "right"):
                topic = (
                    f"/world/default/model/{definition.model_name}/link/stereo_{side}"
                    "/sensor/camera/image"
                )
                if not node.subscribe(Image, topic, partial(self.receive, side)):
                    raise RuntimeError(f"Cannot subscribe to {definition.vehicle_id} {side} camera")
                self.topics.append(topic)
        except RuntimeError:
            self.close()
            raise

    def receive(self, side: str, message: Image) -> None:
        width, height = self.configuration.width, self.configuration.height
        if (message.width, message.height, message.step) != (width, height, width * 3):
            with self.lock:
                self.error = "Unexpected stereo image geometry"
            return
        if len(message.data) != width * height * 3:
            with self.lock:
                self.error = "Invalid stereo RGB buffer"
            return
        timestamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nsec
        if timestamp_ns < self.minimum_timestamp_ns:
            return
        with self.lock:
            if timestamp_ns <= self.last_timestamp_ns or timestamp_ns in self.completed_pairs:
                return
            pair = self.pending.setdefault(timestamp_ns, {})
            pair[side] = bytes(message.data)
            if len(pair) == 2:
                self.completed_pairs[timestamp_ns] = (pair, time.time())
                del self.pending[timestamp_ns]
                # Preserve short delivery bursts without unbounded latency or memory.
                while len(self.completed_pairs) > 12:
                    del self.completed_pairs[min(self.completed_pairs)]
                    self.dropped_pairs += 1
            while len(self.pending) > 8:
                del self.pending[next(iter(self.pending))]
                self.dropped_pairs += 1

    def flush(self) -> None:
        with self.lock:
            completed, self.completed_pairs = self.completed_pairs, {}
        for timestamp_ns, (images, received_wall_time) in sorted(completed.items()):
            if timestamp_ns > self.last_timestamp_ns:
                if self.archive_future is not None and self.archive_future.done():
                    self.archive_future.result()
                    self.archive_future = None
                index = len(self.pairs)
                record_pair = (
                    self.archive_future is None
                    and index < self.recording_limit
                    and (
                        not self.pairs
                        or timestamp_ns - self.pairs[-1]["simulation_timestamp_ns"] >= 500_000_000
                    )
                )
                encoded_images = {}
                for side, pixels in images.items():
                    contents = (
                        f"P6\n{self.configuration.width} {self.configuration.height}\n255\n".encode()
                        + pixels
                    )
                    timestamp_path = self.preview_directory / f"preview-{timestamp_ns}-{side}.ppm"
                    write_atomic(timestamp_path, contents)
                    # Both names live on the runtime volume. Link the immutable image
                    # instead of writing its full pixel buffer a second time.
                    latest_path = self.preview_directory / f"latest-{side}.ppm"
                    temporary = latest_path.with_suffix(".tmp")
                    temporary.unlink(missing_ok=True)
                    os.link(timestamp_path, temporary)
                    temporary.replace(latest_path)
                    encoded_images[side] = contents
                if record_pair:
                    self.pairs.append(
                        {
                            "simulation_timestamp_ns": timestamp_ns,
                            "left": f"{index:04d}-left.ppm",
                            "right": f"{index:04d}-right.ppm",
                            "width": self.configuration.width,
                            "height": self.configuration.height,
                        }
                    )
                self.last_timestamp_ns = timestamp_ns
                self.preview_timestamps.append(timestamp_ns)
                self.available_frames.append(
                    {
                        "simulation_timestamp_ns": timestamp_ns,
                        "received_wall_time": received_wall_time,
                    }
                )
                while len(self.preview_timestamps) > 12:
                    self.available_frames.pop(0)
                    retired = self.preview_timestamps.pop(0)
                    for side in ("left", "right"):
                        (self.preview_directory / f"preview-{retired}-{side}.ppm").unlink(
                            missing_ok=True
                        )
                self.received_wall_time = received_wall_time
                self.error = ""
                if record_pair:
                    manifest = json.dumps(
                        {
                            "schema_version": 1,
                            "source": "gazebo-vehicle-stereo",
                            "vehicle_id": self.definition.vehicle_id,
                            "controller_run_id": self.controller_run_id,
                            "frame": f"{self.definition.model_name}/stereo_left",
                            "pairs": self.pairs,
                            "recording_limit_pairs": self.recording_limit,
                            "recording_rate_limit_hz": 2,
                            "recording_full": len(self.pairs) >= self.recording_limit,
                            "calibration": {
                                "width": self.configuration.width,
                                "height": self.configuration.height,
                                "horizontal_fov_rad": self.configuration.horizontal_fov_rad,
                                "baseline_m": self.configuration.baseline_m,
                                "distortion": "none",
                                "rate_hz": self.configuration.rate_hz,
                                "mount_pitch_rad": self.configuration.mount_pitch_rad,
                                "mount_height_m": self.configuration.mount_height_m,
                                "camera_convention": "Gazebo sensor +X forward, +Y left, +Z up",
                            },
                        }
                    ).encode()
                    self.archive_future = self.archive.submit(
                        self.persist_pair, index, encoded_images, manifest
                    )
        write_atomic(
            self.status_path,
            json.dumps(
                {
                    "schema_version": 1,
                    "vehicle_id": self.definition.vehicle_id,
                    "controller_run_id": self.controller_run_id,
                    "received_wall_time": self.received_wall_time,
                    "simulation_timestamp_ns": self.last_timestamp_ns,
                    "directory": self.directory.relative_to(self.root).as_posix(),
                    "recorded_pairs": len(self.pairs),
                    "runtime_previews": True,
                    "available_frames": self.available_frames,
                    "recording_full": len(self.pairs) >= self.recording_limit,
                    "dropped_pairs": self.dropped_pairs,
                    "error": self.error,
                }
            ).encode(),
        )

    def persist_pair(self, index: int, images: dict[str, bytes], manifest: bytes) -> None:
        """Archive one bounded pair without delaying live frame publication."""
        ensure_capacity(self.directory, sum(map(len, images.values())) + len(manifest))
        for side, contents in images.items():
            write_atomic(self.directory / f"{index:04d}-{side}.ppm", contents)
        write_atomic(self.directory / "report.json", manifest)

    def close(self) -> None:
        for topic in self.topics:
            self.node.unsubscribe(topic)
        self.topics.clear()
        try:
            if self.archive_future is not None:
                self.archive_future.result(timeout=10)
        finally:
            self.archive.shutdown(wait=False, cancel_futures=True)
            self.status_path.unlink(missing_ok=True)
            for path in self.preview_directory.glob("*.ppm"):
                path.unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--healthcheck", action="store_true")
    arguments = parser.parse_args()
    root = Path(os.environ.get("OUTPUT_DIR", "outputs"))
    if arguments.healthcheck:
        try:
            for definition in load_fleet():
                if not definition.enabled:
                    continue
                if not CameraStatus(root, definition.vehicle_id).snapshot()["ready"]:
                    return 1
            return 0
        except (OSError, ValueError, KeyError, TypeError):
            return 1
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s source=camera-streams %(message)s",
        handlers=[logging.StreamHandler(), recording_handler(root / "logs", "camera-streams")],
    )
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda signum, frame: stopping.set())
    signal.signal(signal.SIGINT, lambda signum, frame: stopping.set())
    node = Node()
    recording_clock = CameraRecordingClock()

    def received_clock(message: Clock) -> None:
        recording_clock.observe(message.sim.sec * 1_000_000_000 + message.sim.nsec)

    cameras: dict[str, VehicleCameras] = {}
    generations: dict[str, tuple[str, int]] = {}
    next_inventory = 0.0
    try:
        if not node.subscribe(Clock, "/world/default/clock", received_clock):
            raise RuntimeError("Cannot subscribe to camera recording epoch clock")
        while not stopping.wait(0.1):
            if time.monotonic() >= next_inventory:
                definitions = {item.vehicle_id: item for item in load_fleet() if item.enabled}
                for identifier in list(cameras):
                    if identifier not in definitions:
                        cameras.pop(identifier).close()
                        generations.pop(identifier, None)
                clock = recording_clock.snapshot()
                for identifier, definition in definitions.items():
                    run_file = definition.runtime_directory / "px4.run"
                    if not run_file.exists() or clock is None:
                        continue
                    generation = run_file.read_text(encoding="utf-8").strip()
                    source_generation = generation, clock[0]
                    if identifier not in cameras or generations[identifier] != source_generation:
                        if identifier in cameras:
                            cameras.pop(identifier).close()
                        cameras[identifier] = VehicleCameras(
                            node, definition, root, clock[1], generation
                        )
                        generations[identifier] = source_generation
                        logging.info(
                            "Observing rendered stereo images; recording=%s",
                            cameras[identifier].directory,
                            extra={"vehicle_id": identifier},
                        )
                next_inventory = time.monotonic() + 1
            for camera in cameras.values():
                camera.flush()
    except (OSError, ValueError, RuntimeError) as error:
        logging.exception("Camera observation service failed: %s", error)
        return 1
    finally:
        for camera in cameras.values():
            camera.close()
        node.unsubscribe("/world/default/clock")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
