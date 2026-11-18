"""Live calibrated optical evidence from measured panoramic stereo images only."""

import json
import logging
import os
import signal
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor, wait
from dataclasses import asdict
from functools import partial
from pathlib import Path

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

from common.configuration.fleet import VehicleDefinition, load_fleet
from common.configuration.navigation_stereo import NavigationStereoCalibration
from integrations.vision.geometry import StereoGeometry
from integrations.vision.navigation_source import write_atomic_json


class VehicleStereoObstacles:
    """Exact left/right and six-rig timestamp pairing with one latest-only processing slot."""

    def __init__(self, node: Node, definition: VehicleDefinition, backend: str) -> None:
        self.node, self.definition = node, definition
        self.calibration = NavigationStereoCalibration()
        c = self.calibration
        self.geometry = StereoGeometry(
            c.width,
            c.height,
            c.baseline_m,
            c.horizontal_fov_rad,
            backend,
            sample_stride=4,
            vertical_sample_stride=2,
        )
        # CPU matching dominates batch latency. Each rig owns its matchers;
        # two bounded matching threads overlap native work without sharing a
        # non-thread-safe SGBM instance. CUDA keeps its existing sequential path.
        self.rig_geometries = [self.geometry]
        self.match_pool = None
        if backend == "cpu":
            self.rig_geometries.extend(
                StereoGeometry(
                    c.width,
                    c.height,
                    c.baseline_m,
                    c.horizontal_fov_rad,
                    backend,
                    sample_stride=4,
                    vertical_sample_stride=2,
                )
                for _ in range(c.rig_count - 1)
            )
            self.match_pool = ThreadPoolExecutor(
                max_workers=2, thread_name_prefix=definition.vehicle_id + "-matching"
            )
        self.backend = backend
        self.output = definition.runtime_directory / "navigation-depth.json"
        self.control = definition.runtime_directory / "navigation-depth-control.json"
        self.lock = threading.Lock()
        self.pending: dict[int, tuple[float, dict[tuple[int, str], bytes]]] = {}
        self.latest = None
        self.consumed_timestamp = -1
        self.generation = ""
        self.source_session_id = uuid.uuid4().hex
        self.camera_topics: dict[tuple[int, str], str] = {}
        self.masked_side: str | None = None
        self.pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix=definition.vehicle_id + "-stereo"
        )
        self.future = None
        self.closed = False
        self.archived_batches = 0
        self.archive_limit = int(os.environ.get("UAV_NAV_STEREO_ARCHIVE_BATCHES", "0"))
        if not 0 <= self.archive_limit <= 10:
            raise ValueError("Stereo input archive limit must be in 0..10 batches")
        self.dropped_batches = 0
        self.dropped_out_of_order_batches = 0
        for rig in range(c.rig_count):
            for side in ["left", "right"]:
                topic = (
                    f"/world/default/model/{definition.model_name}/link/"
                    f"navigation_stereo_{rig}_{side}/sensor/camera/image"
                )
                if not node.subscribe(Image, topic, partial(self.receive, rig, side)):
                    raise RuntimeError(
                        f"{definition.vehicle_id}: cannot subscribe stereo rig {rig}"
                    )
                self.camera_topics[(rig, side)] = topic

    def receive(self, rig: int, side: str, message: Image) -> None:
        c = self.calibration
        if (message.width, message.height, message.step) != (c.width, c.height, c.width * 3):
            return
        if len(message.data) != c.width * c.height * 3:
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nsec
        with self.lock:
            if self.closed or side == self.masked_side or stamp <= self.consumed_timestamp:
                return
            received, batch = self.pending.setdefault(stamp, (time.monotonic(), {}))
            batch[(rig, side)] = bytes(message.data)
            if len(batch) == c.rig_count * 2:
                if self.latest is not None and stamp <= self.latest[0]:
                    # Completion order is not capture order. Never replace a
                    # newer complete batch or refresh a duplicate's receive clock.
                    self.pending.pop(stamp)
                    self.dropped_batches += 1
                    self.dropped_out_of_order_batches += 1
                    return
                if self.latest is not None:
                    self.dropped_batches += 1
                self.latest = (stamp, received, batch)
                self.pending.pop(stamp)
            while len(self.pending) > 2:
                self.pending.pop(min(self.pending))

    def synchronize_input_links(self) -> None:
        """Simulate a missing side by removing its actual transport subscriptions."""
        settings = {}
        if self.control.exists():
            controls = json.loads(self.control.read_text())
            if controls.get("controller_run_id") == self.generation:
                settings = controls["settings"]
        missing = {"missing_left": "left", "missing_right": "right"}.get(settings.get("fault"))
        if missing == self.masked_side:
            return
        with self.lock:
            previous = self.masked_side
            self.masked_side = missing
            self.pending.clear()
            self.latest = None
        for (rig, side), topic in self.camera_topics.items():
            if side == missing:
                self.node.unsubscribe(topic)
            elif side == previous:
                if not self.node.subscribe(Image, topic, partial(self.receive, rig, side)):
                    raise RuntimeError(f"Cannot restore navigation stereo input {topic}")
        self.publish_unavailable(
            f"Stereo input link missing_{missing}; transport subscriptions removed"
            if missing
            else "Waiting for complete stereo pairs after input link restoration"
        )

    def publish_unavailable(self, error: str) -> None:
        write_atomic_json(
            self.output,
            {
                "schema_version": 1,
                "vehicle_id": self.definition.vehicle_id,
                "controller_run_id": self.generation,
                "ready": False,
                "source_session_id": self.source_session_id,
                "error": error,
                "subscribed_sides": [
                    side for side in ["left", "right"] if side != self.masked_side
                ],
            },
        )

    def tick(self) -> None:
        if self.future is not None:
            if not self.future.done():
                return
            self.future.result()
            self.future = None
        generation_file = self.definition.runtime_directory / "px4.run"
        try:
            generation = generation_file.read_text().strip()
        except OSError:
            return
        if generation != self.generation:
            with self.lock:
                self.pending.clear()
                self.latest = None
                self.consumed_timestamp = -1
            self.generation = generation
            self.source_session_id = uuid.uuid4().hex
            self.publish_unavailable(
                "Waiting for six calibrated stereo pairs in new controller generation"
            )
        self.synchronize_input_links()
        if self.masked_side is not None:
            return
        with self.lock:
            batch = self.latest
            self.latest = None
            if batch is None:
                return
            self.consumed_timestamp = batch[0]
        self.future = self.pool.submit(self.process, batch)

    def process(self, batch: tuple) -> None:
        stamp, captured, images = batch
        settings = {}
        try:
            if self.control.exists():
                controls = json.loads(self.control.read_text())
                if controls.get("controller_run_id") == self.generation:
                    settings = controls["settings"]
            fault = settings.get("fault", "none")
            if fault == "stale":
                return
            if fault in {"missing_left", "missing_right"}:
                self.publish_unavailable(f"Stereo input link {fault}")
                return
            if self.archived_batches < self.archive_limit:
                directory = (
                    self.definition.output_directory(Path(os.environ.get("OUTPUT_DIR", "outputs")))
                    / "perception"
                    / "navigation-stereo"
                    / self.source_session_id
                    / "inputs"
                )
                directory.mkdir(parents=True, exist_ok=True)
                for (rig, side), pixels in images.items():
                    path = directory / f"{stamp}-{rig}-{side}.ppm"
                    path.write_bytes(
                        f"P6\n{self.calibration.width} {self.calibration.height}\n255\n".encode()
                        + pixels
                    )
                write_atomic_json(
                    directory / f"{stamp}.json",
                    {
                        "schema_version": 1,
                        "calibration": asdict(self.calibration),
                        "vehicle_id": self.definition.vehicle_id,
                        "controller_run_id": self.generation,
                        "timestamp_ns": stamp,
                        "input_quality": settings,
                    },
                )
                self.archived_batches += 1
            started = time.monotonic()
            points = []
            fractions = []
            quality_seed = (settings.get("seed", 0) + stamp // 200_000_000) % 2**31
            rng = np.random.default_rng(quality_seed)
            encoded_pairs = []
            for rig in range(self.calibration.rig_count):
                encoded = []
                for side in ["left", "right"]:
                    image = (
                        np.frombuffer(images[(rig, side)], np.uint8)
                        .reshape(self.calibration.height, self.calibration.width, 3)
                        .copy()
                    )
                    blur = settings.get("blur_sigma", 0)
                    if blur:
                        image = cv2.GaussianBlur(image, (0, 0), blur)
                    noise = settings.get("noise_sigma", 0)
                    if noise:
                        image = np.clip(
                            image.astype(np.float32) + rng.normal(0, noise, image.shape), 0, 255
                        ).astype(np.uint8)
                    if fault == "dark":
                        image[:] = 0
                    ppm = (
                        f"P6\n{self.calibration.width} {self.calibration.height}\n255\n".encode()
                        + image.tobytes()
                    )
                    encoded.append(ppm)
                encoded_pairs.append(encoded)
            if self.match_pool is None:
                measurements = [self.geometry.measure(*pair) for pair in encoded_pairs]
            else:
                futures = [
                    self.match_pool.submit(geometry.measure, *pair)
                    for geometry, pair in zip(self.rig_geometries, encoded_pairs)
                ]
                # Drain the fixed batch even on a matching failure. No later batch
                # can accumulate work behind an unfinished rig from this batch.
                wait(futures)
                measurements = [future.result() for future in futures]
            for rig, measured in enumerate(measurements):
                fractions.append(measured.valid_fraction)
                for x, y, z in measured.points:
                    if abs(y) / z > 0.4 or not 0.8 <= z <= 20:
                        continue
                    if fault == "depth_scale":
                        x, y, z = x * 1.5, y * 1.5, z * 1.5
                    if 0.8 <= z <= 20:
                        points.append([x, y, z, rig])
            if fault == "invalid_depth":
                points = []
                fractions = [0.0] * self.calibration.rig_count
            finished = time.monotonic()
            ready = len(points) >= 100 and min(fractions) >= 0.08
            write_atomic_json(
                self.output,
                {
                    "schema_version": 1,
                    "vehicle_id": self.definition.vehicle_id,
                    "controller_run_id": self.generation,
                    "source_session_id": self.source_session_id,
                    "calibration_id": self.calibration.identifier
                    if fault != "calibration"
                    else "invalid-calibration",
                    "calibration": asdict(self.calibration),
                    "ready": ready,
                    "error": None
                    if ready
                    else "Insufficient consistent stereo depth; invalid pixels remain unknown",
                    "timestamp_seconds": stamp / 1e9,
                    "captured_monotonic": captured,
                    "published_monotonic": finished,
                    "processing_seconds": finished - started,
                    "backend": self.backend,
                    "matching_workers": 2 if self.match_pool is not None else 1,
                    "point_sampling_stride_px": [
                        self.geometry.sample_stride,
                        self.geometry.vertical_sample_stride,
                    ],
                    "valid_fractions": fractions,
                    "points": points,
                    "input_quality": settings,
                    "quality_frame_seed": quality_seed,
                    "dropped_batches": self.dropped_batches,
                    "dropped_out_of_order_batches": self.dropped_out_of_order_batches,
                },
            )
        except (OSError, ValueError, RuntimeError, cv2.error) as error:
            self.publish_unavailable(str(error))

    def close(self) -> None:
        with self.lock:
            self.closed = True
            self.pending.clear()
            self.latest = None
        for topic in self.camera_topics.values():
            self.node.unsubscribe(topic)
        self.pool.shutdown(wait=True, cancel_futures=True)
        if self.match_pool is not None:
            self.match_pool.shutdown(wait=True, cancel_futures=True)


def main() -> int:
    logging.basicConfig(level=logging.INFO)
    cv2.setNumThreads(1)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    backend = os.environ.get("UAV_NAV_STEREO_BACKEND", "cpu")
    node = Node()
    vehicles = {}
    try:
        for definition in load_fleet():
            if definition.enabled:
                vehicles[definition.vehicle_id] = VehicleStereoObstacles(node, definition, backend)
        while not stop.wait(0.05):
            for vehicle in vehicles.values():
                vehicle.tick()
    except (OSError, ValueError, RuntimeError) as error:
        logging.exception("Stereo obstacle worker failed: %s", error)
        return 1
    finally:
        for vehicle in vehicles.values():
            vehicle.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
