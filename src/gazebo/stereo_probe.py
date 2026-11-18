"""Verify rendered stereo images in an isolated scene without PX4 or ROS."""

import argparse
import json
import os
import signal
import subprocess
import threading
import time
import uuid
from collections import deque
from functools import partial
from pathlib import Path
from typing import TYPE_CHECKING

from gazebo.perception_scene import write_stereo_scene

if TYPE_CHECKING:
    from gz.msgs10.image_pb2 import Image


class StereoCapture:
    """Retain bounded RGB pairs matched by exact simulation timestamp."""

    def __init__(self, destination: Path, pair_count: int) -> None:
        self.destination = destination
        self.pair_count = pair_count
        self.pending: dict[int, dict] = {}
        self.pairs: list[dict] = []
        self.lock = threading.Lock()
        self.complete = threading.Event()
        self.error: str | None = None

    def receive(self, side: str, message: "Image") -> None:
        """Validate RGB layout and write bounded evidence, not a navigation input."""
        with self.lock:
            if self.complete.is_set():
                return
            try:
                width, height = message.width, message.height
                if not (0 < width <= 4096 and 0 < height <= 4096):
                    raise ValueError("Rendered image dimensions are invalid")
                if message.step != width * 3 or len(message.data) != width * height * 3:
                    raise ValueError("Expected tightly packed RGB8 image data")
                timestamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nsec
                pair = self.pending.setdefault(timestamp_ns, {})
                pair[side] = (width, height, bytes(message.data))
                if len(pair) == 2:
                    received_monotonic = time.monotonic()
                    if self.pairs and timestamp_ns <= self.pairs[-1]["simulation_timestamp_ns"]:
                        raise ValueError("Capture clock moved backwards or repeated a timestamp")
                    if pair["left"][:2] != pair["right"][:2]:
                        raise ValueError("Stereo image dimensions differ")
                    index = len(self.pairs)
                    for name, (width, height, pixels) in pair.items():
                        path = self.destination / f"{index:04d}-{name}.ppm"
                        path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels)
                    self.pairs.append(
                        {
                            "simulation_timestamp_ns": timestamp_ns,
                            "received_monotonic": received_monotonic,
                            "left": f"{index:04d}-left.ppm",
                            "right": f"{index:04d}-right.ppm",
                            "width": width,
                            "height": height,
                            "different_views": pair["left"][2] != pair["right"][2],
                        }
                    )
                    del self.pending[timestamp_ns]
                    if len(self.pairs) >= self.pair_count:
                        self.complete.set()
                while len(self.pending) > 16:
                    del self.pending[next(iter(self.pending))]
            except (ValueError, OSError) as error:
                self.error = str(error)
                self.complete.set()


def run_probe(
    destination: Path,
    timeout_seconds: float,
    pair_count: int,
    mapping_sequence: bool = False,
    rate_hz: int | None = None,
) -> dict:
    """Start and stop an owned server; never attach to the live flight world."""
    if not 1 <= pair_count <= 200 or not 1 <= timeout_seconds <= 120:
        raise ValueError("Use 1..200 pairs and a timeout of 1..120 seconds")
    if rate_hz is not None and not 1 <= rate_hz <= 50:
        raise ValueError("Use a camera rate of 1..50 simulation Hz")
    destination.mkdir(parents=True, exist_ok=False)
    partition = f"stereo-probe-{uuid.uuid4().hex}"
    # Transport reads partition settings when its Node is constructed.
    os.environ["GZ_PARTITION"] = partition
    os.environ["GZ_IP"] = "127.0.0.1"
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    world = write_stereo_scene(
        destination / "scene.sdf", detailed=mapping_sequence, rate_hz=rate_hz
    )
    capture = StereoCapture(destination, pair_count)
    node = Node()
    truth = deque(maxlen=10000)
    truth_lock = threading.Lock()

    def receive_truth(message: Pose_V) -> None:
        timestamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nsec
        for pose in message.pose:
            if pose.name == "inspection_rig":
                with truth_lock:
                    truth.append(
                        (timestamp_ns, [pose.position.x, pose.position.y, pose.position.z])
                    )

    topics = []
    process = None
    started = time.monotonic()
    last_move = -1
    try:
        if mapping_sequence:
            topic = "/world/perception_probe/pose/info"
            if not node.subscribe(Pose_V, topic, receive_truth):
                raise RuntimeError("Cannot subscribe to test-camera evaluation truth")
            topics.append(topic)
        for side in ("left", "right"):
            topic = f"/perception_probe/{side}"
            callback = partial(capture.receive, side)
            if not node.subscribe(Image, topic, callback):
                raise RuntimeError(f"Cannot subscribe to {topic}")
            topics.append(topic)
        environment = dict(os.environ)
        environment["LIBGL_ALWAYS_SOFTWARE"] = "true"
        with (destination / "gazebo.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                ["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "2", str(world)],
                stdout=log,
                stderr=subprocess.STDOUT,
                env=environment,
                start_new_session=True,
            )
            while not capture.complete.wait(0.2):
                if process.poll() is not None:
                    raise RuntimeError(f"Rendering server exited with code {process.returncode}")
                if time.monotonic() - started > timeout_seconds:
                    raise TimeoutError("No complete stereo capture before the rendering deadline")
                if mapping_sequence:
                    with capture.lock:
                        count = len(capture.pairs)
                    if count > 0 and count // 3 > last_move:
                        # Move only the isolated test-camera rig, never a vehicle model.
                        y_m = 0.02 * min(count, 80)
                        response = subprocess.run(
                            [
                                "gz",
                                "service",
                                "-s",
                                "/world/perception_probe/set_pose",
                                "--reqtype",
                                "gz.msgs.Pose",
                                "--reptype",
                                "gz.msgs.Boolean",
                                "--timeout",
                                "1500",
                                "--req",
                                f'name: "inspection_rig" position {{ y: {y_m} }} orientation {{ w: 1 }}',
                            ],
                            capture_output=True,
                            text=True,
                            timeout=3,
                            check=False,
                        )
                        if response.returncode or "data: true" not in response.stdout:
                            raise RuntimeError("Test-camera rig movement was not acknowledged")
                        last_move = count // 3
            if capture.error:
                raise RuntimeError(capture.error)
            if not all(pair["different_views"] for pair in capture.pairs):
                raise RuntimeError("Stereo views are identical; depth evidence is insufficient")
            report = {"passed": True}
    except (OSError, RuntimeError, TimeoutError) as error:
        report = {"passed": False, "error": str(error)}
    finally:
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
        for topic in topics:
            node.unsubscribe(topic)
    report.update(
        {
            "schema_version": 1,
            "source": "gazebo-moving-camera-probe"
            if mapping_sequence
            else "gazebo-stationary-stereo-probe",
            "partition": partition,
            "elapsed_wall_seconds": time.monotonic() - started,
            "pairs": capture.pairs,
            "requested_pairs": pair_count,
            "requested_rate_hz": rate_hz or (10 if mapping_sequence else 5),
            "scope": "rendering-only; no vehicle estimation or flight-control integration",
            "calibration": {
                "width": 640 if mapping_sequence else 320,
                "height": 480 if mapping_sequence else 240,
                "horizontal_fov_rad": 1.0,
                "baseline_m": 0.16,
                "distortion": "none",
                "camera_convention": "Gazebo sensor +X forward, +Y left, +Z up",
            },
        }
    )
    if mapping_sequence:
        with truth_lock:
            history = list(truth)
        for pair in capture.pairs:
            stamp = pair["simulation_timestamp_ns"]
            nearest = min(history, key=lambda item: abs(item[0] - stamp)) if history else None
            if nearest and abs(nearest[0] - stamp) <= 50_000_000:
                pair["evaluation_rig_position_enu_m"] = nearest[1]
    (destination / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-seconds", type=float, default=45)
    parser.add_argument("--pairs", type=int, default=5)
    parser.add_argument("--mapping-sequence", action="store_true")
    parser.add_argument("--rate-hz", type=int)
    arguments = parser.parse_args()
    report = run_probe(
        arguments.output,
        arguments.timeout_seconds,
        arguments.pairs,
        arguments.mapping_sequence,
        arguments.rate_hz,
    )
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
