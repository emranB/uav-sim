"""Observe live stereo using independent, bounded per-vehicle estimator sessions."""

import json
import logging
import math
import os
import shutil
import signal
import subprocess
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from common.configuration.fleet import VehicleDefinition, load_fleet
from common.logging.camera_status import CameraStatus
from common.logging.stereo import StereoRecording
from common.logging.stereo_diagnostics import write_tracking_failure
from common.logging.stereo_inputs import StereoInputArchive
from integrations.vision.cadence import live_frame_rate, next_stereo_frame
from integrations.vision.geometry import StereoGeometry
from integrations.vision.mapping import camera_configuration
from integrations.vision.native import NativeStereoEstimator


def publish_snapshot(path: Path, snapshot: dict) -> None:
    """Publish one complete snapshot; readers never observe partial JSON."""
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(snapshot, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def optical_points_in_map(points: list, position: list, orientation: list) -> list:
    """Transform left optical-frame metres using estimated XYZW orientation."""
    x, y, z, w = orientation
    norm = sum(value * value for value in orientation)
    if not 0.99 < norm < 1.01:
        raise ValueError("Estimator orientation is not a unit quaternion")
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    if not points:
        return []
    return (np.asarray(points) @ rotation.T + np.asarray(position)).round(4).tolist()


def retain_tracked_geometry(current: dict, previous: dict) -> tuple[dict, dict]:
    """Keep historical geometry without turning it into a current pose estimate."""
    geometry_fields = ("trajectory", "landmarks", "surfaces")
    if current["position_valid"]:
        previous = {key: current[key] for key in geometry_fields}
        previous["timestamp_seconds"] = current["timestamp_seconds"]
        previous["map_id"] = current.get("map_id")
        previous["map_generation"] = current.get("map_generation")
        return {**current, "map_frozen": False}, previous
    if previous:
        current = {
            **current,
            **{key: previous[key] for key in geometry_fields},
            "map_frozen": True,
            "last_tracked_timestamp_seconds": previous["timestamp_seconds"],
            "display_map_id": previous["map_id"],
            "display_map_generation": previous.get("map_generation"),
        }
    return current, previous


class LiveStereoSession:
    """Own one native estimator and a single in-flight pair, without control authority."""

    def __init__(self, definition: VehicleDefinition, root: Path) -> None:
        self.definition, self.root = definition, root
        self.camera = CameraStatus(root, definition.vehicle_id)
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, name=f"vision-{definition.vehicle_id}")
        self.status_path = definition.runtime_directory / "mapping.json"

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.stopping.set()
        self.thread.join(timeout=40)
        if self.thread.is_alive():
            raise RuntimeError(f"{self.definition.vehicle_id}: estimator failed to stop")
        self.status_path.unlink(missing_ok=True)

    def report_error(self, message: str) -> None:
        self.status_path.parent.mkdir(parents=True, exist_ok=True)
        publish_snapshot(
            self.status_path,
            {
                "schema_version": 1,
                "vehicle_id": self.definition.vehicle_id,
                "controller_run_id": self.camera.snapshot().get("controller_run_id"),
                "received_wall_time": time.time(),
                "position_valid": False,
                "error": message,
            },
        )

    def run(self) -> None:
        while not self.stopping.is_set():
            try:
                status = self.camera.snapshot()
                if not status["ready"]:
                    self.report_error("Waiting for fresh stereo camera input")
                    self.stopping.wait(1)
                    continue
                self.report_error("Starting ORB-SLAM3; loading the visual vocabulary")
                self.observe(status)
            except (
                OSError,
                ValueError,
                KeyError,
                RuntimeError,
                cv2.error,
                subprocess.SubprocessError,
            ) as error:
                logging.exception("Live mapping failed for %s", self.definition.vehicle_id)
                self.report_error(str(error))
                self.stopping.wait(3)

    def observe(self, initial: dict) -> None:
        identifier = self.definition.vehicle_id
        source_directory = (self.root / initial["directory"]).resolve()
        recording = StereoRecording.open(source_directory / "report.json")
        capture_rate_hz = recording.calibration.get("rate_hz")
        if type(capture_rate_hz) is not int or not 1 <= capture_rate_hz <= 10:
            raise ValueError("Live stereo calibration requires an integer capture rate in 1..10 Hz")
        geometry = StereoGeometry(
            recording.frames[0].width,
            recording.frames[0].height,
            recording.calibration["baseline_m"],
            recording.calibration["horizontal_fov_rad"],
            backend=os.environ.get("UAV_SURFACE_BACKEND", "cpu"),
        )
        session_id = uuid.uuid4().hex
        work = self.definition.runtime_directory / "vision" / session_id
        work.mkdir(parents=True)
        logs = self.definition.output_directory(self.root) / "perception" / "mapping" / session_id
        logs.mkdir(parents=True)
        processing_rate_hz = live_frame_rate(
            capture_rate_hz, int(os.environ.get("UAV_SLAM_RATE_HZ", "5"))
        )
        # Archive sampling is deliberately slower than live capture; its frame intervals
        # must not configure the live estimator's keyframe timing.
        (work / "camera.yaml").write_text(
            camera_configuration(
                recording,
                processing_rate_hz,
                feature_count=int(os.environ.get("UAV_SLAM_FEATURES", "4000")),
                pyramid_levels=int(os.environ.get("UAV_SLAM_LEVELS", "8")),
            ),
            encoding="utf-8",
        )
        archive_limit = int(os.environ.get("UAV_MAPPING_INPUT_ARCHIVE_PAIRS", "0"))
        if not 0 <= archive_limit <= 1000:
            raise ValueError("Estimator input archive limit must be in 0..1000 pairs")
        input_archive = (
            StereoInputArchive(
                logs / "inputs",
                recording,
                archive_limit,
                processing_rate_hz,
                initial["controller_run_id"],
            )
            if archive_limit
            else None
        )
        history: deque[list] = deque(maxlen=12)
        previous_timestamp = -1
        frame_count, tracked_count = 0, 0
        tracking_loss_events = 0
        input_gap_events = 0
        previously_tracked = False
        previous_images = None
        previous_pose = None
        maximum_pose_speed_m_s = 0.0
        previous_map_revision = None
        retained_geometry: dict = {}
        last_surface_timestamp = -1
        depth_fraction = 0.0
        try:
            with NativeStereoEstimator(work, logs / "native.log") as estimator:
                while not self.stopping.is_set():
                    status = self.camera.snapshot()
                    if (status.get("controller_run_id"), status.get("directory")) != (
                        initial["controller_run_id"],
                        initial["directory"],
                    ):
                        return
                    frame = next_stereo_frame(status, previous_timestamp, processing_rate_hz)
                    if frame is None:
                        self.stopping.wait(0.025)
                        continue
                    status = {**status, **frame}
                    timestamp = status.get("simulation_timestamp_ns", -1)
                    if not status["ready"] or timestamp == previous_timestamp:
                        self.stopping.wait(0.1)
                        continue
                    if timestamp < previous_timestamp:
                        return
                    # Keep live image exchange on the shared runtime volume, not host recording storage.
                    preview_directory = (
                        self.definition.runtime_directory / "stereo-preview"
                        if status.get("runtime_previews")
                        else source_directory
                    )
                    # Immutable timestamp-named files prevent mixed left/right generations.
                    try:
                        images = [
                            (preview_directory / f"preview-{timestamp}-{side}.ppm").read_bytes()
                            for side in ("left", "right")
                        ]
                    except FileNotFoundError:
                        self.stopping.wait(0.1)
                        continue
                    if any(len(image) > 1024 * 1024 for image in images):
                        raise ValueError("Stereo image exceeds its bounded input budget")
                    started = time.monotonic()
                    snapshot = estimator.estimate(images, timestamp, self.stopping)
                    if snapshot is None:
                        return
                    estimator_seconds = time.monotonic() - started
                    archive_started = time.monotonic()
                    if input_archive is not None:
                        input_archive.append(timestamp, images)
                    input_archive_seconds = time.monotonic() - archive_started
                    surface_started = time.monotonic()
                    if (
                        previous_timestamp >= 0
                        and timestamp - previous_timestamp > 2e9 / processing_rate_hz
                    ):
                        input_gap_events += 1
                    frame_count += 1
                    tracked_count += int(snapshot["position_valid"])
                    tracking_loss_events += int(
                        previously_tracked and not snapshot["position_valid"]
                    )
                    previously_tracked = snapshot["position_valid"]
                    if snapshot["position_valid"]:
                        position = snapshot["position_map_m"]
                        if previous_pose is not None and previous_pose[2] == snapshot.get("map_id"):
                            elapsed = (timestamp - previous_pose[0]) / 1e9
                            maximum_pose_speed_m_s = max(
                                maximum_pose_speed_m_s,
                                math.dist(position, previous_pose[1]) / elapsed,
                            )
                        previous_pose = (timestamp, position, snapshot.get("map_id"))
                    else:
                        previous_pose = None
                    if snapshot.get("map_revision") != previous_map_revision:
                        history.clear()
                        last_surface_timestamp = -1
                    previous_map_revision = snapshot.get("map_revision")
                    if snapshot["position_valid"] and (
                        last_surface_timestamp < 0
                        or timestamp - last_surface_timestamp >= 500_000_000
                    ):
                        measurements = geometry.measure(*images, include_preview=False)
                        depth_fraction = measurements.valid_fraction
                        last_surface_timestamp = timestamp
                        history.append(
                            optical_points_in_map(
                                measurements.points,
                                snapshot["position_map_m"],
                                snapshot["orientation_xyzw"],
                            )
                        )
                    elif not snapshot["position_valid"]:
                        history.clear()
                        depth_fraction = 0.0
                    surfaces = [point for frame in history for point in frame]
                    # Bound dense display cost independently from the native sparse map.
                    surfaces = surfaces[:: max(1, (len(surfaces) + 5999) // 6000)]
                    snapshot.update(
                        schema_version=1,
                        vehicle_id=identifier,
                        session_id=session_id,
                        controller_run_id=initial["controller_run_id"],
                        received_wall_time=status["received_wall_time"],
                        processed_wall_time=time.time(),
                        simulation_timestamp_ns=timestamp,
                        processed_frames=frame_count,
                        tracked_frames=tracked_count,
                        tracking_loss_events=tracking_loss_events,
                        input_gap_events=input_gap_events,
                        maximum_pose_speed_m_s=maximum_pose_speed_m_s,
                        processing_seconds=time.monotonic() - started,
                        estimator_seconds=estimator_seconds,
                        input_archive_seconds=input_archive_seconds,
                        surface_seconds=time.monotonic() - surface_started,
                        input_age_seconds=max(0.0, time.time() - status["received_wall_time"]),
                        input_gap_seconds=(timestamp - previous_timestamp) / 1e9
                        if previous_timestamp >= 0
                        else 0.0,
                        frame="local left-camera optical map; metres",
                        surfaces=surfaces,
                        valid_depth_fraction=depth_fraction,
                        surface_timestamp_seconds=last_surface_timestamp / 1e9
                        if last_surface_timestamp >= 0
                        else None,
                        surface_backend=geometry.backend,
                        geometry_scope="recent measured surfaces, not a complete solid world",
                        control_authority=False,
                    )
                    snapshot, retained_geometry = retain_tracked_geometry(
                        snapshot, retained_geometry
                    )
                    publish_snapshot(self.status_path, snapshot)
                    if (
                        previous_images is not None
                        and previous_images[2]
                        and not snapshot["position_valid"]
                        and 1 <= tracking_loss_events <= 6
                        and os.environ.get("UAV_MAPPING_DIAGNOSTICS") == "true"
                    ):
                        # Save only the transition, not every frame while tracking is lost.
                        try:
                            write_tracking_failure(
                                logs, previous_images[:2], (timestamp, images), snapshot
                            )
                        except OSError:
                            logging.exception(
                                "Could not preserve tracking inputs for %s", identifier
                            )
                    previous_images = (timestamp, images, snapshot["position_valid"])
                    previous_timestamp = timestamp
        finally:
            # Preserve estimator evidence before releasing transient exchange files.
            for filename in (
                "camera.yaml",
                "observations.jsonl",
                "trajectory.tum",
                "keyframes.tum",
                "landmarks.json",
            ):
                artifact = work / filename
                if artifact.is_file():
                    shutil.copyfile(artifact, logs / filename)
            publish_snapshot(
                logs / "session.json",
                {
                    "schema_version": 1,
                    "vehicle_id": identifier,
                    "session_id": session_id,
                    "controller_run_id": initial["controller_run_id"],
                    "processed_frames": frame_count,
                    "tracked_frames": tracked_count,
                    "source_recording": initial["recording"],
                    "control_authority": False,
                },
            )
            shutil.rmtree(work)


def main() -> int:
    logging.basicConfig(level=logging.INFO)
    cv2.setNumThreads(1)
    root = Path(os.environ.get("OUTPUT_DIR", "outputs")).resolve()
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda signum, frame: stopping.set())
    signal.signal(signal.SIGINT, lambda signum, frame: stopping.set())
    sessions: dict[str, LiveStereoSession] = {}
    try:
        while not stopping.wait(1):
            definitions = {item.vehicle_id: item for item in load_fleet() if item.enabled}
            for identifier in list(sessions):
                if identifier not in definitions:
                    sessions.pop(identifier).close()
            for identifier, definition in definitions.items():
                if identifier not in sessions:
                    sessions[identifier] = LiveStereoSession(definition, root)
                    sessions[identifier].start()
    finally:
        for session in sessions.values():
            session.stopping.set()
        for session in sessions.values():
            session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
