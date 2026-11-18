"""Bounded file adapter for measured stereo evidence and explicit simulation input faults."""

import json
import math
import threading
import time
from pathlib import Path

from common.configuration.navigation_stereo import NavigationStereoCalibration
from common.interfaces.obstacles import StaleObservationError, StereoObstacleFrame
from common.logging.storage import write_atomic


def write_atomic_json(path: Path, payload: dict) -> None:
    """Publish one complete latest record, never a partially replaced measurement."""
    write_atomic(path, json.dumps(payload, allow_nan=False).encode())


class FileStereoObstacles:
    """Read calibrated optical measurements only, not scene definitions or truth poses."""

    def __init__(self, vehicle_id: str, runtime_directory: Path) -> None:
        self.vehicle_id = vehicle_id
        self.path = runtime_directory / "navigation-depth.json"
        self.control_path = runtime_directory / "navigation-depth-control.json"
        self.calibration = NavigationStereoCalibration()
        self._cache_lock = threading.RLock()
        self._cached_payload = None
        self._cached_key = None
        self._cached_frame = None

    def latest(self, controller_run_id: str) -> StereoObstacleFrame:
        with self._cache_lock:
            try:
                return self._read_latest(controller_run_id)
            except (KeyError, TypeError, IndexError, AttributeError) as error:
                raise RuntimeError(
                    f"{self.vehicle_id}: malformed stereo obstacle record"
                ) from error

    def _read_latest(self, controller_run_id: str) -> StereoObstacleFrame:
        try:
            stat = self.path.stat()
            key = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
            if stat.st_size > 2_000_000:
                raise RuntimeError("Stereo obstacle output exceeds its bounded schema")
            if key != self._cached_key:
                self._cached_payload = json.loads(self.path.read_text())
                self._cached_key = key
                self._cached_frame = None
            payload = self._cached_payload
        except (OSError, ValueError) as error:
            raise RuntimeError(
                f"{self.vehicle_id}: stereo obstacle output unavailable: {error}"
            ) from error
        if (
            payload.get("schema_version") != 1
            or payload.get("vehicle_id") != self.vehicle_id
            or payload.get("controller_run_id") != controller_run_id
        ):
            raise RuntimeError(f"{self.vehicle_id}: stereo source identity/generation mismatch")
        if not payload.get("ready"):
            raise RuntimeError(f"{self.vehicle_id}: stereo unavailable: {payload.get('error')}")
        if payload.get("calibration_id") != self.calibration.identifier:
            raise RuntimeError(f"{self.vehicle_id}: stereo calibration mismatch")
        now = time.monotonic()
        captured, published = payload["captured_monotonic"], payload["published_monotonic"]
        if not all(type(v) in {int, float} and math.isfinite(v) for v in (captured, published)):
            raise RuntimeError("Invalid stereo freshness metadata")
        if captured > now or not captured <= published <= now + 0.01:
            raise RuntimeError(f"{self.vehicle_id}: stereo measurement stale or clock inconsistent")
        if (
            type(payload["timestamp_seconds"]) not in {int, float}
            or not math.isfinite(payload["timestamp_seconds"])
            or payload["timestamp_seconds"] < 0
            or type(payload["processing_seconds"]) not in {int, float}
            or not math.isfinite(payload["processing_seconds"])
            or not 0 <= payload["processing_seconds"] <= 1.8
            or not isinstance(payload["source_session_id"], str)
            or not payload["source_session_id"]
            or payload["backend"] not in {"cpu", "vpi-cuda"}
        ):
            raise RuntimeError(f"{self.vehicle_id}: invalid stereo source metadata")
        if self._cached_frame is not None:
            self._check_age(self._cached_frame)
            return self._cached_frame
        fractions = tuple(payload["valid_fractions"])
        points = tuple(tuple(p) for p in payload["points"])
        if (
            len(fractions) != self.calibration.rig_count
            or min(fractions) < 0.08
            or any(
                type(v) not in {int, float} or not math.isfinite(v) or not 0 <= v <= 1
                for v in fractions
            )
            or not 100 <= len(points) <= 30000
        ):
            raise RuntimeError(f"{self.vehicle_id}: stereo has insufficient supported coverage")
        for point in points:
            if (
                len(point) != 4
                or any(type(v) not in {int, float} or not math.isfinite(v) for v in point)
                or type(point[3]) is not int
                or not 0 <= point[3] < self.calibration.rig_count
                or not 0.8 <= point[2] <= 20
            ):
                raise RuntimeError("Invalid stereo optical measurement")
        sampling_stride = tuple(payload.get("point_sampling_stride_px", (4, 4)))
        if sampling_stride not in {(4, 4), (4, 2)}:
            raise RuntimeError("Unsupported stereo obstacle point sampling")
        self._cached_frame = StereoObstacleFrame(
            self.vehicle_id,
            controller_run_id,
            payload["source_session_id"],
            payload["timestamp_seconds"],
            captured,
            published,
            payload["calibration_id"],
            points,
            fractions,
            payload["processing_seconds"],
            payload["backend"],
            tuple(sorted(payload.get("input_quality", {}).items())),
            sampling_stride,
        )
        self._check_age(self._cached_frame)
        return self._cached_frame

    def _check_age(self, frame: StereoObstacleFrame) -> None:
        age = time.monotonic() - frame.captured_monotonic
        if age > 1.8:
            raise StaleObservationError(
                self.vehicle_id,
                f"{self.vehicle_id}: stereo measurement stale",
                timestamp_seconds=frame.timestamp_seconds,
                delivery_age_wall_seconds=age,
                source_session_id=frame.source_session_id,
            )

    def configure(self, controller_run_id: str, settings: dict) -> None:
        """Inject explicit camera-input faults; no obstacle geometry or localization values."""
        allowed = {"fault", "blur_sigma", "noise_sigma", "seed"}
        if set(settings) - allowed:
            raise ValueError("Unsupported camera-input setting")
        fault = settings.get("fault", "none")
        if fault not in {
            "none",
            "missing_left",
            "missing_right",
            "invalid_depth",
            "stale",
            "calibration",
            "depth_scale",
            "dark",
        }:
            raise ValueError("Unsupported stereo input fault")
        for key, maximum in [("blur_sigma", 3), ("noise_sigma", 20)]:
            value = settings.get(key, 0)
            if (
                type(value) not in {int, float}
                or not math.isfinite(value)
                or not 0 <= value <= maximum
            ):
                raise ValueError(f"{key} is outside supported camera quality limits")
        seed = settings.get("seed", 0)
        if type(seed) is not int or not 0 <= seed < 2**31:
            raise ValueError("Camera noise seed must be a nonnegative 31-bit integer")
        write_atomic_json(
            self.control_path,
            {"schema_version": 1, "controller_run_id": controller_run_id, "settings": settings},
        )

    def reset(self, controller_run_id: str) -> None:
        self.configure(controller_run_id, {"fault": "none"})
