"""Bounded archival of the exact image pairs submitted to one native estimator."""

import json
from pathlib import Path

from common.logging.stereo import StereoRecording
from common.logging.storage import ensure_capacity, write_atomic


class StereoInputArchive:
    """Write immutable pairs from a serial observer thread, independently of UI state."""

    def __init__(
        self,
        directory: Path,
        recording: StereoRecording,
        limit_pairs: int,
        frame_rate_hz: int,
        controller_run_id: str,
    ) -> None:
        if type(limit_pairs) is not int or not 1 <= limit_pairs <= 1000:
            raise ValueError("Estimator input archive limit must be in 1..1000 pairs")
        if not recording.frames or type(frame_rate_hz) is not int or not 1 <= frame_rate_hz <= 10:
            raise ValueError("Estimator archive requires calibrated frames and a 1..10 Hz cadence")
        ensure_capacity(directory.parent, limit_pairs * 2 * 1024**2)
        self.directory = directory
        self.directory.mkdir(exist_ok=False)
        self.limit_pairs = limit_pairs
        self.width, self.height = recording.frames[0].width, recording.frames[0].height
        self._manifest = dict(
            schema_version=1,
            source="native-estimator-input",
            vehicle_id=recording.vehicle_id,
            controller_run_id=controller_run_id,
            pairs=[],
            recording_limit_pairs=limit_pairs,
            calibration={**recording.calibration, "rate_hz": frame_rate_hz},
            input_scope="Exact submitted pairs; missing camera frames are not reconstructed",
        )

    def append(self, timestamp_ns: int, images: list[bytes]) -> bool:
        """Return false when full; each successful append preserves one complete pair."""
        pairs = self._manifest["pairs"]
        if len(pairs) >= self.limit_pairs:
            return False
        if (
            type(timestamp_ns) is not int
            or timestamp_ns < 0
            or (pairs and timestamp_ns <= pairs[-1]["simulation_timestamp_ns"])
        ):
            raise ValueError("Estimator archive timestamps must increase")
        if len(images) != 2 or any(not image or len(image) > 1024 * 1024 for image in images):
            raise ValueError("Estimator archive needs two bounded images")
        index = len(pairs)
        names = [f"{index:04d}-{side}.ppm" for side in ("left", "right")]
        ensure_capacity(self.directory, sum(map(len, images)))
        for name, contents in zip(names, images):
            write_atomic(self.directory / name, contents)
        next_pair = dict(
            simulation_timestamp_ns=timestamp_ns,
            left=names[0],
            right=names[1],
            width=self.width,
            height=self.height,
        )
        next_manifest = {
            **self._manifest,
            "pairs": [*pairs, next_pair],
            "recording_full": len(pairs) + 1 == self.limit_pairs,
        }
        write_atomic(
            self.directory / "report.json",
            (json.dumps(next_manifest, allow_nan=False) + "\n").encode(),
        )
        self._manifest = next_manifest
        return True
