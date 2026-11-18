"""Bounded, read-only access to versioned stereo probe recordings."""

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path


def _read_bounded(path: Path, maximum_bytes: int) -> bytes:
    with path.open("rb") as stream:
        contents = stream.read(maximum_bytes + 1)
    if len(contents) > maximum_bytes:
        raise ValueError(f"Recording file exceeds {maximum_bytes} bytes: {path.name}")
    return contents


@dataclass(frozen=True)
class StereoFrame:
    simulation_timestamp_ns: int
    left: str
    right: str
    width: int
    height: int


@dataclass(frozen=True)
class StereoRecording:
    directory: Path
    frames: tuple[StereoFrame, ...]
    calibration: dict
    source: str = "unspecified recording"
    view_labels: tuple[str, str] = ("Left camera", "Right camera")
    evaluation_positions_enu_m: tuple = ()
    vehicle_id: str | None = None
    manifest_sha256: str = ""

    @classmethod
    def open(cls, path: Path) -> "StereoRecording":
        """Reject unknown schemas, unsafe paths and unbounded recording metadata."""
        encoded_manifest = _read_bounded(path, 1024 * 1024)
        manifest = json.loads(encoded_manifest)
        if manifest.get("schema_version") != 1:
            raise ValueError("Stereo recording requires schema version 1")
        pairs = manifest.get("pairs")
        if not isinstance(pairs, list) or not 1 <= len(pairs) <= 10000:
            raise ValueError("Stereo recording requires 1..10000 image pairs")
        frames = []
        evaluation = []
        previous = -1
        directory = path.parent.resolve()
        for pair in pairs:
            if not isinstance(pair, dict):
                raise ValueError("Stereo frame metadata must be an object")
            stamp, width, height = (
                pair.get("simulation_timestamp_ns"),
                pair.get("width"),
                pair.get("height"),
            )
            if type(stamp) is not int or stamp <= previous:
                raise ValueError("Stereo simulation timestamps must increase strictly")
            if any(type(size) is not int or not 0 < size <= 4096 for size in (width, height)):
                raise ValueError("Stereo dimensions must be integers in 1..4096")
            for side in ("left", "right"):
                name = pair.get(side)
                if not isinstance(name, str) or not name:
                    raise ValueError("Stereo frame needs left and right image paths")
                candidate = (directory / name).resolve()
                if not candidate.is_relative_to(directory) or candidate.suffix not in {
                    ".ppm",
                    ".png",
                }:
                    raise ValueError("Stereo image paths must stay inside the recording directory")
            frames.append(StereoFrame(stamp, pair["left"], pair["right"], width, height))
            position = pair.get("evaluation_rig_position_enu_m")
            if position is not None and (
                not isinstance(position, list)
                or len(position) != 3
                or any(
                    type(value) not in {int, float} or not math.isfinite(value)
                    for value in position
                )
            ):
                raise ValueError("Evaluation positions require three finite coordinates")
            evaluation.append(tuple(position) if position is not None else None)
            previous = stamp
        calibration = manifest.get("calibration", {})
        if not isinstance(calibration, dict):
            raise ValueError("Stereo calibration must be an object")
        source = manifest.get("source", "unspecified recording")
        labels = manifest.get("view_labels", ["Left camera", "Right camera"])
        if not isinstance(source, str) or len(source) > 120:
            raise ValueError("Recording source must be a short text label")
        if (
            not isinstance(labels, list)
            or len(labels) != 2
            or any(not isinstance(label, str) or len(label) > 80 for label in labels)
        ):
            raise ValueError("Recording needs two short view labels")
        vehicle_id = manifest.get("vehicle_id")
        if vehicle_id is not None and (
            not isinstance(vehicle_id, str) or not vehicle_id or len(vehicle_id) > 64
        ):
            raise ValueError("Recording vehicle identity must be a short nonempty label")
        return cls(
            directory,
            tuple(frames),
            calibration,
            source,
            tuple(labels),
            tuple(evaluation),
            vehicle_id,
            hashlib.sha256(encoded_manifest).hexdigest(),
        )

    def images(self, index: int) -> tuple[bytes, bytes]:
        """Read one bounded pair, checking containment again before opening files."""
        if type(index) is not int or not 0 <= index < len(self.frames):
            raise ValueError("Choose a frame inside the recording")
        frame = self.frames[index]
        images = []
        for name in (frame.left, frame.right):
            path = (self.directory / name).resolve()
            if not path.is_relative_to(self.directory):
                raise ValueError("Stereo image moved outside the recording directory")
            images.append(_read_bounded(path, 16 * 1024 * 1024))
        return images[0], images[1]
