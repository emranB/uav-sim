"""Bounded access to estimated, recorded map artifacts."""

import json
import math
from dataclasses import dataclass
from pathlib import Path


def _read(path: Path, limit: int) -> str:
    with path.open("rb") as stream:
        contents = stream.read(limit + 1)
    if len(contents) > limit:
        raise ValueError(f"Map artifact exceeds its size limit: {path.name}")
    return contents.decode("utf-8")


@dataclass(frozen=True)
class RecordedMap:
    trajectory: tuple[tuple[float, float, float], ...]
    landmarks: tuple[tuple[float, float, float], ...]
    summary: str

    @classmethod
    def open(cls, path: Path) -> "RecordedMap":
        report = json.loads(_read(path, 1024 * 1024))
        if report.get("schema_version") != 1 or "tracked_frames" not in report:
            raise ValueError("Expected a version-1 mapping report")
        directory = path.parent.resolve()
        artifacts = []
        for name in ("trajectory.tum", "landmarks.json"):
            artifact = (directory / name).resolve()
            if not artifact.is_relative_to(directory):
                raise ValueError("Map artifacts must remain inside their recording directory")
            artifacts.append(artifact)
        trajectory = []
        for line in _read(artifacts[0], 2 * 1024 * 1024).splitlines():
            if line.startswith("#") or not line.strip():
                continue
            values = [float(value) for value in line.split()]
            if len(values) != 8 or not all(math.isfinite(value) for value in values):
                raise ValueError("Invalid camera trajectory sample")
            trajectory.append(tuple(values[1:4]))
        landmarks = json.loads(_read(artifacts[1], 5 * 1024 * 1024)).get("points", [])
        if len(trajectory) > 10000 or not isinstance(landmarks, list) or len(landmarks) > 20000:
            raise ValueError("Map presentation exceeds its point budget")
        for point in landmarks:
            if (
                not isinstance(point, list)
                or len(point) != 3
                or any(
                    type(value) not in {int, float} or not math.isfinite(value) for value in point
                )
            ):
                raise ValueError("Map landmarks require three finite coordinates")
        summary = (
            f"Recorded ORB-SLAM3 | Tracked {report['tracked_frames']}/{report['total_frames']} "
            f"frames | {len(landmarks)} last-tracked landmarks | "
            + (
                "Processing checks passed"
                if report.get("passed")
                else "Validation incomplete or failed"
            )
        )
        return cls(tuple(trajectory), tuple(tuple(point) for point in landmarks), summary)
