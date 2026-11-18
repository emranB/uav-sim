"""Preserve bounded, exact stereo inputs around an estimator tracking failure."""

import gzip
import json
from pathlib import Path


def write_tracking_failure(
    directory: Path,
    previous: tuple[int, list[bytes]],
    current: tuple[int, list[bytes]],
    snapshot: dict,
) -> Path:
    """Save two timestamped pairs; input bytes remain unchanged after decompression."""
    for timestamp, images in (previous, current):
        if type(timestamp) is not int or timestamp < 0:
            raise ValueError("Tracking diagnostics require nonnegative nanosecond timestamps")
        if len(images) != 2 or any(not image or len(image) > 1024 * 1024 for image in images):
            raise ValueError("Tracking diagnostics require two images of at most 1 MiB each")
    destination = directory / f"tracking-loss-{current[0]}"
    destination.mkdir()
    for label, (_, images) in (("previous", previous), ("current", current)):
        for side, contents in zip(("left", "right"), images):
            (destination / f"{label}-{side}.ppm.gz").write_bytes(
                gzip.compress(contents, compresslevel=1, mtime=0)
            )
    metadata = {
        "previous_timestamp_ns": previous[0],
        "current_timestamp_ns": current[0],
        "tracking_state": snapshot["tracking_state"],
        "map_id": snapshot.get("map_id"),
        "scope": "exact failed input and its preceding estimator pair",
    }
    (destination / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return destination
