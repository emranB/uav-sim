"""Time native stereo pose estimation on a bounded isolated camera recording."""

import argparse
import statistics
import threading
import time
from pathlib import Path

from common.logging.stereo import StereoRecording
from common.logging.storage import write_json_atomic
from integrations.vision.mapping import camera_configuration
from integrations.vision.native import NativeStereoEstimator


def main() -> int:
    """Measure an estimator-only upper bound without consuming evaluation poses."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--features", type=int, default=2000, metavar="500..5000")
    args = parser.parse_args()

    recording = StereoRecording.open(args.source)
    if not 30 <= len(recording.frames) <= 200:
        parser.error("Use a bounded probe recording with 30..200 pairs")
    destination = args.source.parent / f"estimator-{args.features}-{time.time_ns()}"
    destination.mkdir()
    (destination / "camera.yaml").write_text(
        camera_configuration(recording, feature_count=args.features), encoding="utf-8"
    )
    samples = []
    issues = []
    stopping = threading.Event()
    try:
        with NativeStereoEstimator(destination, destination / "native.log") as estimator:
            deadline = time.monotonic() + 60
            for index, frame in enumerate(recording.frames):
                if time.monotonic() >= deadline:
                    raise TimeoutError("Native replay exceeded its 60 s processing budget")
                start = time.monotonic()
                images = [
                    (recording.directory / filename).read_bytes()
                    for filename in (frame.left, frame.right)
                ]
                estimate = estimator.estimate(
                    images,
                    frame.simulation_timestamp_ns,
                    stopping,
                    timeout_seconds=max(0.1, deadline - time.monotonic()),
                )
                samples.append(
                    {
                        "index": index,
                        "simulation_timestamp_ns": frame.simulation_timestamp_ns,
                        "processing_seconds": time.monotonic() - start,
                        "position_valid": estimate["position_valid"],
                        "map_generation": estimate.get("map_generation"),
                    }
                )
    except (OSError, RuntimeError, TimeoutError, ValueError) as error:
        issues.append(str(error))

    # First submission includes estimator startup. Subsequent costs still
    # include image reads and the same native exchange used by live mapping.
    durations = [sample["processing_seconds"] for sample in samples[1:]]
    rate_hz = len(durations) / sum(durations) if durations else 0.0
    if rate_hz < 30:
        issues.append("Estimator-only throughput below 30 poses per wall second")
    if len(samples) != len(recording.frames):
        issues.append("Recording replay did not complete")
    if not samples or not samples[-1]["position_valid"]:
        issues.append("No valid pose at end of replay")
    tracked = [sample for sample in samples if sample["position_valid"]]
    if tracked and (
        any(not sample["position_valid"] for sample in samples[tracked[0]["index"] :])
        or len({sample["map_generation"] for sample in tracked}) != 1
        or tracked[0]["map_generation"] is None
    ):
        issues.append("Tracking continuity or coordinate generation failed after initialization")
    output = destination / "report.json"
    write_json_atomic(
        output,
        {
            "schema_version": 1,
            "scope": "Unpaced native stereo replay without surface mapping or flight control",
            "source": str(args.source),
            "source_sha256": recording.manifest_sha256,
            "features": args.features,
            "passed": not issues,
            "issues": issues,
            "throughput_wall_hz_after_startup": rate_hz,
            "median_processing_seconds_after_startup": statistics.median(durations)
            if durations
            else None,
            "tracked_frames": len(tracked),
            "samples": samples,
        },
    )
    print(output)
    return int(bool(issues))


if __name__ == "__main__":
    raise SystemExit(main())
