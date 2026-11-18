"""Compare optional surface backends on immutable stereo recordings, without flight."""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from common.logging.stereo import StereoRecording
from integrations.vision.geometry import StereoGeometry


def benchmark(source: Path, indices: list[int], repetitions: int) -> dict:
    """Measure wall/CPU costs and common measured-surface depth differences."""
    recording = StereoRecording.open(source)
    calibration = recording.calibration
    images = [recording.images(index) for index in indices]
    results = {}
    measurements = {}
    for backend in ("cpu", "vpi-cuda"):
        geometry = StereoGeometry(
            recording.frames[0].width,
            recording.frames[0].height,
            calibration["baseline_m"],
            calibration["horizontal_fov_rad"],
            backend,
        )
        geometry.measure(*images[0])
        durations = []
        cpu_started = time.process_time()
        wall_started = time.monotonic()
        for _ in range(repetitions):
            measurements[backend] = []
            for pair in images:
                started = time.monotonic()
                measured = geometry.measure(*pair)
                durations.append(time.monotonic() - started)
                measurements[backend].append(measured)
        results[backend] = dict(
            samples=len(durations),
            p50_seconds=float(np.percentile(durations, 50)),
            p95_seconds=float(np.percentile(durations, 95)),
            maximum_seconds=max(durations),
            wall_seconds=time.monotonic() - wall_started,
            cpu_seconds=time.process_time() - cpu_started,
        )
    comparisons = []
    focal = geometry.focal_px
    for index, cpu, cuda in zip(indices, measurements["cpu"], measurements["vpi-cuda"]):

        def indexed_depth(points: list) -> dict:
            return {
                (
                    round(x * focal / z + recording.frames[0].width / 2),
                    round(y * focal / z + recording.frames[0].height / 2),
                ): z
                for x, y, z in points
            }

        reference = indexed_depth(cpu.points)
        candidate = indexed_depth(cuda.points)
        common = reference.keys() & candidate.keys()
        differences = [abs(reference[key] - candidate[key]) for key in common]
        comparisons.append(
            dict(
                frame_index=index,
                timestamp_ns=recording.frames[index].simulation_timestamp_ns,
                cpu_valid_fraction=cpu.valid_fraction,
                cuda_valid_fraction=cuda.valid_fraction,
                common_points=len(common),
                median_absolute_depth_difference_m=float(np.median(differences))
                if differences
                else None,
                p95_absolute_depth_difference_m=float(np.percentile(differences, 95))
                if differences
                else None,
            )
        )
    return dict(
        source=str(source),
        manifest_sha256=recording.manifest_sha256,
        vehicle_id=recording.vehicle_id,
        timings=results,
        comparisons=comparisons,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--indices", type=int, nargs="+", required=True)
    parser.add_argument("--repetitions", type=int, default=5)
    args = parser.parse_args()
    if not 1 <= args.repetitions <= 20 or not 1 <= len(args.indices) <= 30:
        parser.error("Use 1..30 frames and 1..20 repetitions")
    cv2.setNumThreads(1)
    print(json.dumps(benchmark(args.source, args.indices, args.repetitions), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
