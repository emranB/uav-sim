"""Compare offline file and native pipe exchanges on identical retained return pairs."""

import argparse
import hashlib
import json
import math
import statistics
import threading
import time
from pathlib import Path

from common.logging.stereo import StereoRecording
from common.logging.storage import write_json_atomic
from integrations.vision.mapping import camera_configuration
from integrations.vision.native import NativeStereoEstimator
from integrations.vision.native_pipe import NativePosePipe


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    source = StereoRecording.open(args.recording)
    if not 30 <= len(source.frames) <= 200:
        parser.error("Expected 30..200 retained camera pairs")
    manifest = json.loads(args.recording.read_text())
    return_start = manifest["return_start_ns"]
    images = []
    digest = hashlib.sha256()
    for frame in source.frames:
        pair = [(source.directory / name).read_bytes() for name in (frame.left, frame.right)]
        for image in pair:
            digest.update(image)
        images.append(pair)
    args.output.mkdir(exist_ok=False)
    modes = {}
    issues = []
    for mode in ("file", "pipe"):
        directory = args.output / mode
        directory.mkdir()
        calibration = directory / "camera.yaml"
        calibration.write_text(camera_configuration(source, 40, 1200) + "loopClosing: 0\n")
        stopping = threading.Event()
        samples = []
        factory = (
            NativePosePipe(calibration, directory / "native.log")
            if mode == "pipe"
            else NativeStereoEstimator(directory, directory / "native.log")
        )
        with factory as estimator:
            started = None
            for frame, pair in zip(source.frames, images):
                if started is None and frame.simulation_timestamp_ns >= return_start:
                    started = time.monotonic()
                scheduled = (
                    started + (frame.simulation_timestamp_ns - return_start) / 0.50e9
                    if started is not None
                    else None
                )
                if scheduled is not None:
                    time.sleep(max(0, scheduled - time.monotonic()))
                tick = time.monotonic()
                estimate = estimator.estimate(
                    pair, frame.simulation_timestamp_ns, stopping, timeout_seconds=10
                )
                completed = time.monotonic()
                samples.append(
                    {
                        "timestamp_ns": frame.simulation_timestamp_ns,
                        "latency_wall_seconds": completed - tick,
                        "paced_receipt_age_wall_seconds": completed - scheduled
                        if scheduled is not None
                        else None,
                        "pose": {
                            key: estimate[key]
                            for key in (
                                "position_valid",
                                "tracking_state",
                                "map_generation",
                                "map_origin_id",
                                "position_map_m",
                                "orientation_xyzw",
                                "timestamp_seconds",
                            )
                        },
                    }
                )
        durations = [
            item["latency_wall_seconds"] for item in samples if item["timestamp_ns"] >= return_start
        ]
        ages = [
            item["paced_receipt_age_wall_seconds"]
            for item in samples
            if item["paced_receipt_age_wall_seconds"] is not None
        ]
        modes[mode] = {
            "service_capacity_wall_hz": len(durations) / sum(durations),
            "p95_latency_wall_seconds": statistics.quantiles(durations, n=20)[-1],
            "worst_paced_receipt_age_wall_seconds": max(ages),
            "samples": samples,
        }

    position_errors = []
    angle_errors = []
    mismatch = []
    for first, second in zip(modes["file"]["samples"], modes["pipe"]["samples"]):
        a, b = first["pose"], second["pose"]
        fields = ("timestamp_seconds", "tracking_state", "map_generation", "map_origin_id")
        if any(a[key] != b[key] for key in fields):
            mismatch.append(first["timestamp_ns"])
        if a["position_valid"] and b["position_valid"]:
            position_errors.append(math.dist(a["position_map_m"], b["position_map_m"]))
            dot = abs(sum(x * y for x, y in zip(a["orientation_xyzw"], b["orientation_xyzw"])))
            angle_errors.append(math.degrees(2 * math.acos(min(1.0, dot))))
    if mismatch or not position_errors or max(position_errors) > 0.03 or max(angle_errors) > 1.0:
        issues.append("Pose or tracking parity exceeded the declared 3 cm / 1 degree comparison")
    output = args.output / "report.json"
    write_json_atomic(
        output,
        {
            "schema_version": 1,
            "passed": not issues,
            "issues": issues,
            "input": str(args.recording),
            "input_sha256": digest.hexdigest(),
            "pairs": len(source.frames),
            "requested_pacing_rtf": 0.5,
            "max_position_difference_m": max(position_errors, default=None),
            "max_orientation_difference_deg": max(angle_errors, default=None),
            "tracking_mismatch_timestamps_ns": mismatch,
            "transports": modes,
        },
    )
    print(output)
    return int(bool(issues))


if __name__ == "__main__":
    raise SystemExit(main())
