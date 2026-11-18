"""Measure the live stereo pose stream without changing flight or estimator settings."""

import argparse
import json
import math
import statistics
import time
from pathlib import Path

from common.configuration.fleet import load_fleet
from common.logging.storage import write_json_atomic


def main() -> int:
    """Record distinct native pose samples and check external-vision input targets."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", required=True)
    parser.add_argument("--seconds", type=int, default=10, choices=range(5, 31), metavar="5..30")
    args = parser.parse_args()
    definitions = {vehicle.vehicle_id: vehicle for vehicle in load_fleet()}
    if args.vehicle not in definitions:
        parser.error(f"Unknown vehicle: {args.vehicle}")

    directory = definitions[args.vehicle].runtime_directory
    output = Path("outputs/reports") / f"visual-odometry-input-{time.time_ns()}.json"
    samples = []
    issues = []
    previous_revision = None
    previous_publication = None
    started = time.monotonic()
    try:
        controller_run = (directory / "px4.run").read_text().strip()
        while time.monotonic() - started < args.seconds:
            path = directory / "mapping.json"
            stat = path.stat()
            publication = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
            if publication == previous_publication:
                time.sleep(0.01)
                continue
            with path.open("rb") as stream:
                contents = stream.read(3 * 1024 * 1024 + 1)
            if len(contents) > 3 * 1024 * 1024:
                raise ValueError("Mapping snapshot exceeds 3 MiB")
            snapshot = json.loads(contents)
            previous_publication = publication
            revision = (snapshot["session_id"], snapshot["processed_frames"])
            if revision != previous_revision:
                receipt_age = time.time() - snapshot["received_wall_time"]
                if not math.isfinite(receipt_age):
                    raise ValueError("Mapping snapshot has invalid image receipt time")
                samples.append(
                    {
                        "timestamp_ns": snapshot["simulation_timestamp_ns"],
                        "image_receipt_age_wall_seconds": receipt_age,
                        "controller_run_id": snapshot["controller_run_id"],
                        "session_id": snapshot["session_id"],
                        "map_generation": snapshot.get("map_generation"),
                        "position_valid": snapshot["position_valid"],
                        "processed_frames": snapshot["processed_frames"],
                    }
                )
                previous_revision = revision
            time.sleep(0.01)
        if (directory / "px4.run").read_text().strip() != controller_run:
            issues.append("Controller generation changed during measurement")
    except (KeyError, OSError, ValueError) as error:
        controller_run = None
        issues.append(str(error))

    intervals = [
        (second["timestamp_ns"] - first["timestamp_ns"]) / 1e9
        for first, second in zip(samples, samples[1:])
    ]
    simulated_seconds = sum(intervals)
    rate_hz = len(intervals) / simulated_seconds if simulated_seconds > 0 else 0.0
    ages = [sample["image_receipt_age_wall_seconds"] for sample in samples]
    skipped_publications = sum(
        max(0, second["processed_frames"] - first["processed_frames"] - 1)
        for first, second in zip(samples, samples[1:])
        if first["session_id"] == second["session_id"]
    )
    if len(samples) < 2 or any(interval <= 0 for interval in intervals):
        issues.append("Need advancing visual capture timestamps")
    if (
        any(
            not sample["position_valid"]
            or sample["map_generation"] is None
            or sample["controller_run_id"] != controller_run
            for sample in samples
        )
        or len({(sample["session_id"], sample["map_generation"]) for sample in samples}) > 1
    ):
        issues.append("Tracking, controller generation or visual frame identity changed")
    if skipped_publications:
        issues.append("Probe missed mapping publications")
    if not skipped_publications and rate_hz < 30:
        issues.append("Distinct visual poses did not reach the 30 Hz PX4 integration target")
    if ages and (min(ages) < 0 or max(ages) > 0.25):
        issues.append("Visual poses exceeded the 250 ms wall age budget")
    write_json_atomic(
        output,
        {
            "schema_version": 1,
            "vehicle_id": args.vehicle,
            "scope": "Read-only external vision input check on the existing deployment",
            "passed": not issues,
            "issues": issues,
            "distinct_pose_rate_sim_hz": rate_hz,
            "median_image_receipt_age_wall_seconds": statistics.median(ages) if ages else None,
            "max_image_receipt_age_wall_seconds": max(ages, default=None),
            "skipped_publications": skipped_publications,
            "samples": samples,
        },
    )
    print(output)
    return int(bool(issues))


if __name__ == "__main__":
    raise SystemExit(main())
