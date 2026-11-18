"""Record bounded read-only API evidence during a separately initiated authored demo."""

import argparse
import json
import time
import urllib.request
from bisect import bisect_left
from pathlib import Path

import numpy as np

from common.logging.stereo import StereoRecording
from integrations.vision.evaluation import camera_offset_from_calibration, evaluate_trajectory

MAP_FIELDS = (
    "session_id",
    "map_id",
    "map_origin_id",
    "map_generation",
    "position_valid",
    "position_map_m",
    "tracking_state",
    "simulation_timestamp_ns",
    "stale",
    "processed_frames",
    "tracked_frames",
    "tracking_loss_events",
    "input_gap_events",
    "processing_seconds",
    "estimator_seconds",
    "surface_seconds",
    "surface_backend",
    "input_age_seconds",
    "map_frozen",
    "error",
    "received_wall_time",
)


def read_resource(resource: str) -> dict:
    """Read the loopback API; this observer has no command operations."""
    with urllib.request.urlopen(f"http://127.0.0.1:18080/{resource}", timeout=3) as response:
        return json.load(response)


def summarize(records: list[dict], offset: list[float]) -> dict:
    """Keep accuracy, phase availability and timing as separate measurements."""
    result = {}
    for vehicle_id in ("vehicle_1", "vehicle_2"):
        truth, observations = [], []
        for record in records:
            if "scene" not in record:
                continue
            scene = record["scene"]
            for pose in scene["poses"]:
                if pose["vehicle_id"] == vehicle_id:
                    truth.append({**pose, "epoch": scene["epoch"], "stale": scene["stale"]})
            mapping = record.get("mapping", {}).get(vehicle_id, {})
            if all(
                field in mapping
                for field in ("session_id", "map_id", "simulation_timestamp_ns", "position_valid")
            ):
                observations.append({**mapping, "epoch": scene["epoch"]})
        unique = {
            (item["session_id"], item["simulation_timestamp_ns"]): item for item in observations
        }
        timings = {}
        for field in (
            "processing_seconds",
            "estimator_seconds",
            "surface_seconds",
            "input_age_seconds",
        ):
            values = [item[field] for item in unique.values() if field in item]
            if values:
                timings[field] = dict(
                    samples=len(values),
                    p50=float(np.percentile(values, 50)),
                    p95=float(np.percentile(values, 95)),
                    maximum=max(values),
                )
        phases = {name: dict(samples=0, valid=0, frozen=0) for name in ("above_3m", "near_ground")}
        ordered_truth = sorted(
            {(r["epoch"], r["simulation_time_seconds"]): r for r in truth}.items()
        )
        truth_keys = [item[0] for item in ordered_truth]
        for item in unique.values():
            stamp = item["simulation_timestamp_ns"] / 1e9
            index = bisect_left(truth_keys, (item["epoch"], stamp))
            candidates = ordered_truth[max(0, index - 1) : index + 1]
            candidates = [
                r
                for key, r in candidates
                if key[0] == item["epoch"] and abs(key[1] - stamp) <= 0.1 and not r["stale"]
            ]
            if not candidates:
                continue
            pose = min(candidates, key=lambda r: abs(r["simulation_time_seconds"] - stamp))
            phase = phases["above_3m" if pose["position_enu_m"][2] > 3 else "near_ground"]
            phase["samples"] += 1
            phase["valid"] += int(item["position_valid"] and not item.get("stale", True))
            phase["frozen"] += int(item.get("map_frozen", False))
        sessions = {}
        for item in unique.values():
            session = sessions.setdefault(item["session_id"], {})
            for field in (
                "processed_frames",
                "tracked_frames",
                "tracking_loss_events",
                "input_gap_events",
            ):
                if field in item:
                    session[field] = max(session.get(field, 0), item[field])
        result[vehicle_id] = dict(
            session_counters=sessions,
            phases=phases,
            accuracy=evaluate_trajectory(observations, truth, offset),
            timings=timings,
            sampled_invalid_frames=sum(not item["position_valid"] for item in unique.values()),
            sampled_frozen_frames=sum(item.get("map_frozen", False) for item in unique.values()),
            sampling_scope="API sample, not every estimator frame; raw evidence retained",
        )
    return result


def capture_samples(destination: Path, duration_seconds: float) -> list[dict]:
    """Sample at most ten times per wall second for a bounded interval."""
    records = []
    started = time.monotonic()
    with (destination / "samples.jsonl").open("w", encoding="utf-8") as stream:
        while time.monotonic() - started < duration_seconds:
            tick = time.monotonic()
            record = {"wall_time": time.time()}
            try:
                record["scene"] = read_resource("scene")
                record["mapping"] = {
                    key: {
                        field: value
                        for field, value in read_resource(
                            f"perception/map?vehicle_id={key}"
                        ).items()
                        if field in MAP_FIELDS
                    }
                    for key in ("vehicle_1", "vehicle_2")
                }
            except (OSError, ValueError) as error:
                record["error"] = str(error)
            record["request_seconds"] = time.monotonic() - tick
            records.append(record)
            stream.write(json.dumps(record, allow_nan=False) + "\n")
            stream.flush()
            time.sleep(max(0, 0.1 - (time.monotonic() - tick)))
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--seconds", type=float, default=240)
    parser.add_argument("--baseline-m", type=float)
    parser.add_argument("--mount-height-m", type=float)
    parser.add_argument("--calibration-recording", type=Path)
    parser.add_argument("--replay-samples", type=Path)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 900:
        parser.error("Observation duration must be in 1..900 wall seconds")
    if args.calibration_recording:
        recording = StereoRecording.open(args.calibration_recording)
        calibration = recording.calibration
        calibration_hash = recording.manifest_sha256
    else:
        calibration = {"baseline_m": args.baseline_m, "mount_height_m": args.mount_height_m}
        calibration_hash = None
    try:
        offset = camera_offset_from_calibration(calibration)
    except ValueError as error:
        parser.error(str(error) + "; supply --calibration-recording or both explicit dimensions")
    args.destination.mkdir(parents=True, exist_ok=False)
    records = []
    started = time.monotonic()
    if args.replay_samples:
        if args.replay_samples.stat().st_size > 64 * 1024 * 1024:
            parser.error("Replay samples exceed the 64 MiB evidence budget")
        records = [json.loads(line) for line in args.replay_samples.read_text().splitlines()]
    else:
        records = capture_samples(args.destination, args.seconds)
    throughput = {}
    for record in records:
        scene = record.get("scene", {})
        if scene.get("stale", True) or not scene.get("poses"):
            continue
        epoch = str(scene["epoch"])
        stamp = scene["poses"][0]["simulation_time_seconds"]
        item = throughput.setdefault(epoch, {"first_wall": record["wall_time"], "first_sim": stamp})
        item.update(last_wall=record["wall_time"], last_sim=stamp)
    for item in throughput.values():
        elapsed = item["last_wall"] - item["first_wall"]
        item["observed_real_time_factor"] = (
            (item["last_sim"] - item["first_sim"]) / elapsed if elapsed else None
        )
    report = dict(
        simulation_throughput=throughput,
        schema_version=1,
        scope="read-only sampled observation; no flight authority",
        camera_offset_body_m=offset,
        calibration_manifest_sha256=calibration_hash,
        elapsed_wall_seconds=time.monotonic() - started,
        sample_count=len(records),
        request_errors=sum("error" in r for r in records),
        replay_source=str(args.replay_samples) if args.replay_samples else None,
        vehicles=summarize(records, offset),
    )
    (args.destination / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"report": str(args.destination / "report.json")}))
    return 0 if not report["request_errors"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
