"""Replay a bounded recorded prefix into an independent native observer, without flight."""

import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np

from common.logging.stereo import StereoRecording
from integrations.vision.mapping import camera_configuration
from integrations.vision.native import NativeStereoEstimator


def replay(source: Path, destination: Path, frames: int, levels: int, pace_hz: float) -> dict:
    """Compare tracking on actual archived timestamps; never invent missing live frames."""
    recording = StereoRecording.open(source)
    selected = recording.frames[:frames]
    intervals = np.diff([frame.simulation_timestamp_ns / 1e9 for frame in selected])
    frame_rate = round(1 / float(np.median(intervals)))
    (destination / "camera.yaml").write_text(
        camera_configuration(recording, frame_rate, 4000, levels)
    )
    work = destination / "exchange"
    work.mkdir()
    (work / "camera.yaml").write_text((destination / "camera.yaml").read_text())
    observations = []
    started = time.monotonic()
    try:
        with NativeStereoEstimator(work, destination / "native.log") as estimator:
            with (destination / "observations.jsonl").open("w") as stream:
                for index, frame in enumerate(selected):
                    if time.monotonic() - started > 180:
                        raise TimeoutError("Recorded prefix exceeded its 180 second deadline")
                    tick = time.monotonic()
                    snapshot = estimator.estimate(
                        list(recording.images(index)),
                        frame.simulation_timestamp_ns,
                        threading.Event(),
                    )
                    observation = {
                        key: snapshot.get(key)
                        for key in (
                            "timestamp_seconds",
                            "position_valid",
                            "position_map_m",
                            "tracking_state",
                            "map_id",
                            "map_origin_id",
                            "map_generation",
                        )
                    }
                    observation["source_index"] = index
                    observation["processing_seconds"] = time.monotonic() - tick
                    observations.append(observation)
                    stream.write(json.dumps(observation) + "\n")
                    stream.flush()
                    if pace_hz:
                        time.sleep(max(0, 1 / pace_hz - (time.monotonic() - tick)))
    finally:
        for filename in work.iterdir():
            filename.unlink()
        work.rmdir()
    first = next((index for index, item in enumerate(observations) if item["position_valid"]), None)
    evaluated = observations[first:] if first is not None else []
    losses = sum(
        a["position_valid"] and not b["position_valid"] for a, b in zip(evaluated, evaluated[1:])
    )
    identities = {(r["map_id"], r["map_generation"]) for r in evaluated if r["position_valid"]}
    return dict(
        schema_version=1,
        vehicle_id=recording.vehicle_id,
        manifest_sha256=recording.manifest_sha256,
        levels=levels,
        source_frames=len(selected),
        source_rate_hz=frame_rate,
        pace_wall_hz=pace_hz,
        source=recording.source,
        scope="Recorded prefix only; no interpolation or vehicle commands",
        first_tracked_source_index=first,
        tracking_loss_events=losses,
        valid_after_initialization=sum(r["position_valid"] for r in evaluated),
        evaluated_frames=len(evaluated),
        coordinate_frames=sorted(identities),
        passed=first is not None and len(evaluated) >= 20 and losses == 0 and len(identities) == 1,
        elapsed_wall_seconds=time.monotonic() - started,
        control_authority=False,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--frames", type=int, default=70)
    parser.add_argument("--levels", type=int, default=12)
    parser.add_argument("--pace-hz", type=float, default=5)
    args = parser.parse_args()
    if not 2 <= args.frames <= 1000 or not 8 <= args.levels <= 16 or not 0 <= args.pace_hz <= 10:
        parser.error("Use 2..1000 frames, 8..16 levels and 0..10 pacing Hz")
    args.destination.mkdir(parents=True, exist_ok=False)
    try:
        result = replay(args.source, args.destination, args.frames, args.levels, args.pace_hz)
    except (OSError, ValueError, RuntimeError, TimeoutError) as error:
        result = dict(schema_version=1, passed=False, error=str(error))
    (args.destination / "report.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
