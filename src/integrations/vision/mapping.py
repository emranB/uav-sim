"""Run ORB-SLAM3 on recorded images without any vehicle-command transport."""

import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

from common.logging.stereo import StereoRecording


def camera_configuration(
    recording: StereoRecording,
    frame_rate_hz: int | None = None,
    feature_count: int = 2000,
    pyramid_levels: int = 8,
) -> str:
    """Describe the calibrated parallel stereo rig in ORB-SLAM3's file format."""
    calibration = recording.calibration
    if type(feature_count) is not int or not 500 <= feature_count <= 5000:
        raise ValueError("Mapping feature budget must be an integer in 500..5000")
    if type(pyramid_levels) is not int or not 8 <= pyramid_levels <= 16:
        raise ValueError("Mapping pyramid must have 8..16 levels")
    width, height = recording.frames[0].width, recording.frames[0].height
    # The pinned extractor subtracts two 16-pixel borders at every pyramid level.
    # A zero/negative interior can abort in its keypoint-distribution allocation.
    highest_scale = 1.2 ** (pyramid_levels - 1)
    if min(round(width / highest_scale), round(height / highest_scale)) <= 32:
        raise ValueError("Highest pyramid image must exceed 32 pixels; reduce pyramid levels")
    baseline = calibration.get("baseline_m")
    field_of_view = calibration.get("horizontal_fov_rad")
    if (
        type(baseline) not in {int, float}
        or not math.isfinite(baseline)
        or baseline <= 0
        or type(field_of_view) not in {int, float}
        or not math.isfinite(field_of_view)
        or not 0 < field_of_view < math.pi
        or calibration.get("distortion") != "none"
    ):
        raise ValueError("Mapping requires calibrated parallel stereo imagery")
    if any((frame.width, frame.height) != (width, height) for frame in recording.frames):
        raise ValueError("Mapping image dimensions changed during the recording")
    focal = width / (2 * math.tan(field_of_view / 2))
    differences = [
        right.simulation_timestamp_ns - left.simulation_timestamp_ns
        for left, right in zip(recording.frames, recording.frames[1:])
    ]
    fps = (
        round(1e9 / sorted(differences)[len(differences) // 2])
        if differences
        else calibration.get("rate_hz", 5)
    )
    if frame_rate_hz is not None:
        if type(frame_rate_hz) is not int:
            raise ValueError("Live camera cadence must be an integer")
        fps = frame_rate_hz
    if not 1 <= fps <= 120:
        raise ValueError("Recorded camera cadence must be in 1..120 frames per second")
    # The pinned release dereferences camera 2 while printing Rectified settings.
    # Explicit pinhole calibration with zero distortion keeps both cameras defined.
    lines = ["%YAML:1.0", 'File.version: "1.0"', 'Camera.type: "PinHole"']
    for key, value in {
        "Camera1.fx": focal,
        "Camera1.fy": focal,
        "Camera1.cx": width / 2,
        "Camera1.cy": height / 2,
        "Camera.width": width,
        "Camera.height": height,
        "Camera.fps": fps,
        "Camera.RGB": 0,
        "Stereo.b": baseline,
        "Stereo.ThDepth": 80.0,
        "ORBextractor.nFeatures": feature_count,
        "ORBextractor.scaleFactor": 1.2,
        "ORBextractor.nLevels": pyramid_levels,
        "ORBextractor.iniThFAST": 20,
        "ORBextractor.minThFAST": 7,
        "Viewer.KeyFrameSize": 0.05,
        "Viewer.KeyFrameLineWidth": 1.0,
        "Viewer.GraphLineWidth": 0.9,
        "Viewer.PointSize": 2.0,
        "Viewer.CameraSize": 0.08,
        "Viewer.CameraLineWidth": 3.0,
        "Viewer.ViewpointX": 0.0,
        "Viewer.ViewpointY": -0.7,
        "Viewer.ViewpointZ": -1.8,
        "Viewer.ViewpointF": 500.0,
        "Viewer.imageViewScale": 1.0,
    }.items():
        lines.append(f"{key}: {value}")
    for camera in (1, 2):
        if camera == 2:
            for name, value in (
                ("fx", focal),
                ("fy", focal),
                ("cx", width / 2),
                ("cy", height / 2),
            ):
                lines.append(f"Camera2.{name}: {value}")
        for name in ("k1", "k2", "p1", "p2", "k3"):
            lines.append(f"Camera{camera}.{name}: 0.0")
    lines.extend(
        [
            "Stereo.T_c1_c2: !!opencv-matrix",
            "  rows: 4",
            "  cols: 4",
            "  dt: f",
            f"  data: [1.0, 0.0, 0.0, {baseline}, 0.0, 1.0, 0.0, 0.0, "
            "0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]",
        ]
    )
    return "\n".join(lines) + "\n"


def run_mapping(source: Path, destination: Path, timeout_seconds: float = 300) -> dict:
    """Export explicit tracking outcomes; missing tracking is a failed mapping check."""
    if not math.isfinite(timeout_seconds) or not 1 <= timeout_seconds <= 1800:
        raise ValueError("Mapping timeout must be in 1..1800 seconds")
    recording = StereoRecording.open(source)
    configuration = camera_configuration(recording)
    destination.mkdir(parents=True, exist_ok=False)
    settings_path = destination / "camera.yaml"
    settings_path.write_text(configuration, encoding="utf-8")
    frames = []
    for index, frame in enumerate(recording.frames):
        # Validate bounded files and containment before passing paths to native code.
        left, right = recording.images(index)
        frames.append(
            {
                "timestamp_seconds": frame.simulation_timestamp_ns / 1e9,
                "left": str((recording.directory / frame.left).resolve()),
                "right": str((recording.directory / frame.right).resolve()),
                "left_sha256": hashlib.sha256(left).hexdigest(),
                "right_sha256": hashlib.sha256(right).hexdigest(),
            }
        )
    inputs = destination / "inputs.json"
    inputs.write_text(json.dumps({"frames": frames}), encoding="utf-8")
    started = time.monotonic()
    process = None
    report = {
        "schema_version": 1,
        "passed": False,
        "source_recording": str(source),
        "source_manifest_sha256": recording.manifest_sha256,
        "vehicle_id": recording.vehicle_id,
        "frame": "ORB-SLAM local map, camera optical convention; metres",
        "scope": "recorded perception only; no control authority or vehicle commands",
    }
    try:
        with (destination / "mapping.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                [
                    "orb-recording",
                    "/opt/orb-slam3/Vocabulary/ORBvoc.txt",
                    str(settings_path.resolve()),
                    str(inputs.resolve()),
                    str(destination.resolve()),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            return_code = process.wait(timeout=timeout_seconds)
        if return_code:
            raise RuntimeError(f"ORB-SLAM3 exited with code {return_code}; inspect mapping.log")
        observations = [
            json.loads(line)
            for line in (destination / "observations.jsonl")
            .read_text(encoding="utf-8")
            .splitlines()
        ]
        tracked = sum(item["position_valid"] for item in observations)
        landmarks = json.loads((destination / "landmarks.json").read_text(encoding="utf-8"))
        report.update(
            {
                "tracked_frames": tracked,
                "total_frames": len(observations),
                "tracking_fraction": tracked / len(frames),
                "landmark_count": len(landmarks["points"]),
                "passed": len(observations) == len(frames)
                and tracked >= 2
                and tracked / len(frames) >= 0.9
                and bool(landmarks["points"]),
                "validation_scope": "recorded-input processing and tracking availability",
            }
        )
        if not report["passed"]:
            report["error"] = "Insufficient valid tracking or mapped landmarks"
        valid = [
            (observation["position_map_m"], truth)
            for observation, truth in zip(observations, recording.evaluation_positions_enu_m)
            if observation["position_valid"] and truth is not None
        ]
        if valid:
            estimate_origin, truth_origin = valid[0]
            errors = []
            for estimate, truth in valid:
                delta = [truth[axis] - truth_origin[axis] for axis in range(3)]
                expected = (-delta[1], -delta[2], delta[0])
                errors.append(
                    math.dist(
                        [estimate[axis] - estimate_origin[axis] for axis in range(3)], expected
                    )
                )
            report["evaluation"] = {
                "samples": len(errors),
                "position_rmse_m": math.sqrt(sum(value * value for value in errors) / len(errors)),
                "maximum_position_error_m": max(errors),
                "alignment": "initial camera optical frame; translation origin only, no scale fit",
                "source": "simulation truth used after estimation, never passed to ORB-SLAM3",
            }
            evaluation = report["evaluation"]
            evaluation["acceptance_rmse_m"] = 0.1
            evaluation["acceptance_maximum_error_m"] = 0.25
            evaluation["passed"] = (
                len(errors) >= 2
                and evaluation["position_rmse_m"] <= 0.1
                and evaluation["maximum_position_error_m"] <= 0.25
            )
            report["passed"] = report["passed"] and evaluation["passed"]
            if not evaluation["passed"]:
                report["error"] = "Recorded probe position error exceeded its acceptance bounds"
        report["revision"] = Path("/opt/orb-slam3/revision.txt").read_text(encoding="utf-8").strip()
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        RuntimeError,
        subprocess.TimeoutExpired,
    ) as error:
        report["passed"] = False
        report["error"] = str(error)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
    report["elapsed_wall_seconds"] = time.monotonic() - started
    (destination / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--timeout-seconds", type=float, default=300)
    arguments = parser.parse_args()
    try:
        report = run_mapping(arguments.source, arguments.destination, arguments.timeout_seconds)
    except (OSError, ValueError) as error:
        report = {"passed": False, "error": str(error)}
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
