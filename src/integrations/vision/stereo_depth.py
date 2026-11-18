"""Compute read-only metric depth for the isolated, rectified stereo probe."""

import argparse
import json
import math
from pathlib import Path

from common.logging.stereo import StereoRecording


def estimate_recording(source: Path, destination: Path) -> dict:
    """Produce depth evidence; never send estimates or commands to a vehicle."""
    import cv2
    import numpy as np

    recording = StereoRecording.open(source)
    calibration = recording.calibration
    baseline_m = calibration.get("baseline_m")
    field_of_view_rad = calibration.get("horizontal_fov_rad")
    if (
        not isinstance(baseline_m, (int, float))
        or isinstance(baseline_m, bool)
        or not math.isfinite(baseline_m)
        or not 0 < baseline_m < 1
        or not isinstance(field_of_view_rad, (int, float))
        or isinstance(field_of_view_rad, bool)
        or not math.isfinite(field_of_view_rad)
        or not 0 < field_of_view_rad < math.pi
        or calibration.get("distortion") != "none"
        or calibration.get("camera_convention") != "Gazebo sensor +X forward, +Y left, +Z up"
    ):
        raise ValueError("Depth requires the calibrated, parallel stereo probe recording")
    destination.mkdir(parents=True, exist_ok=False)
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=64,
        blockSize=5,
        P1=8 * 25,
        P2=32 * 25,
        uniquenessRatio=10,
        speckleWindowSize=50,
        speckleRange=2,
        disp12MaxDiff=1,
    )
    frames = []
    for index, frame in enumerate(recording.frames):
        if (frame.width, frame.height) not in {(320, 240), (640, 480)}:
            raise ValueError(
                "This feasibility adapter accepts only the calibrated probe dimensions"
            )
        images = recording.images(index)
        gray = [
            cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_GRAYSCALE) for image in images
        ]
        if any(image is None or image.shape != (frame.height, frame.width) for image in gray):
            raise ValueError("Recorded image dimensions do not match metadata")
        disparity = matcher.compute(gray[0], gray[1]).astype(np.float32) / 16.0
        valid = disparity > 0
        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        focal_length_px = frame.width / (2 * math.tan(field_of_view_rad / 2))
        depth[valid] = focal_length_px * baseline_m / disparity[valid]
        # Invalid estimates remain NaN; the color image is a display, not a confidence map.
        np.save(destination / f"{index:04d}-depth-m.npy", depth)
        display = np.zeros(disparity.shape, dtype=np.uint8)
        display[valid] = np.clip(depth[valid] / 15 * 255, 0, 255).astype(np.uint8)
        color = cv2.applyColorMap(display, cv2.COLORMAP_TURBO)
        color[~valid] = 0
        if not cv2.imwrite(str(destination / f"{index:04d}-depth.png"), color):
            raise OSError("Cannot write depth preview")
        # This fixed probe sees a front wall at 7 m. This is evaluation-only scene truth.
        wall = depth[
            frame.height // 3 : frame.height * 2 // 3,
            frame.width * 13 // 32 : frame.width * 20 // 32,
        ]
        observed = wall[np.isfinite(wall)]
        frames.append(
            {
                "simulation_timestamp_ns": frame.simulation_timestamp_ns,
                "valid_fraction": float(valid.mean()),
                "wall_region_valid_pixels": int(observed.size),
                "wall_region_median_depth_m": float(np.median(observed)) if observed.size else None,
                "wall_expected_depth_m": 7.0,
                "depth_file": f"{index:04d}-depth-m.npy",
                "preview": f"{index:04d}-depth.png",
            }
        )
    report = {
        "schema_version": 1,
        "source_recording": str(source),
        "source_manifest_sha256": recording.manifest_sha256,
        "frames": frames,
        "opencv_version": cv2.__version__,
        "numpy_version": np.__version__,
        "depth_units": "metres along camera optical axis",
        "invalid_value": "NaN",
        "preview_scale": "black=invalid; TURBO scale 0..15 metres, farther values clipped",
        "scope": "calibrated probe scene only; no obstacle planner, estimator or flight integration",
        "passed": all(
            item["wall_region_valid_pixels"] >= 50
            and item["wall_region_median_depth_m"] is not None
            and abs(item["wall_region_median_depth_m"] - 7.0) < 1.0
            for item in frames
        ),
    }
    (destination / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    arguments = parser.parse_args()
    try:
        report = estimate_recording(arguments.source, arguments.destination)
    except (OSError, ValueError, ImportError) as error:
        print(json.dumps({"passed": False, "error": str(error)}))
        return 1
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
