"""Geometric image detections and feature overlays, without navigation commands."""

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

from common.logging.stereo import StereoRecording


@dataclass(frozen=True)
class ImageRegion:
    """A pixel-space contour, not an object class or a three-dimensional obstacle."""

    bounds_xywh_px: tuple[int, int, int, int]
    centroid_xy_px: tuple[float, float]
    area_px2: float


def detect_regions(image: np.ndarray) -> tuple[ImageRegion, ...]:
    """Find bounded external contours; no claim about unseen or textureless surfaces."""
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Region extraction requires an unsigned 8-bit BGR image")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        moments = cv2.moments(contour)
        if area >= 100 and moments["m00"] > 0:
            candidates.append(
                ImageRegion(
                    tuple(int(value) for value in cv2.boundingRect(contour)),
                    (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]),
                    area,
                )
            )
    return tuple(sorted(candidates, key=lambda item: item.area_px2, reverse=True)[:100])


def analyze_recording(source: Path, destination: Path, maximum_frames: int = 200) -> dict:
    if not 1 <= maximum_frames <= 1000:
        raise ValueError("Use an analysis limit of 1..1000 frames")
    recording = StereoRecording.open(source)
    destination.mkdir(parents=True, exist_ok=False)
    extractor = cv2.ORB_create(nfeatures=1200)
    pairs, results = [], []
    for index, frame in enumerate(recording.frames[:maximum_frames]):
        left, _ = recording.images(index)
        image = cv2.imdecode(np.frombuffer(left, np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (frame.height, frame.width):
            raise ValueError("Cannot decode the recorded image geometry")
        keypoints = extractor.detect(image, None)
        features = cv2.drawKeypoints(image, keypoints, None, color=(150, 230, 80))
        regions = detect_regions(image)
        annotated = image.copy()
        for region in regions:
            x, y, width, height = region.bounds_xywh_px
            cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 190, 255), 2)
            center = tuple(round(value) for value in region.centroid_xy_px)
            cv2.circle(annotated, center, 3, (0, 190, 255), -1)
        names = f"{index:04d}-features.png", f"{index:04d}-regions.png"
        for name, view in zip(names, (features, annotated)):
            if not cv2.imwrite(str(destination / name), view):
                raise OSError(f"Cannot write {name}")
        pairs.append(
            {
                "simulation_timestamp_ns": frame.simulation_timestamp_ns,
                "left": names[0],
                "right": names[1],
                "width": frame.width,
                "height": frame.height,
            }
        )
        results.append(
            {"feature_count": len(keypoints), "regions": [asdict(item) for item in regions]}
        )
    report = {
        "schema_version": 1,
        "source": "recorded-geometric-analysis",
        "view_labels": ["Visual features", "Geometric image regions"],
        "source_recording": str(source),
        "source_manifest_sha256": recording.manifest_sha256,
        "vehicle_id": recording.vehicle_id,
        "pairs": pairs,
        "results": results,
        "input_frames": len(recording.frames),
        "processed_frames": len(pairs),
        "limited": len(recording.frames) > maximum_frames,
        "scope": "pixel-space contours and ORB features; no semantic labels or obstacle clearance",
    }
    (destination / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--maximum-frames", type=int, default=200)
    arguments = parser.parse_args()
    try:
        report = analyze_recording(
            arguments.source, arguments.destination, arguments.maximum_frames
        )
    except (OSError, ValueError, cv2.error) as error:
        print(json.dumps({"completed": False, "error": str(error)}))
        return 1
    print(json.dumps({"completed": True, "processed_frames": report["processed_frames"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
