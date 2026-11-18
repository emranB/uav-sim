"""Probe native stereo initialization with unsupported and supported image geometry."""

import argparse
import json
import tempfile
import threading
from pathlib import Path

import cv2
import numpy as np

from common.logging.stereo import StereoRecording
from integrations.vision.mapping import camera_configuration
from integrations.vision.native import NativeStereoEstimator


def probe(source: Path) -> dict:
    """Reject left-only texture, then initialize from an eight-pixel stereo shift."""
    recording = StereoRecording.open(source)
    frame = recording.frames[0]
    left = np.random.default_rng(19).integers(0, 256, (frame.height, frame.width), dtype=np.uint8)
    blank = np.zeros_like(left)
    right = np.zeros_like(left)
    right[:, :-8] = left[:, 8:]
    outcomes = []
    with tempfile.TemporaryDirectory(prefix="uav-initialization-probe-") as directory:
        work = Path(directory)
        (work / "camera.yaml").write_text(camera_configuration(recording, 10, 4000, 12))
        with NativeStereoEstimator(work, work / "native.log") as estimator:
            for timestamp, label, pair in (
                (1_000_000_000, "left_texture_without_stereo", (left, blank)),
                (2_000_000_000, "supported_metric_stereo", (left, right)),
            ):
                images = [cv2.imencode(".png", image)[1].tobytes() for image in pair]
                snapshot = estimator.estimate(images, timestamp, threading.Event())
                outcomes.append(
                    dict(
                        scenario=label,
                        position_valid=snapshot["position_valid"],
                        tracking_state=snapshot["tracking_state"],
                        landmarks=len(snapshot["landmarks"]),
                    )
                )
        log = (work / "native.log").read_text()[-3000:]
    return dict(
        schema_version=1,
        source_manifest_sha256=recording.manifest_sha256,
        scope="Native initialization only; synthetic stereo geometry, no vehicle commands",
        passed=not outcomes[0]["position_valid"]
        and outcomes[0]["landmarks"] == 0
        and outcomes[1]["position_valid"]
        and outcomes[1]["landmarks"] >= 30,
        outcomes=outcomes,
        native_log=log,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration_recording", type=Path)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    result = probe(args.calibration_recording)
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
