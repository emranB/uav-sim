"""Recording boundaries reject unsafe paths and ambiguous simulation time."""

import json
import tempfile
import unittest
from pathlib import Path

from common.logging.stereo import StereoRecording


class StereoRecordingTests(unittest.TestCase):
    def test_recording_round_trip_and_missing_image_are_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = self.write_manifest(root)
            for side in ("left", "right"):
                (root / f"{side}.ppm").write_bytes(b"P6\n1 1\n255\n\xff\x00\x00")
            recording = StereoRecording.open(path)
            self.assertEqual(recording.images(0)[0], recording.images(0)[1])
            (root / "left.ppm").unlink()
            with self.assertRaises(OSError):
                recording.images(0)

    def test_external_paths_and_nonmonotonic_time_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = self.write_manifest(root)
            original = json.loads(path.read_text(encoding="utf-8"))
            for change in ("outside", "duplicate_time", "bad_dimensions", "unknown_schema"):
                manifest = json.loads(json.dumps(original))
                if change == "outside":
                    manifest["pairs"][0]["left"] = "../outside.ppm"
                elif change == "duplicate_time":
                    manifest["pairs"].append(manifest["pairs"][0])
                elif change == "bad_dimensions":
                    manifest["pairs"][0]["width"] = True
                else:
                    manifest["schema_version"] = 2
                path.write_text(json.dumps(manifest), encoding="utf-8")
                with self.subTest(change=change), self.assertRaises(ValueError):
                    StereoRecording.open(path)

    @staticmethod
    def write_manifest(root: Path) -> Path:
        path = root / "report.json"
        path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "pairs": [
                        {
                            "simulation_timestamp_ns": 0,
                            "left": "left.ppm",
                            "right": "right.ppm",
                            "width": 1,
                            "height": 1,
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        return path
