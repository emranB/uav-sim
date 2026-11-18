"""Native/replay composition rejects mismatched evidence and preserves exact overlap."""

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from integrations.blender.native_composite import load_native_recording


class NativeCompositeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.job = self.root / "replay-jobs/job-test"
        self.job.mkdir(parents=True)
        self.identifier = "capture-" + "a" * 32
        (self.job / "export-manifest.json").write_text(
            json.dumps(
                {
                    "capture_id": self.identifier,
                    "start_monotonic": 102,
                    "end_monotonic": 105,
                }
            )
        )
        capture = self.root / "replay" / self.identifier
        capture.mkdir(parents=True)
        (capture / "manifest.json").write_text(
            json.dumps(
                {
                    "started_monotonic": 100,
                    "started_wall_time": 1000,
                }
            )
        )
        self.package = self.root / "native-screen" / self.identifier
        self.package.mkdir(parents=True)
        self.movie = self.package / "native-screen.mkv"
        self.movie.write_bytes(b"recorded pixels")
        self.record = {
            "state": "completed",
            "capture_id": self.identifier,
            "dimensions": [1600, 1000],
            "first_input_wall_time": 1001.5,
            "input_frame_wall_times": [1001.5 + index / 10 for index in range(31)],
            "sha256": hashlib.sha256(self.movie.read_bytes()).hexdigest(),
        }
        self.publish()

    def publish(self) -> None:
        (self.package / "recording.json").write_text(json.dumps(self.record))

    def test_selected_export_clock_and_actual_screen_overlap(self) -> None:
        result = load_native_recording(self.job, lambda: None)
        self.assertEqual(result["capture_started_wall_time"], 1002)
        self.assertEqual(result["start_seconds"], 0)
        self.assertEqual(result["end_seconds"], 2.5)
        self.assertEqual(result["native_start_seconds"], 0.5)

    def test_native_save_tail_is_retained_without_extending_recorded_flight(self) -> None:
        self.record["input_frame_wall_times"] = [1001.5 + index / 10 for index in range(61)]
        self.publish()
        result = load_native_recording(self.job, lambda: None)
        self.assertEqual(result["end_seconds"], 3)
        self.assertEqual(result["native_end_seconds"], 5.5)
        from integrations.blender.native_composite import composite_annotations

        path = self.job / "labels.ass"
        composite_annotations(path, "Mission", result)
        self.assertIn("CAPTURE ENDED | LAST RECORDED POSE", path.read_text())
        self.assertIn("SEALED at +2.900 s", path.read_text())

    def test_modified_movie_and_wrong_capture_are_rejected(self) -> None:
        self.movie.write_bytes(b"other run")
        with self.assertRaisesRegex(ValueError, "checksum"):
            load_native_recording(self.job, lambda: None)
        self.record["capture_id"] = "capture-" + "b" * 32
        self.publish()
        with self.assertRaisesRegex(ValueError, "identity"):
            load_native_recording(self.job, lambda: None)

    def test_non_monotonic_timestamps_and_unsealed_screen_are_rejected(self) -> None:
        self.record["input_frame_wall_times"] = [1001.5, 1001.5]
        self.publish()
        with self.assertRaisesRegex(ValueError, "increasing"):
            load_native_recording(self.job, lambda: None)
        self.record["state"] = "recording"
        self.publish()
        with self.assertRaises(TimeoutError):
            load_native_recording(self.job, lambda: None, timeout_seconds=0)

    def test_long_screen_gap_is_not_silently_repeated_under_moving_replay(self) -> None:
        self.record["input_frame_wall_times"] = [1001.5, 1004]
        self.publish()
        with self.assertRaisesRegex(ValueError, "input gap"):
            load_native_recording(self.job, lambda: None)

    def test_cancellation_propagates_before_using_a_sealed_movie(self) -> None:
        def cancelled() -> None:
            raise InterruptedError("Owner cancelled")

        with self.assertRaises(InterruptedError):
            load_native_recording(self.job, cancelled)
