"""Reject malformed or stale stereo evidence at the file boundary."""

import json
import tempfile
import time
import unittest
from pathlib import Path

from common.configuration.navigation_stereo import NavigationStereoCalibration
from common.interfaces.obstacles import StaleObservationError
from integrations.vision.navigation_source import FileStereoObstacles, write_atomic_json


class StereoSourceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.source = FileStereoObstacles("vehicle_1", Path(self.directory.name))
        self.payload = {
            "schema_version": 1,
            "vehicle_id": "vehicle_1",
            "controller_run_id": "run",
            "source_session_id": "source",
            "timestamp_seconds": 10.0,
            "captured_monotonic": time.monotonic() - 0.1,
            "published_monotonic": time.monotonic(),
            "calibration_id": NavigationStereoCalibration().identifier,
            "ready": True,
            "points": [[0.0, 0.0, 5.0, i % 6] for i in range(120)],
            "valid_fractions": [0.5] * 6,
            "processing_seconds": 0.05,
            "backend": "cpu",
        }

    def publish(self) -> None:
        write_atomic_json(self.source.path, self.payload)

    def test_cached_frame_still_checks_age_and_generation(self) -> None:
        self.publish()
        self.assertEqual(len(self.source.latest("run").points), 120)
        with self.assertRaisesRegex(RuntimeError, "generation"):
            self.source.latest("different-run")
        self.payload["captured_monotonic"] -= 3
        self.publish()
        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.source.latest("run")

    def test_calibration_and_invalid_depth_are_rejected(self) -> None:
        self.payload["calibration_id"] = "wrong"
        self.publish()
        with self.assertRaisesRegex(RuntimeError, "calibration"):
            self.source.latest("run")
        self.payload["calibration_id"] = NavigationStereoCalibration().identifier
        self.payload["points"][0][2] = -1.0
        self.publish()
        with self.assertRaisesRegex(RuntimeError, "Invalid stereo"):
            self.source.latest("run")

    def test_missing_coverage_stays_unknown(self) -> None:
        self.payload["valid_fractions"][2] = 0
        self.publish()
        with self.assertRaisesRegex(RuntimeError, "coverage"):
            self.source.latest("run")

    def test_reset_clears_input_fault_without_removing_history(self) -> None:
        self.publish()
        self.source.configure("run", {"fault": "invalid_depth"})
        self.source.reset("new-run")
        controls = json.loads(self.source.control_path.read_text())
        self.assertEqual(controls["controller_run_id"], "new-run")
        self.assertEqual(controls["settings"], {"fault": "none"})
        self.assertTrue(self.source.path.exists())

    def test_unsupported_geometry_input_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.source.configure("run", {"obstacles": []})

    def test_only_valid_stale_records_are_retryable(self) -> None:
        self.payload["captured_monotonic"] -= 3
        self.publish()
        with self.assertRaises(StaleObservationError) as caught:
            self.source.latest("run")
        self.assertEqual(caught.exception.source_session_id, "source")
        for key, value in [
            ("calibration_id", "wrong"),
            ("timestamp_seconds", float("nan")),
            ("controller_run_id", "other"),
            ("points", [[0.0, 0.0, -1.0, 0]] * 120),
        ]:
            previous = self.payload[key]
            self.payload[key] = value
            # Deliberately malformed JSON numbers bypass the normal writer validation.
            self.source.path.write_text(json.dumps(self.payload))
            with self.subTest(key=key), self.assertRaises(RuntimeError) as failure:
                self.source.latest("run")
            self.assertNotIsInstance(failure.exception, StaleObservationError)
            self.payload[key] = previous
        self.payload["captured_monotonic"] = time.monotonic() + 3
        self.payload["published_monotonic"] = time.monotonic() + 4
        self.publish()
        with self.assertRaises(RuntimeError) as failure:
            self.source.latest("run")
        self.assertNotIsInstance(failure.exception, StaleObservationError)
