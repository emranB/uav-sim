"""Retained case evidence keeps its revision and cannot bypass physical input checks."""

import json
import tempfile
import unittest
from pathlib import Path

from common.configuration.navigation_stereo import NavigationStereoCalibration
from testing.simulation.stereo_acceptance import load_preserved_cases


class StereoEvidencePreservationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        self.scene = root / "scene.json"
        self.scene.write_text(json.dumps({"schema_version": 1, "obstacles": []}))
        self.flight = root / "flight.json"
        self.payload = {
            "passed": True,
            "perception_criteria_passed": True,
            "obstacle_mode": "camera",
            "scenario": json.loads(self.scene.read_text()),
            "navigation_result": {
                "outcome": "reached",
                "camera_input_quality": {"fault": "none"},
                "camera_calibration_id": NavigationStereoCalibration().identifier,
            },
        }
        self.flight.write_text(json.dumps(self.payload))
        self.matrix = root / "matrix.json"
        self.matrix.write_text(
            json.dumps(
                {
                    "revision": "explicit-unit-fixture",
                    "cases": [
                        {
                            "name": "scene",
                            "mode": "camera",
                            "status": "pass",
                            "report": str(self.flight),
                        }
                    ],
                }
            )
        )
        self.declared = {"scene": {"scene": str(self.scene), "input_quality": {}}}

    def test_retained_case_is_unique_and_keeps_tested_revision_and_hash(self) -> None:
        cases = load_preserved_cases([self.matrix, self.matrix], self.declared, ["camera"])
        self.assertEqual(len(cases), 1)
        self.assertEqual(cases[0]["source_revision"], "explicit-unit-fixture")
        self.assertTrue(cases[0]["reused"])
        self.assertEqual(len(cases[0]["report_sha256"]), 64)

    def test_failed_physical_result_cannot_be_preserved_as_pass(self) -> None:
        self.payload["passed"] = False
        self.flight.write_text(json.dumps(self.payload))
        with self.assertRaisesRegex(ValueError, "incompatible or failed"):
            load_preserved_cases([self.matrix], self.declared, ["camera"])

    def test_changed_physical_scene_cannot_reuse_old_flight(self) -> None:
        self.scene.write_text(
            json.dumps({"schema_version": 1, "obstacles": [{"fixture": "changed"}]})
        )
        with self.assertRaisesRegex(ValueError, "incompatible or failed"):
            load_preserved_cases([self.matrix], self.declared, ["camera"])

    def test_diagnosed_failure_is_retained_as_failure_and_not_replaced(self) -> None:
        matrix = json.loads(self.matrix.read_text())
        matrix["cases"][0]["status"] = "fail"
        self.matrix.write_text(json.dumps(matrix))
        self.payload["passed"] = False
        self.payload["navigation_result"]["outcome"] = "failed"
        self.flight.write_text(json.dumps(self.payload))
        cases = load_preserved_cases([self.matrix], self.declared, ["camera"], retain_failed=True)
        self.assertEqual(len(cases), 1)
        self.assertEqual(cases[0]["status"], "fail")
        self.assertEqual(cases[0]["report"], str(self.flight))
