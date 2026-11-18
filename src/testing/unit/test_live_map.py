"""Current-session identity and real freshness are prerequisites for map readiness."""

import json
import tempfile
import time
import unittest
from pathlib import Path

from common.logging.live_map import read_live_map


class LiveMapTests(unittest.TestCase):
    def test_previous_generation_and_stale_maps_are_not_ready(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            (root / "px4.run").write_text("current", encoding="utf-8")
            snapshot = {
                "schema_version": 1,
                "vehicle_id": "vehicle_1",
                "controller_run_id": "previous",
                "received_wall_time": time.time(),
                "position_valid": True,
                "timestamp_seconds": 1,
                "valid_depth_fraction": 0.5,
                "trajectory": [[0, 0, 0]],
                "landmarks": [],
            }
            path = root / "mapping.json"
            path.write_text(json.dumps(snapshot), encoding="utf-8")
            self.assertFalse(read_live_map("vehicle_1", root)["ready"])
            snapshot["controller_run_id"] = "current"
            path.write_text(json.dumps(snapshot), encoding="utf-8")
            self.assertTrue(read_live_map("vehicle_1", root)["ready"])
            snapshot["received_wall_time"] -= 10
            path.write_text(json.dumps(snapshot), encoding="utf-8")
            self.assertTrue(read_live_map("vehicle_1", root)["stale"])

    def test_nonfinite_surface_and_wrong_target_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            (root / "px4.run").write_text("current", encoding="utf-8")
            snapshot = {
                "schema_version": 1,
                "vehicle_id": "vehicle_1",
                "controller_run_id": "current",
                "received_wall_time": time.time(),
                "position_valid": True,
                "surfaces": [[float("nan"), 0, 1]],
            }
            (root / "mapping.json").write_text(json.dumps(snapshot), encoding="utf-8")
            self.assertIn("coordinates", read_live_map("vehicle_1", root)["error"])
            self.assertFalse(read_live_map("vehicle_2", root)["ready"])
