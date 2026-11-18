"""Recorded map readers preserve frame semantics and reject invalid coordinates."""

import json
import tempfile
import unittest
from pathlib import Path

from common.logging.mapping import RecordedMap


class RecordedMapTests(unittest.TestCase):
    def test_reads_estimates_without_relabelling_them_as_global_coordinates(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report = root / "report.json"
            report.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "tracked_frames": 2,
                        "total_frames": 3,
                        "passed": False,
                    }
                ),
                encoding="utf-8",
            )
            (root / "trajectory.tum").write_text("1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n")
            landmarks = root / "landmarks.json"
            landmarks.write_text(json.dumps({"points": [[1, 2, 3]]}), encoding="utf-8")
            mapping = RecordedMap.open(report)
            self.assertEqual(mapping.trajectory[-1], (1, 0, 0))
            self.assertIn("incomplete or failed", mapping.summary)
            landmarks.write_text('{"points": [[1, 2, NaN]]}', encoding="utf-8")
            with self.assertRaises(ValueError):
                RecordedMap.open(report)
