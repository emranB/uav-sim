"""Recorded imagery loads asynchronously and missing data is visible."""

import json
import tempfile
import time
import unittest
from pathlib import Path

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from control_panel.sensors.recording import StereoRecordingDialog


class StereoPlaybackTests(unittest.TestCase):
    def test_playback_freezes_its_manifest_and_escape_stops_the_timer(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for side in ("left", "right"):
                (root / f"{side}.ppm").write_bytes(b"P6\n1 1\n255\n\xff\x00\x00")
            pairs = [
                {
                    "simulation_timestamp_ns": stamp,
                    "left": "left.ppm",
                    "right": "right.ppm",
                    "width": 1,
                    "height": 1,
                }
                for stamp in (0, 1_000_000_000)
            ]
            path = root / "report.json"
            path.write_text(json.dumps({"schema_version": 1, "pairs": pairs}), encoding="utf-8")
            dialog = StereoRecordingDialog(path)
            self.wait_loaded(dialog)
            path.write_text('{"schema_version": 99}', encoding="utf-8")
            dialog.position.setValue(1)
            self.wait_loaded(dialog)
            self.assertIn("Frame 2/2", dialog.status.text())
            dialog.position.setValue(0)
            self.wait_loaded(dialog)
            dialog.play.click()
            self.assertTrue(dialog.timer.isActive())
            dialog.reject()
            self.assertFalse(dialog.timer.isActive())

    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def wait_loaded(self, dialog: StereoRecordingDialog) -> None:
        deadline = time.monotonic() + 3
        while dialog._loading and time.monotonic() < deadline:
            QTest.qWait(20)
        self.assertFalse(dialog._loading, "Recording load exceeded its test deadline")

    def test_real_pixels_and_missing_recording_are_distinct(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for side in ("left", "right"):
                (root / f"{side}.ppm").write_bytes(b"P6\n1 1\n255\n\xff\x00\x00")
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
            dialog = StereoRecordingDialog(path)
            self.wait_loaded(dialog)
            self.assertIn("RECORDED", dialog.status.text())
            self.assertFalse(dialog.views[0].pixmap().isNull())
            self.assertFalse(dialog.play.isEnabled())
            dialog.close()
            missing = StereoRecordingDialog(root / "missing.json")
            self.wait_loaded(missing)
            self.assertIn("Cannot read", missing.status.text())
            self.assertFalse(missing.play.isEnabled())
            missing.close()
