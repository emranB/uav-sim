"""Embedded reconstruction reports missing data and clears previous vehicle state."""

import unittest

from PySide6.QtWidgets import QApplication

from control_panel.sensors.reconstruction import LiveReconstructionView
from testing.component.test_panels import RecordedClient


class ReconstructionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_stale_result_and_vehicle_change_clear_geometry(self) -> None:
        client = RecordedClient()
        view = LiveReconstructionView(client)
        payload = {
            "ready": True,
            "vehicle_id": "vehicle_1",
            "timestamp_seconds": 2,
            "age_seconds": 0.1,
            "valid_depth_fraction": 0.5,
            "trajectory": [[0, 0, 0]],
            "landmarks": [[1, 2, 3]],
            "surfaces": [[2, 1, 4]],
        }
        client.resource_received.emit("perception/map", payload)
        self.assertEqual(len(view.canvas.surfaces), 1)
        client.selection_changed.emit("vehicle_2")
        self.assertIsNone(view.canvas.mapping)
        self.assertEqual(view.canvas.surfaces, ())
        client.resource_received.emit("perception/map", {"ready": False, "error": "stale"})
        self.assertIn("stale", view.status.text())
        view.close()
        self.assertFalse(view.timer.isActive())

    def test_tracking_loss_retains_only_a_clearly_frozen_same_session_map(self) -> None:
        client = RecordedClient()
        view = LiveReconstructionView(client)
        client.resource_received.emit(
            "perception/map",
            {
                "ready": True,
                "vehicle_id": "vehicle_1",
                "session_id": "first",
                "timestamp_seconds": 5,
                "age_seconds": 0.1,
                "valid_depth_fraction": 0.5,
                "trajectory": [[0, 0, 0]],
                "landmarks": [[1, 2, 3]],
                "surfaces": [],
            },
        )
        client.resource_received.emit(
            "perception/map",
            {
                "ready": False,
                "session_id": "first",
                "tracking_state": 4,
            },
        )
        self.assertIn("FROZEN", view.status.text())
        self.assertTrue(view.canvas.dimmed)
        self.assertIsNotNone(view.canvas.mapping)
        client.resource_received.emit(
            "perception/map",
            {
                "ready": False,
                "session_id": "second",
                "tracking_state": 1,
            },
        )
        self.assertIsNone(view.canvas.mapping)
        self.assertNotIn("FROZEN", view.status.text())
        view.close()

    def test_reopened_view_displays_backend_history_as_frozen(self) -> None:
        client = RecordedClient()
        view = LiveReconstructionView(client)
        client.resource_received.emit(
            "perception/map",
            {
                "ready": False,
                "position_valid": False,
                "vehicle_id": "vehicle_1",
                "session_id": "retained",
                "map_frozen": True,
                "last_tracked_timestamp_seconds": 5,
                "timestamp_seconds": 8,
                "trajectory": [[0, 0, 0]],
                "landmarks": [[1, 2, 3]],
                "surfaces": [],
            },
        )
        self.assertIn("FROZEN", view.status.text())
        self.assertTrue(view.canvas.dimmed)
        self.assertIsNotNone(view.canvas.mapping)
        self.assertEqual(view.last_valid["timestamp_seconds"], 5)
        view.close()
