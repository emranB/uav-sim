"""Stale and changed camera targets never retain a healthy preview."""

import unittest

from PySide6.QtWidgets import QApplication

from control_panel.sensors.live import LiveStereoView
from testing.component.test_panels import RecordedClient


class LiveCameraTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_target_change_clears_pixels_and_stale_state_stops_requests(self) -> None:
        client = RecordedClient()
        view = LiveStereoView(client)
        view.show()
        client.resource_received.emit(
            "perception",
            {
                "vehicle_id": "vehicle_1",
                "camera": {
                    "ready": True,
                    "simulation_timestamp_ns": 1,
                    "recorded_pairs": 3,
                },
            },
        )
        client.camera_image_received.emit("left", b"P6\n1 1\n255\n\xff\x00\x00")
        self.assertFalse(view.views["left"].pixmap().isNull())
        view.refresh()
        self.assertEqual(client.requests[-2:], [("camera/left", None), ("camera/right", None)])
        client.selection_changed.emit("vehicle_2")
        self.assertIn("unavailable", view.views["left"].text())
        count = len(client.requests)
        view.refresh()
        self.assertEqual(len(client.requests), count)
        view.close()

    def test_sensor_health_agrees_with_the_rendered_camera_capability(self) -> None:
        from common.simulation_manager.perception import vehicle_perception
        from control_panel.sensors.panel import SensorsPanel

        client = RecordedClient()
        panel = SensorsPanel(client)
        inventory = vehicle_perception(stereo_enabled=True).snapshot("vehicle_1")
        inventory["camera"] = {"ready": True, "simulation_timestamp_ns": 1, "recorded_pairs": 3}
        client.resource_received.emit("perception", inventory)
        client.telemetry_received.emit(
            {"connected": False, "vehicle_id": "vehicle_1", "run_id": "controller-1"}
        )
        self.assertEqual(panel.table.item(4, 1).text(), "Stale / unknown")
        client.telemetry_received.emit(
            {
                "connected": True,
                "vehicle_id": "vehicle_1",
                "run_id": "controller-1",
                "health": [
                    {
                        "system": "Camera",
                        "vehicle_id": "vehicle_1",
                        "controller_generation": "controller-1",
                        "state": "healthy",
                        "reason": "Fresh measured stereo pair",
                        "evidence_age_seconds": 0.1,
                    }
                ],
            }
        )
        self.assertEqual(panel.table.item(4, 1).text(), "Healthy")
        self.assertEqual(panel.table.item(4, 2).text(), "Rendered stereo")
        self.assertNotIn("No stereo camera", panel.details.text())
        panel.close()
