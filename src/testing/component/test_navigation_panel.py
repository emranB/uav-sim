"""Focused navigation intent, unavailable state and visible measured-space layers."""

import unittest

from PySide6.QtWidgets import QApplication

from control_panel.navigation.panel import NavigationPanel
from testing.component.test_panels import RecordedClient


class NavigationPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_focused_start_is_explicit_and_active_run_disables_start(self) -> None:
        client = RecordedClient()
        client.vehicle_id = "vehicle_2"
        panel = NavigationPanel(client)
        panel.receive(
            "navigation",
            {
                "vehicle_id": "vehicle_2",
                "available": False,
                "state": "unavailable",
                "error": "sensor unavailable",
            },
        )
        self.assertFalse(panel.start.isEnabled())
        self.assertIn("unavailable", panel.state.text())
        panel.receive(
            "navigation",
            {"vehicle_id": "vehicle_2", "available": True, "active": False, "state": "idle"},
        )
        panel.start.click()
        self.assertEqual(
            client.requests[-1],
            (
                "navigation/start",
                {
                    "vehicle_id": "vehicle_2",
                    "goal_offset_enu_m": [0.0, 14.0],
                    "obstacle_mode": "range",
                },
            ),
        )
        panel.receive(
            "navigation",
            {
                "vehicle_id": "vehicle_2",
                "available": True,
                "active": True,
                "state": "blocked",
                "observed_free_enu_m": [[0, 1]],
                "occupied_enu_m": [[0, 4]],
                "path_enu_m": [[0, 0], [0, 1]],
            },
        )
        self.assertFalse(panel.start.isEnabled())
        panel.map.resize(500, 250)
        self.assertFalse(panel.map.grab().isNull())
        self.assertEqual(panel.map.snapshot["observed_free_enu_m"], [[0, 1]])
        panel.selection_changed("vehicle_1")
        self.assertFalse(panel.map.snapshot)
        panel.timer.stop()
        panel.close()

    def test_camera_selection_is_explicit_and_locked_during_flight(self) -> None:
        client = RecordedClient()
        client.vehicle_id = "vehicle_1"
        panel = NavigationPanel(client)
        snapshot = {
            "vehicle_id": "vehicle_1",
            "available": True,
            "state": "idle",
            "supported_obstacle_modes": ["range", "camera", "fused"],
        }
        panel.receive("navigation", snapshot)
        panel.mode.setCurrentIndex(panel.mode.findData("camera"))
        self.assertFalse(panel.start.isEnabled())
        self.assertIn("2–10m", panel.goal_constraint.text())
        panel.north.setValue(9)
        self.assertTrue(panel.start.isEnabled())
        panel.start.click()
        self.assertEqual(client.requests[-1][1]["obstacle_mode"], "camera")
        panel.receive("navigation", {**snapshot, "state": "navigating", "active": True})
        self.assertFalse(panel.mode.isEnabled())
        self.assertFalse(panel.start.isEnabled())
        panel.timer.stop()
        panel.close()

    def test_api_started_run_displays_acknowledged_sensing_and_goal_without_commands(self) -> None:
        client = RecordedClient()
        client.vehicle_id = "vehicle_1"
        panel = NavigationPanel(client)
        panel.receive(
            "navigation",
            {
                "vehicle_id": "vehicle_1",
                "available": True,
                "state": "navigating",
                "active": True,
                "supported_obstacle_modes": ["range", "camera", "fused"],
                "obstacle_mode": "fused",
                "goal_offset_enu_m": [-1, 9],
            },
        )
        self.assertEqual(panel.mode.currentData(), "fused")
        self.assertEqual((panel.east.value(), panel.north.value()), (-1, 9))
        self.assertFalse(panel.mode.isEnabled())
        self.assertFalse(panel.north.isEnabled())
        self.assertFalse(client.requests)
        panel.timer.stop()
        panel.close()
