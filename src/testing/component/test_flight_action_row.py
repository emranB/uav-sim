"""Shortcut target selection, immutable dispatch scope and rendered placement."""

import unittest
from unittest.mock import patch

from PySide6.QtCore import QPoint
from PySide6.QtWidgets import QApplication, QWidget

from common.simulation_manager.flight_actions import flight_action_status
from desktop.flight_actions import FlightActionRow
from desktop.window import MainWindow
from testing.component.test_panels import RecordedClient


class FlightActionRowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.client = RecordedClient()
        self.row = FlightActionRow(self.client)
        self.addCleanup(self.row.close)
        self.fleet = [
            {
                "vehicle_id": identity,
                "label": name,
                "flight_actions": flight_action_status(
                    {"connected": True, "armed": False, "ready": True}
                ),
            }
            for identity, name in (("vehicle_1", "Atlas"), ("vehicle_2", "Boreal"))
        ]
        self.client.resource_received.emit("vehicles", {"items": self.fleet})

    def test_default_none_and_explicit_atlas_boreal_both_selection(self) -> None:
        self.assertFalse(self.row.buttons["start"].isEnabled())
        self.assertEqual(self.row.selected_ids, set())
        for targets in (("vehicle_1",), ("vehicle_2",), ("vehicle_1", "vehicle_2")):
            self.row.clear_selection()
            for target in targets:
                self.row.checkboxes[target].setChecked(True)
            self.row.dispatch("start")
            operation, body = self.client.requests[-1]
            self.assertEqual(operation, "flight/start")
            self.assertEqual(body["target_ids"], list(targets))
            self.client.target_operation_result.emit(
                body["request_id"], {"results": {target: {"accepted": True} for target in targets}}
            )

    def test_group_unready_is_visible_and_no_command_is_dispatched(self) -> None:
        self.fleet[1]["flight_actions"] = flight_action_status(
            {"connected": False, "armed": False, "ready": False}
        )
        self.client.resource_received.emit("vehicles", {"items": self.fleet})
        self.row.select_all()
        self.assertFalse(self.row.buttons["start"].isEnabled())
        self.assertIn("Boreal", self.row.readiness.text())
        self.assertIn("connection", self.row.buttons["start"].toolTip())
        self.row.dispatch("start")
        self.assertEqual(self.client.requests, [])

    def test_pending_selection_change_keeps_original_targets_and_partial_feedback(self) -> None:
        self.row.select_all()
        self.row.dispatch("start")
        _, body = self.client.requests[-1]
        self.row.clear_selection()
        self.client.target_operation_result.emit(
            body["request_id"],
            {
                "results": {
                    "vehicle_1": {"accepted": True},
                    "vehicle_2": {"accepted": False, "error": "Native ACK denied"},
                }
            },
        )
        self.assertEqual(body["target_ids"], ["vehicle_1", "vehicle_2"])
        self.assertIn("Atlas: acknowledged", self.row.feedback.text())
        self.assertIn("Boreal: Native ACK denied", self.row.feedback.text())
        self.assertIn("0 selected", self.row.targets.text())
        self.assertFalse(self.row.buttons["start"].isEnabled())

    def test_new_vehicle_is_not_silently_added_to_selection(self) -> None:
        self.row.checkboxes["vehicle_1"].setChecked(True)
        self.client.resource_received.emit("vehicles", {"items": self.fleet})
        self.assertEqual(self.row.selected_ids, {"vehicle_1"})

    def test_unsupported_actions_remain_disabled_with_reasons(self) -> None:
        self.row.select_all()
        for action in ("takeoff", "return", "pause"):
            self.assertFalse(self.row.buttons[action].isEnabled())
            self.assertIn("unavailable", self.row.buttons[action].toolTip())

    def test_rendered_row_is_below_qgc(self) -> None:
        with patch("desktop.window.SimulationClient", return_value=RecordedClient()):
            window = MainWindow()
            window.resize(1850, 1126)
            window.show()
            self.application.processEvents()
            qgc = window.findChild(QWidget, "qgcEmbeddedView")
            row = window.findChild(FlightActionRow)
            qgc_bottom = qgc.mapTo(window, QPoint(0, qgc.height())).y()
            row_top = row.mapTo(window, QPoint(0, 0)).y()
            self.assertGreaterEqual(row_top, qgc_bottom)
            self.assertGreater(qgc.height(), row.height())
            window.close()


def test_telemetry_refresh_preserves_open_target_menu_actions():
    _app = QApplication.instance() or QApplication([])
    client = RecordedClient()
    row = FlightActionRow(client)
    payload = {
        "items": [
            {
                "vehicle_id": identity,
                "label": label,
                "flight_actions": flight_action_status({"connected": True, "ready": True}),
            }
            for identity, label in (("vehicle_1", "Atlas"), ("vehicle_2", "Boreal"))
        ]
    }
    try:
        row.receive("vehicles", payload)
        original = row.checkboxes["vehicle_2"]
        row.receive("vehicles", payload)
        assert row.checkboxes["vehicle_2"] is original
        original.setChecked(True)
        row.receive("vehicles", payload)
        assert row.selected_ids == {"vehicle_2"}
        assert original.isChecked()
    finally:
        row.close()
