"""Native GNSS feedback keeps dispatch identity while selection and Inspect change."""

import json
import tempfile
import unittest
from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget

from common.simulation_manager.flight_actions import flight_action_status
from desktop.flight_actions import FlightActionRow
from desktop.replay_capture import ReplayCaptureRow
from testing.component.test_panels import RecordedClient
from testing.simulation.combined_presentation import CombinedPresentationHarness


class GnssRecordedClient(RecordedClient):
    gnss_operation_result = Signal(str, dict)

    def execute_gnss_targets(
        self, targets: tuple[str, ...], enabled: bool, request_id: str
    ) -> None:
        self.requests.append(
            (
                "sensors/gnss",
                {
                    "target_ids": list(targets),
                    "enabled": enabled,
                    "request_id": request_id,
                },
            )
        )


class CombinedGnssTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.window = QMainWindow()
        self.client = GnssRecordedClient()
        self.window.client = self.client
        central = QWidget()
        layout = QVBoxLayout(central)
        self.row = FlightActionRow(self.client)
        self.capture = ReplayCaptureRow(self.client)
        layout.addWidget(self.row)
        layout.addWidget(self.capture)
        self.window.setCentralWidget(central)
        self.harness = CombinedPresentationHarness(self.window, self.root)
        self.addCleanup(self.window.close)
        self.addCleanup(self.harness.close)
        self.client.resource_received.emit(
            "vehicles",
            {
                "items": [
                    {
                        "vehicle_id": identity,
                        "label": label,
                        "flight_actions": flight_action_status({"connected": True, "ready": True}),
                    }
                    for identity, label in (("vehicle_1", "Atlas"), ("vehicle_2", "Boreal"))
                ]
            },
        )
        self.client.requests.clear()

    def ack(self, identifier: str) -> dict:
        for future in self.harness.futures:
            future.result(timeout=1)
        return json.loads((self.root / (identifier + "-ack.json")).read_text())

    def test_ack_uses_original_targets_and_value_after_selection_and_inspect_change(self) -> None:
        self.harness.apply_request(
            {
                "kind": "gnss",
                "request_id": "original",
                "target_ids": ["vehicle_1"],
                "enabled": False,
            }
        )
        self.assertFalse(self.harness.gnss_apply.isEnabled())
        self.assertFalse(self.harness.gnss_enabled.isEnabled())
        self.row.clear_selection()
        self.row.checkboxes["vehicle_2"].setChecked(True)
        self.client.select_vehicle("vehicle_2")
        self.harness.gnss_enabled.setChecked(True)
        self.client.gnss_operation_result.emit("original", {"accepted": True})
        result = self.ack("original")
        self.assertEqual(result["target_ids"], ["vehicle_1"])
        self.assertFalse(result["requested_enabled"])
        self.assertTrue(result["accepted"])
        self.assertTrue(self.harness.gnss_apply.isEnabled())

    def test_duplicate_pending_and_unrelated_ack_cannot_overwrite_dispatch(self) -> None:
        self.harness.apply_request(
            {"kind": "gnss", "request_id": "first", "target_ids": ["vehicle_1"], "enabled": False}
        )
        self.harness.apply_request(
            {"kind": "gnss", "request_id": "second", "target_ids": ["vehicle_2"], "enabled": True}
        )
        self.assertFalse(self.ack("second")["accepted"])
        self.client.gnss_operation_result.emit("unrelated", {"accepted": True})
        self.assertIsNotNone(self.harness.gnss_pending)
        writes = [body for operation, body in self.client.requests if operation == "sensors/gnss"]
        self.assertEqual(len(writes), 1)
        self.client.gnss_operation_result.emit("first", {"accepted": False, "error": "Denied"})
        self.assertFalse(self.ack("first")["accepted"])
        self.assertEqual(self.ack("first")["target_ids"], ["vehicle_1"])
        self.assertIsNone(self.harness.gnss_pending)
