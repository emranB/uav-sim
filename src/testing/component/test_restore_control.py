"""Desktop restore progress is acknowledged and resets only presentation after completion."""

import unittest

from PySide6.QtWidgets import QApplication

from desktop.restore import RestoreControl
from testing.component.test_panels import RecordedClient


class RestoreControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_whole_fleet_scope_pending_and_error_are_visible(self) -> None:
        client = RecordedClient()
        row = RestoreControl(client)
        self.addCleanup(row.close)
        self.assertFalse(row.button.isEnabled())
        client.resource_received.emit(
            "restore",
            {
                "restore": {"state": "idle", "active": False},
                "availability": {"available": True, "vehicle_ids": ["atlas", "boreal"]},
            },
        )
        self.assertIn("atlas, boreal", row.button.toolTip())
        row.button.click()
        operation, body = client.requests[-1]
        self.assertEqual(operation, "restore/start")
        self.assertNotIn("target_ids", body)
        self.assertFalse(row.button.isEnabled())
        self.assertNotIn("defaults restored", row.feedback.text())
        client.restore_operation_result.emit(
            body["request_id"],
            {
                "accepted": False,
                "error": "Landing rejected",
            },
        )
        self.assertIn("Landing rejected", row.feedback.text())

    def test_completion_signal_emits_once_and_never_for_readiness_only(self) -> None:
        client = RecordedClient()
        row = RestoreControl(client)
        self.addCleanup(row.close)
        completions = []
        client.restore_completed.connect(completions.append)
        payload = {
            "restore": {
                "state": "restarting_services",
                "active": True,
                "restore_id": "restore-1",
                "vehicle_ids": ["atlas"],
            },
            "availability": {"available": True},
        }
        client.resource_received.emit("restore", payload)
        self.assertEqual(completions, [])
        payload["restore"].update(state="completed", active=False, defaults={"target_ids": []})
        client.resource_received.emit("restore", payload)
        client.resource_received.emit("restore", payload)
        self.assertEqual(len(completions), 1)
