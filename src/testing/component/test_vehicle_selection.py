"""Desktop targeting stays explicit across focused and collection operations."""

import json
import tempfile
import unittest
from unittest.mock import Mock, patch

from PySide6.QtNetwork import QNetworkReply
from PySide6.QtWidgets import QApplication

from desktop.client import SimulationClient


class VehicleSelectionTests(unittest.TestCase):
    def test_explicit_shortcut_targets_and_results_survive_focus_change(self) -> None:
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict("os.environ", {"OUTPUT_DIR": directory}),
        ):
            client = SimulationClient()
            client.timer.stop()
            client.network = Mock()
            client.fetch = Mock()
            client.fleet_ids = ["vehicle_1", "vehicle_2"]
            reply = client.network.post.return_value
            reply.readAll.return_value = json.dumps(
                {
                    "preflight_passed": True,
                    "results": {
                        "vehicle_1": {"accepted": True},
                        "vehicle_2": {"accepted": False, "error": "ACK denied"},
                    },
                }
            ).encode()
            received = []
            client.target_operation_result.connect(lambda request, result: received.append(result))
            targets = ["vehicle_1", "vehicle_2"]
            client.execute_targets("flight/start", targets, "request-1")
            targets.clear()
            client.vehicle_id = "vehicle_2"
            reply.finished.connect.call_args.args[0]()
            self.assertEqual(received[0]["target_ids"], ["vehicle_1", "vehicle_2"])
            self.assertTrue(received[0]["results"]["vehicle_1"]["accepted"])
            self.assertEqual(received[0]["results"]["vehicle_2"]["error"], "ACK denied")
            body = json.loads(client.network.post.call_args.args[1])
            self.assertEqual(body["target_ids"], ["vehicle_1", "vehicle_2"])
            client.deleteLater()

    def test_replay_capture_scope_and_global_poll_survive_focus_change(self) -> None:
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict("os.environ", {"OUTPUT_DIR": directory}),
        ):
            client = SimulationClient()
            client.timer.stop()
            client.network = Mock()
            reply = client.network.post.return_value
            reply.readAll.return_value = b'{"accepted":true,"state":"capturing"}'
            reply.error.return_value = QNetworkReply.NetworkError.NoError
            received = []
            client.replay_operation_result.connect(lambda request, result: received.append(result))
            targets = ["vehicle_1"]
            client.execute_replay("replay/start", {"target_ids": targets}, "capture-request")
            targets.append("vehicle_2")
            client.vehicle_id = "vehicle_2"
            reply.finished.connect.call_args.args[0]()
            self.assertEqual(received[0]["state"], "capturing")
            body = json.loads(client.network.post.call_args.args[1])
            self.assertEqual(body, {"target_ids": ["vehicle_1"]})
            self.assertNotIn("vehicle_id", client.network.get.call_args.args[0].url().toString())
            client.vehicle_id = "vehicle_1"
            client.fetch("replay")
            self.assertEqual(client.network.get.call_count, 1)
            resources = []
            client.resource_received.connect(lambda name, payload: resources.append(name))
            get_reply = client.network.get.return_value
            get_reply.readAll.return_value = b'{"capture":{"state":"capturing"},"items":[]}'
            get_reply.finished.connect.call_args.args[0]()
            self.assertEqual(resources, ["replay"])
            client.deleteLater()

    def test_shutdown_aborts_replies_and_prevents_new_polling(self) -> None:
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict("os.environ", {"OUTPUT_DIR": directory}),
        ):
            client = SimulationClient()
            client.network = Mock()
            reply = Mock()
            client.network.findChildren.return_value = [reply]
            client.close()
            client.refresh()
            client.execute("start")
            self.assertFalse(client.timer.isActive())
            reply.abort.assert_called_once()
            client.network.post.assert_not_called()
            client.network.get.assert_not_called()
            client.deleteLater()

    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_collection_fault_and_focused_camera_use_different_explicit_targets(self) -> None:
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict("os.environ", {"OUTPUT_DIR": directory}),
        ):
            client = SimulationClient()
            client.timer.stop()
            client.network = Mock()
            client.fleet_ids = ["vehicle_1", "vehicle_2"]
            client.select_vehicle("vehicle_2", apply_to_all=True)
            client.execute("interference/link", {"loss_percent": 25})
            body = json.loads(client.network.post.call_args.args[1])
            self.assertEqual(body["target_ids"], ["vehicle_1", "vehicle_2"])
            client.execute("viewer/camera", {"preset": "chase"})
            body = json.loads(client.network.post.call_args.args[1])
            self.assertEqual(body["target_ids"], ["vehicle_2"])
            self.assertIn(
                "vehicle_id=vehicle_2",
                client.resource_url("logs?source=px4", "vehicle_2").toString(),
            )
            client.deleteLater()

    def test_workspace_targets_survive_inspection_changes_and_scope_edits(self) -> None:
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict("os.environ", {"OUTPUT_DIR": directory}),
        ):
            client = SimulationClient()
            self.addCleanup(client.close)
            client.timer.stop()
            client.network = Mock()
            client.network.findChildren.return_value = []
            client.fleet_ids = ["vehicle_1", "vehicle_2"]
            client.set_action_targets(("vehicle_1",))
            client.select_vehicle("vehicle_2")
            client.execute("sensors/gnss", {"enabled": False})
            self.assertEqual(
                json.loads(client.network.post.call_args.args[1])["target_ids"], ["vehicle_1"]
            )
            client.execute("viewer/camera", {"preset": "wide"})
            self.assertEqual(
                json.loads(client.network.post.call_args.args[1])["target_ids"], ["vehicle_2"]
            )
            client.execute("mission", {"fileType": "Plan"})
            self.assertEqual(
                json.loads(client.network.post.call_args.args[1])["target_ids"], ["vehicle_1"]
            )

    def test_empty_workspace_targets_never_send_mutating_vehicle_commands(self) -> None:
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict("os.environ", {"OUTPUT_DIR": directory}),
        ):
            client = SimulationClient()
            self.addCleanup(client.close)
            client.timer.stop()
            client.network = Mock()
            client.network.findChildren.return_value = []
            client.set_action_targets(())
            failures = []
            client.operation_failed.connect(lambda operation, reason: failures.append(reason))
            for operation in ("start", "reset", "sensors/gnss", "navigation/mission", "mission"):
                client.execute(operation)
            client.network.post.assert_not_called()
            self.assertEqual(len(failures), 5)
            client.execute("resume")
            self.assertTrue(client.network.post.called)

    def test_unknown_vehicle_never_changes_selection(self) -> None:
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict("os.environ", {"OUTPUT_DIR": directory}),
        ):
            client = SimulationClient()
            client.timer.stop()
            with self.assertRaises(ValueError):
                client.select_vehicle("missing")
            self.assertEqual(client.vehicle_id, "vehicle_1")
            client.deleteLater()
