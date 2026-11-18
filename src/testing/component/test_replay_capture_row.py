"""Replay intent stays separate from later focus/selection and flight commands."""

import time
import unittest

from PySide6.QtWidgets import QApplication

from desktop.replay_capture import ReplayCaptureRow
from testing.component.test_panels import RecordedClient


class ReplayCaptureRowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.client = RecordedClient()
        self.row = ReplayCaptureRow(self.client)
        self.addCleanup(self.row.close)
        self.client.resource_received.emit("replay", {"capture": {"state": "idle"}, "items": []})

    def test_start_waits_for_writer_ack_and_stop_ignores_new_selection(self) -> None:
        self.assertFalse(self.row.start_button.isEnabled())
        self.row.set_targets(("atlas",))
        self.row.start_button.click()
        operation, body = self.client.requests[-1]
        self.assertEqual(operation, "replay/start")
        self.assertEqual(body["target_ids"], ["atlas"])
        self.assertNotIn("● Capturing", self.row.indicator.text())
        self.row.set_targets(("boreal",))
        self.client.replay_operation_result.emit(
            body["request_id"],
            {
                "accepted": True,
                "state": "capturing",
                "vehicle_ids": ["atlas"],
                "vehicle_names": {"atlas": "Atlas"},
                "duration_wall_seconds": 3,
            },
        )
        self.assertIn("Atlas", self.row.indicator.text())
        self.assertNotIn("Boreal", self.row.indicator.text())
        self.row.stop_button.click()
        operation, body = self.client.requests[-1]
        self.assertEqual(operation, "replay/stop")
        self.assertNotIn("target_ids", body)
        self.assertFalse(
            any(operation in {"stop", "flight/land"} for operation, _ in self.client.requests)
        )

    def test_failed_start_and_stale_state_never_claim_active_recording(self) -> None:
        self.row.set_targets(("atlas",))
        self.row.start_button.click()
        _, body = self.client.requests[-1]
        self.client.replay_operation_result.emit(
            body["request_id"],
            {
                "accepted": False,
                "error": "disk full",
            },
        )
        self.assertIn("disk full", self.row.feedback.text())
        self.assertNotIn("● Capturing", self.row.indicator.text())
        self.row.last_received = time.monotonic() - 10
        self.row.refresh()
        self.assertIn("unknown", self.row.indicator.text())
        self.assertFalse(self.row.start_button.isEnabled())
        self.assertFalse(self.row.render_button.isEnabled())

    def test_keep_uses_finalized_capture_not_flight_selection(self) -> None:
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {"state": "stopped"},
                "items": [
                    {
                        "capture_id": "capture-1",
                        "state": "stopped",
                        "retained": False,
                        "vehicle_names": {"atlas": "Atlas"},
                    }
                ],
            },
        )
        self.row.set_targets(("boreal",))
        self.row.keep_button.click()
        operation, body = self.client.requests[-1]
        self.assertEqual(operation, "replay/keep")
        self.assertEqual(body["capture_id"], "capture-1")
        self.assertNotIn("target_ids", body)

    def test_offline_preview_scope_and_progress_do_not_replace_active_capture(self) -> None:
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {"state": "capturing", "vehicle_ids": ["atlas"]},
                "items": [
                    {"capture_id": "saved-1", "state": "stopped", "duration_wall_seconds": 10}
                ],
                "renderer": {"available": True},
                "offline_job": {"state": "idle"},
            },
        )
        self.row.preview_button.click()
        operation, body = self.client.requests[-1]
        self.assertEqual(operation, "replay/render")
        self.assertEqual(body["capture_id"], "saved-1")
        self.assertEqual(body["start_offset_seconds"], 0)
        self.assertEqual(body["end_offset_seconds"], 2)
        self.client.replay_operation_result.emit(
            body["request_id"],
            {
                "accepted": True,
                "job_id": "job-1",
                "state": "queued",
                "progress": 0,
            },
        )
        self.assertEqual(self.row.capture["state"], "capturing")
        self.assertTrue(self.row.stop_button.isEnabled())
        self.assertTrue(self.row.cancel_button.isEnabled())

    def replay_status(self, state: str, *, finalized: bool = False) -> None:
        items = [{"capture_id": "older", "state": "interrupted", "duration_wall_seconds": 1.5}]
        if finalized:
            items.insert(
                0,
                {
                    "capture_id": "new-atlas",
                    "state": "stopped",
                    "duration_wall_seconds": 30,
                },
            )
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {"capture_id": "new-atlas", "state": state},
                "items": items,
            },
        )

    def test_newly_finalized_capture_becomes_keep_and_export_selection(self) -> None:
        self.replay_status("capturing")
        self.assertEqual(self.row.saved.currentData(), "older")
        self.replay_status("finalizing")
        # Finalization and inventory publication may arrive in separate polls.
        self.replay_status("stopped")
        self.replay_status("stopped", finalized=True)
        self.assertEqual(self.row.saved.currentData(), "new-atlas")
        self.assertEqual(self.row.to_seconds.value(), 30)
        self.row.keep_button.click()
        self.assertEqual(self.client.requests[-1][1]["capture_id"], "new-atlas")

    def test_deliberate_older_choice_during_capture_survives_finalization(self) -> None:
        self.replay_status("capturing")
        self.row.saved.activated.emit(0)
        self.replay_status("capturing")
        self.replay_status("stopped", finalized=True)
        self.assertEqual(self.row.saved.currentData(), "older")
        self.assertEqual(self.row.to_seconds.value(), 1.5)

    def test_deliberate_older_choice_after_finalization_survives_polling(self) -> None:
        self.replay_status("capturing")
        self.replay_status("stopped", finalized=True)
        self.row.saved.setCurrentIndex(self.row.saved.findData("older"))
        self.row.saved.activated.emit(self.row.saved.currentIndex())
        self.replay_status("stopped", finalized=True)
        self.assertEqual(self.row.saved.currentData(), "older")

    def test_historical_stopped_status_does_not_override_saved_choice(self) -> None:
        self.replay_status("stopped")
        self.replay_status("stopped", finalized=True)
        self.assertEqual(self.row.saved.currentData(), "older")

    def test_automatic_queue_status_and_completed_mp4_are_visible(self) -> None:
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {"state": "stopped"},
                "items": [],
                "offline_job": {"state": "idle"},
                "automatic_job": {"capture_id": "capture-1", "state": "queued", "queued_count": 1},
            },
        )
        self.assertIn("queued", self.row.job_status.text())
        self.assertTrue(self.row.cancel_button.isEnabled())
        self.assertFalse(self.row.play_button.isEnabled())
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {"state": "stopped"},
                "items": [],
                "offline_job": {"state": "idle"},
                "automatic_job": {
                    "capture_id": "capture-1",
                    "state": "completed",
                    "progress": 1,
                    "video_path": "/outputs/replay-jobs/job-1/mission-replay.mp4",
                },
            },
        )
        self.assertIn("completed", self.row.job_status.text())
        self.assertTrue(self.row.play_button.isEnabled())
        self.assertFalse(self.row.cancel_button.isEnabled())

    def test_automatic_failure_does_not_claim_a_playable_movie(self) -> None:
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {"state": "stopped"},
                "items": [],
                "automatic_job": {"state": "failed", "error": "Recorded model revision differs"},
            },
        )
        self.assertIn("Recorded model revision differs", self.row.job_status.text())
        self.assertFalse(self.row.play_button.isEnabled())

    def test_save_mission_queues_current_finalized_run_and_preserves_history_selection(self):
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {"capture_id": "current", "state": "stopped", "vehicle_ids": ["atlas"]},
                "items": [
                    {"capture_id": "older", "state": "stopped"},
                    {"capture_id": "current", "state": "stopped"},
                ],
            },
        )
        self.row.saved.setCurrentIndex(0)
        self.assertTrue(self.row.queue_current_saved_run("atlas"))
        operation, body = self.client.requests[-1]
        self.assertEqual(operation, "replay/keep")
        self.assertEqual(body["capture_id"], "current")
        self.assertEqual(self.row.saved.currentData(), "older")

    def test_save_mission_does_not_finalize_active_or_unrelated_capture(self):
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {
                    "capture_id": "current",
                    "state": "capturing",
                    "vehicle_ids": ["atlas"],
                },
                "items": [],
            },
        )
        self.assertFalse(self.row.queue_current_saved_run("atlas"))
        self.row.capture["state"] = "stopped"
        self.assertFalse(self.row.queue_current_saved_run("boreal"))
        self.assertFalse(self.client.requests)

    def test_rendering_scope_stays_explicit_when_another_saved_capture_is_selected(self) -> None:
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {"state": "idle"},
                "items": [
                    {
                        "capture_id": "capture-selected",
                        "state": "stopped",
                        "vehicle_names": {"boreal": "Boreal"},
                    },
                    {
                        "capture_id": "capture-rendered",
                        "state": "stopped",
                        "vehicle_names": {"atlas": "Atlas"},
                    },
                ],
                "automatic_job": {
                    "capture_id": "capture-rendered",
                    "state": "rendering",
                    "progress": 0.3,
                    "queued_count": 1,
                    "title": "Recorded mission",
                },
            },
        )
        self.row.saved.setCurrentIndex(0)
        self.assertIn("rendered", self.row.job_status.text())
        self.assertIn("Atlas", self.row.job_status.text())
        self.assertIn("1 queued", self.row.job_status.text())
        self.assertNotIn("Boreal", self.row.job_status.text())
        self.assertEqual(self.row.saved.currentData(), "capture-selected")
        self.assertEqual(self.row.job_status.toolTip(), "Rendering capture: capture-rendered")

    def test_stop_queue_rejection_shows_retention_and_failure_reason(self) -> None:
        self.client.resource_received.emit(
            "replay",
            {
                "capture": {
                    "state": "stopped",
                    "automatic_render_error": "Queue is full",
                    "automatic_capture_retained": True,
                },
                "items": [],
            },
        )
        self.assertIn("Automatic 3D replay not queued: Queue is full", self.row.details.text())
        self.assertIn("Capture retained", self.row.details.text())
        self.assertFalse(self.row.play_button.isEnabled())
