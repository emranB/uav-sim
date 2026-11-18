"""Finalized captures automatically render once, with bounded failure and cancellation."""

import threading
import time
import unittest
from unittest.mock import Mock

from common.logging.automatic_replay import AutomaticReplayRenders
from common.logging.replay_capture import ReplayCapture
from testing.unit import test_replay_export as export_fixtures
from testing.unit.test_replay_capture import Observations


class ControlledRenderer:
    def __init__(self, *, blocked=False, failure=False):
        self.blocked = blocked
        self.failure = failure
        self.calls = 0
        self.started = threading.Event()

    def availability(self):
        return {"available": True}

    def render(self, directory, cancelled, check, progress):
        self.calls += 1
        self.started.set()
        if self.failure:
            raise RuntimeError("Recorded scene assets unavailable")
        while self.blocked:
            check()
            cancelled.wait(0.01)
        progress(1.0)


class AutomaticReplayTests(unittest.TestCase):
    def setUp(self) -> None:
        export_fixtures.ReplayExportTests.setUp(self)

    def wait_state(self, queue, states):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            result = queue.snapshot()
            if result["state"] in states:
                return result
            time.sleep(0.01)
        self.fail(f"Automatic replay did not reach {states}: {queue.snapshot()}")

    def queue(self, renderer):
        self.jobs.renderer = renderer
        queue = AutomaticReplayRenders(self.store, self.jobs)
        self.addCleanup(queue.close)
        return queue

    def test_repeated_save_retains_one_job_across_restart(self) -> None:
        renderer = ControlledRenderer()
        queue = self.queue(renderer)
        first = queue.enqueue(self.identifier)
        second = queue.enqueue(self.identifier)
        self.assertTrue(second["duplicate"])
        result = self.wait_state(queue, {"completed"})
        self.assertEqual(renderer.calls, 1)
        self.assertEqual(first["capture_id"], result["capture_id"])
        self.assertTrue(self.store.inventory()[0]["retained"])
        queue.close()
        restarted = AutomaticReplayRenders(self.store, self.jobs)
        self.addCleanup(restarted.close)
        self.assertEqual(restarted.enqueue(self.identifier)["job_id"], result["job_id"])
        self.assertEqual(renderer.calls, 1)

    def test_render_failure_is_visible_and_not_implicitly_retried(self) -> None:
        renderer = ControlledRenderer(failure=True)
        queue = self.queue(renderer)
        queue.enqueue(self.identifier)
        result = self.wait_state(queue, {"failed"})
        self.assertIn("assets unavailable", result["error"])
        self.assertTrue(queue.enqueue(self.identifier)["duplicate"])
        self.assertEqual(renderer.calls, 1)
        self.store.manifest(self.identifier, verify_data=True)

    def test_cancel_preserves_source_and_does_not_repeat(self) -> None:
        renderer = ControlledRenderer(blocked=True)
        queue = self.queue(renderer)
        queue.enqueue(self.identifier)
        self.assertTrue(renderer.started.wait(2))
        queue.cancel()
        self.wait_state(queue, {"cancelled"})
        self.assertTrue(queue.enqueue(self.identifier)["duplicate"])
        self.assertEqual(renderer.calls, 1)
        self.store.manifest(self.identifier, verify_data=True)

    def test_queue_capacity_rejects_fifth_and_cancel_removes_pending_work(self) -> None:
        jobs = Mock()
        jobs.directory = self.jobs.directory
        jobs.snapshot.return_value = {"state": "rendering"}
        queue = AutomaticReplayRenders(self.store, jobs)
        self.addCleanup(queue.close)
        # Admission contract uses finalized manifests; no renderer or flight owner involved.
        self.store.manifest = Mock(return_value={"state": "stopped", "duration_wall_seconds": 1})
        self.store.keep = Mock()
        for index in range(4):
            queue.enqueue(f"capture-{index}")
        with self.assertRaisesRegex(RuntimeError, "queue is full"):
            queue.enqueue("capture-4")
        queue.cancel()
        self.assertEqual(queue.snapshot()["queued_count"], 0)
        jobs.start.assert_not_called()

    def test_stop_then_keep_dispatches_exactly_once_without_flight_dependencies(self) -> None:
        renderer = ControlledRenderer()
        self.jobs.renderer = renderer
        capture = ReplayCapture(Observations(), self.store, self.jobs)
        self.addCleanup(capture.close)
        result = capture.start(["atlas"])
        deadline = time.monotonic() + 2
        while (
            capture.snapshot()["vehicles"]["atlas"]["samples"] < 2 and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        capture.execute("replay/stop", {})
        deadline = time.monotonic() + 2
        while capture.snapshot()["state"] == "finalizing" and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(capture.execute("replay/stop", {})["duplicate"])
        capture.execute("replay/keep", {"capture_id": result["capture_id"]})
        self.wait_state(capture.automatic, {"completed"})
        self.assertEqual(renderer.calls, 1)

    def test_master_reset_closes_old_queue_and_allows_a_new_capture(self) -> None:
        renderer = ControlledRenderer()
        self.jobs.renderer = renderer
        capture = ReplayCapture(Observations(), self.store, self.jobs)
        self.addCleanup(capture.close)
        capture.close(reason="master_reset")
        capture.start(["atlas"])
        deadline = time.monotonic() + 2
        while (
            capture.snapshot()["vehicles"]["atlas"]["samples"] < 2 and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        capture.stop()
        self.wait_state(capture.automatic, {"completed"})
        self.assertEqual(renderer.calls, 1)

    def test_explicit_keep_policy_waits_for_save_before_rendering(self) -> None:
        renderer = ControlledRenderer()
        self.jobs.renderer = renderer
        capture = ReplayCapture(Observations(), self.store, self.jobs, render_on_stop=False)
        self.addCleanup(capture.close)
        result = capture.start(["atlas"])
        deadline = time.monotonic() + 2
        while capture.snapshot()["vehicles"]["atlas"]["samples"] < 2:
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.01)
        capture.stop()
        while capture.snapshot()["state"] == "finalizing":
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.01)
        self.assertFalse(renderer.started.wait(0.3))
        capture.execute("replay/keep", {"capture_id": result["capture_id"]})
        self.wait_state(capture.automatic, {"completed"})
        self.assertEqual(renderer.calls, 1)

    def test_save_title_is_used_once_and_keep_survives_render_admission_error(self) -> None:
        renderer = ControlledRenderer()
        self.jobs.renderer = renderer
        capture = ReplayCapture(Observations(), self.store, self.jobs)
        self.addCleanup(capture.close)
        first = capture.execute(
            "replay/keep", {"capture_id": self.identifier, "title": "Both GPS on"}
        )
        result = self.wait_state(capture.automatic, {"completed"})
        self.assertEqual(first["automatic_render"]["title"], "Both GPS on")
        repeated = capture.execute(
            "replay/keep", {"capture_id": self.identifier, "title": "Another title"}
        )
        self.assertTrue(repeated["automatic_render"]["duplicate"])
        self.assertEqual(repeated["automatic_render"]["job_id"], result["job_id"])
        self.assertEqual(renderer.calls, 1)
        invalid = capture.execute(
            "replay/keep", {"capture_id": self.identifier, "title": "bad\nname"}
        )
        self.assertTrue(invalid["retained"])
        self.assertIn("printable", invalid["automatic_render_error"])

    def test_stop_retains_capture_when_automatic_queue_admission_is_rejected(self) -> None:
        self.jobs.renderer = ControlledRenderer()
        capture = ReplayCapture(Observations(), self.store, self.jobs)
        self.addCleanup(capture.close)
        capture.automatic.enqueue = Mock(side_effect=RuntimeError("Automatic replay queue is full"))
        started = capture.start(["atlas"])
        deadline = time.monotonic() + 2
        while (
            capture.snapshot()["vehicles"]["atlas"]["samples"] < 2 and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        capture.stop()
        deadline = time.monotonic() + 2
        while not capture.snapshot().get("automatic_render_error") and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIn("queue is full", capture.snapshot()["automatic_render_error"])
        self.assertTrue(capture.snapshot()["automatic_capture_retained"])
        self.store.manifest(started["capture_id"], verify_data=True)
        self.assertTrue(
            next(
                item
                for item in self.store.inventory()
                if item["capture_id"] == started["capture_id"]
            )["retained"]
        )
