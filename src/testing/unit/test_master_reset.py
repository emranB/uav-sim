"""Master Reset requires acknowledged landing and fresh disarm before restart."""

import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock

from common.lifecycle.restore import MasterReset


class RestoreOwner:
    def __init__(self) -> None:
        self.calls = []
        self.connected = True
        self.armed = True
        self.generation = "old"
        self.available = True
        self.reject_landing = False
        self.fail_restart = False
        self.restarted = False
        self.block = None

    def describe(self) -> dict:
        return {
            "available": self.available,
            "reason": "Missing UAV owner",
            "vehicle_ids": ["atlas", "boreal"],
            "services": ["gazebo"],
        }

    def enter(self) -> None:
        self.calls.append("enter")

    def leave(self) -> None:
        self.calls.append("leave")

    def prepare(self, identities) -> None:
        self.calls.append("resume_and_recover_link")
        if self.block:
            self.block.wait(2)

    def land(self, identity) -> dict:
        self.calls.append(f"land:{identity}")
        if self.reject_landing:
            return {"accepted": False, "error": "Landing rejected"}
        self.armed = False
        return {"accepted": True}

    def state(self, identity) -> dict:
        return {
            "connected": self.connected,
            "armed": self.armed,
            "run_id": self.generation,
            "heartbeat_age_seconds": 0.1,
            "navigation_active": False,
            "ready": True,
        }

    def restart(self, identities) -> dict:
        self.calls.append("restart")
        self.restarted = True
        if self.fail_restart:
            raise RuntimeError("Gazebo did not acknowledge restart")
        self.generation = "new"
        return {"controller_generations": {identity: "new" for identity in identities}}

    def defaults(self) -> dict:
        self.calls.append("defaults")
        return {"target_ids": []}


class MasterResetTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.owner = RestoreOwner()
        self.capture = Mock()
        self.reset = MasterReset(
            self.owner, Path(self.temporary.name), self.capture, landing_timeout_seconds=0.7
        )
        self.addCleanup(self.reset.close)

    def finished(self) -> dict:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            status = self.reset.snapshot()
            if not status["active"]:
                return status
            time.sleep(0.01)
        self.fail("Reset did not finish")

    def test_landing_capture_finalization_restart_and_defaults_are_ordered(self) -> None:
        self.capture.close.side_effect = lambda **kw: self.owner.calls.append("capture_finalized")
        saved = Path(self.temporary.name) / "saved-mission.plan"
        saved.write_bytes(b"saved history preserved")
        self.reset.start()
        status = self.finished()
        self.assertEqual(status["state"], "completed")
        self.assertLess(
            self.owner.calls.index("land:boreal"), self.owner.calls.index("capture_finalized")
        )
        self.assertLess(
            self.owner.calls.index("capture_finalized"), self.owner.calls.index("restart")
        )
        self.assertLess(self.owner.calls.index("restart"), self.owner.calls.index("defaults"))
        self.capture.close.assert_called_once_with(reason="master_reset")
        self.assertEqual(status["defaults"]["target_ids"], [])
        self.assertEqual(saved.read_bytes(), b"saved history preserved")

    def test_rejected_landing_or_unknown_link_never_restarts_services(self) -> None:
        for rejection in (True, False):
            self.owner.reject_landing = rejection
            self.owner.connected = rejection
            self.reset.start()
            self.assertEqual(self.finished()["state"], "error")
            self.assertFalse(self.owner.restarted)
            self.capture.close.assert_not_called()

    def test_missing_service_rejected_before_any_action(self) -> None:
        self.owner.available = False
        with self.assertRaisesRegex(RuntimeError, "Missing UAV owner"):
            self.reset.start()
        self.assertEqual(self.owner.calls, [])

    def test_failed_restart_has_real_error_and_does_not_restore_defaults(self) -> None:
        self.owner.fail_restart = True
        self.reset.start()
        status = self.finished()
        self.assertEqual(status["state"], "error")
        self.assertIn("did not acknowledge", status["error"])
        self.assertNotIn("defaults", self.owner.calls)

    def test_repeated_click_is_rejected_and_does_not_duplicate_landing(self) -> None:
        self.owner.block = threading.Event()
        self.reset.start()
        with self.assertRaisesRegex(RuntimeError, "already in progress"):
            self.reset.start()
        self.owner.block.set()
        self.assertEqual(self.finished()["state"], "completed")
        self.assertEqual(self.owner.calls.count("land:atlas"), 1)

    def test_cancel_waits_for_safe_landing_then_skips_restart(self) -> None:
        self.owner.block = threading.Event()
        self.reset.start()
        self.reset.cancel()
        self.owner.block.set()
        result = self.finished()
        self.assertEqual(result["state"], "cancelled")
        self.assertFalse(self.owner.armed)
        self.assertFalse(self.owner.restarted)
        self.capture.close.assert_not_called()

    def test_async_navigation_landing_requires_actual_acknowledgement(self) -> None:
        self.owner.armed = False
        self.owner.land = lambda identity: {
            "accepted": True,
            "landing_acknowledgement_pending": True,
        }
        self.reset.start()
        result = self.finished()
        self.assertEqual(result["state"], "error")
        self.assertIn("landing acknowledgement failed", result["error"])
        self.assertFalse(self.owner.restarted)
