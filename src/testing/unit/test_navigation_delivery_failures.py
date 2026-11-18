"""Adverse delivery and shutdown checks confined to temporary files and owned children."""

import errno
import json
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from common.navigation.session import NavigationSession
from integrations.vision.navigation_source import write_atomic_json
from testing.unit.test_navigation_session import EstimatedVehicle, MeasuredRanges


class NavigationDeliveryFailureTests(unittest.TestCase):
    def test_recording_disk_full_reports_failure_after_safe_cancellation(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            historical = directory / "historical.json"
            historical.write_text('{"preserved": true}')
            vehicle, lease = EstimatedVehicle(), threading.Lock()
            session = NavigationSession("vehicle_1", vehicle, MeasuredRanges(), lease, directory)
            original_write = Path.write_bytes

            def fail_run_record(path: Path, *args: object, **kwargs: object) -> int:
                if path.parent == directory and path != historical:
                    raise OSError(errno.ENOSPC, "No space left on fixture device")
                return original_write(path, *args, **kwargs)

            with patch.object(Path, "write_bytes", fail_run_record):
                session.start({"goal_offset_enu_m": [0, 10]})
                session.cancel()
                session.thread.join(timeout=5)
            self.assertFalse(session.thread.is_alive())
            self.assertFalse(vehicle.armed)
            self.assertEqual(vehicle.landings, 1)
            self.assertTrue(lease.acquire(blocking=False))
            lease.release()
            status = session.snapshot()
            self.assertEqual(status["outcome"], "failed")
            self.assertFalse(status["report_persisted"])
            self.assertIn("vehicle_1", status["error"])
            self.assertIn("No space left", status["error"])
            self.assertEqual(historical.read_text(), '{"preserved": true}')

    def test_close_cancels_before_arming_and_stops_velocity_sender(self) -> None:
        class ClosingRanges(MeasuredRanges):
            closed = False

            def close(self) -> None:
                self.closed = True

        with tempfile.TemporaryDirectory() as name:
            vehicle, ranges, lease = EstimatedVehicle(), ClosingRanges(), threading.Lock()
            session = NavigationSession("vehicle_2", vehicle, ranges, lease, Path(name))
            session.start({"goal_offset_enu_m": [0, 10]})
            session.close()
            self.assertFalse(session.thread.is_alive())
            self.assertTrue(ranges.closed)
            self.assertFalse(vehicle.armed)
            self.assertEqual(session.snapshot()["outcome"], "cancelled")
            velocity_count = len(vehicle.velocities)
            time.sleep(0.2)
            self.assertEqual(len(vehicle.velocities), velocity_count)
            self.assertTrue(lease.acquire(blocking=False))
            lease.release()
            self.assertEqual(
                json.loads(next(Path(name).glob("*.json")).read_text())["outcome"], "cancelled"
            )

    def test_atomic_publish_failures_preserve_last_complete_record_and_remove_temporary(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "latest.json"
            write_atomic_json(path, {"sequence": 1})
            for operation in ["write_bytes", "replace"]:
                with self.subTest(operation=operation):
                    with patch.object(Path, operation, side_effect=OSError(errno.ENOSPC, "full")):
                        with self.assertRaises(OSError):
                            write_atomic_json(path, {"sequence": 2})
                    self.assertEqual(json.loads(path.read_text()), {"sequence": 1})
                    self.assertEqual(list(Path(name).glob("*.tmp")), [])
            write_atomic_json(path, {"sequence": 3})
            self.assertEqual(json.loads(path.read_text()), {"sequence": 3})

    def test_killed_publisher_cannot_replace_last_record_with_partial_json(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            path = directory / "latest.json"
            write_atomic_json(path, {"sequence": 1})
            script = """
import sys, time
from pathlib import Path
from unittest.mock import patch
from integrations.vision.navigation_source import write_atomic_json
root = Path(sys.argv[1])
def hold_before_replace(self, destination):
    (root / 'replacement-ready').touch()
    time.sleep(60)
with patch.object(Path, 'replace', hold_before_replace):
    write_atomic_json(root / 'latest.json', {'sequence': 2})
"""
            child = subprocess.Popen([sys.executable, "-c", script, name])
            try:
                deadline = time.monotonic() + 5
                while not (directory / "replacement-ready").exists():
                    if child.poll() is not None or time.monotonic() > deadline:
                        self.fail("Owned publisher did not reach the replacement boundary")
                    time.sleep(0.02)
                child.kill()
                child.wait(timeout=3)
                self.assertEqual(json.loads(path.read_text()), {"sequence": 1})
                # SIGKILL cannot run finally. Orphan files are ignored by the reader;
                # the temporary fixture owns cleanup, never the historical-record tree.
                self.assertEqual(len(list(directory.glob("*.tmp"))), 1)
                write_atomic_json(path, {"sequence": 3})
                self.assertEqual(json.loads(path.read_text()), {"sequence": 3})
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait(timeout=3)
