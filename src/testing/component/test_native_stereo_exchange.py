"""Native exchange returns only the requested snapshot and retains failure bounds."""

import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from integrations.vision.native import NativeStereoEstimator


def _estimator(work: Path) -> NativeStereoEstimator:
    estimator = NativeStereoEstimator.__new__(NativeStereoEstimator)
    estimator.work = work
    estimator.log_path = work / "native.log"
    estimator.started = True
    estimator.process = Mock()
    estimator.process.poll.return_value = None
    return estimator


class NativeStereoExchangeTests(unittest.TestCase):
    def test_previous_snapshot_is_ignored_until_requested_frame_arrives(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            path = work / "snapshot.json"
            path.write_text(json.dumps({"timestamp_seconds": 1.0}))
            expected = {"timestamp_seconds": 2.0, "landmarks": [[1, 2, 3]]}
            estimator = _estimator(work)
            stopping = threading.Event()

            def publish() -> None:
                if not stopping.wait(0.035):
                    temporary = work / "snapshot.tmp"
                    temporary.write_text(json.dumps(expected))
                    temporary.replace(path)

            writer = threading.Thread(target=publish)
            estimator.process.stdin.write.side_effect = lambda request: writer.start()
            try:
                result = estimator.estimate([b"left", b"right"], 2_000_000_000, stopping)
                self.assertEqual(result, expected)
            finally:
                stopping.set()
                writer.join(timeout=1)
            self.assertFalse(writer.is_alive())

    def test_cancellation_returns_without_waiting_for_native_output(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            estimator = _estimator(Path(directory))
            stopping = threading.Event()
            stopping.set()
            self.assertIsNone(estimator.estimate([b"left", b"right"], 1, stopping))

    def test_native_exit_and_processing_timeout_remain_failures(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            estimator = _estimator(Path(directory))
            estimator.process.poll.return_value = 2
            with self.assertRaisesRegex(RuntimeError, "Estimator exited"):
                estimator.estimate([b"left", b"right"], 1, threading.Event())
            estimator.process.poll.return_value = None
            with patch("integrations.vision.native.time.monotonic", side_effect=[100, 131]):
                with self.assertRaisesRegex(TimeoutError, "30 second deadline"):
                    estimator.estimate([b"left", b"right"], 1, threading.Event())

    def test_oversized_snapshot_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            (work / "snapshot.json").write_bytes(b" " * (3 * 1024 * 1024 + 1))
            with self.assertRaisesRegex(ValueError, "point budget"):
                _estimator(work).estimate([b"left", b"right"], 1, threading.Event())
