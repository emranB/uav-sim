"""Concurrent atomic status publication must not break offline storage admission."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from common.logging.storage import directory_bytes


class ReplayOutputAccountingTests(unittest.TestCase):
    def test_atomic_rename_during_measurement_is_counted_on_next_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stable = root / "video.mp4"
            stable.write_bytes(b"12345678")
            transient = root / ".status.tmp"
            transient.write_bytes(b"0123456789")
            destination = root / "status.json"
            original = Path.is_file

            def raced_is_file(path):
                result = original(path)
                if path == transient and result:
                    transient.rename(destination)
                return result

            with patch.object(Path, "is_file", raced_is_file):
                self.assertGreaterEqual(directory_bytes(root), 8)
            self.assertEqual(directory_bytes(root), 18)

    def test_permission_errors_remain_visible(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            file = root / "unreadable"
            file.write_bytes(b"evidence")
            original = Path.stat

            def denied(path, *args, **kwargs):
                if path == file:
                    raise PermissionError("Output cannot be measured")
                return original(path, *args, **kwargs)

            with patch.object(Path, "stat", denied):
                with self.assertRaises(PermissionError):
                    directory_bytes(root)
