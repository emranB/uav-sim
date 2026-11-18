"""A refreshed native window file cannot substitute for a new service's real window."""

import json
import tempfile
import time
import unittest
from pathlib import Path

from common.lifecycle.windows import current_service_window


class ServiceWindowTests(unittest.TestCase):
    def test_old_owner_and_reused_pid_are_rejected_despite_fresh_window_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            window = Path(directory) / "QGroundControl.window"
            window.write_text("123")
            now = time.time() - 0.1
            service = {"pid": 222, "process_start_ticks": 900, "machine": "candidate"}
            metadata = {
                "schema_version": 1,
                "window_id": 123,
                "owner_pid": 222,
                "owner_start_ticks": 900,
                "machine": "candidate",
                "native_pid": 333,
                "native_start_ticks": 950,
                "published_wall_time": now + 0.05,
            }
            path = window.with_suffix(".owner.json")
            path.write_text(json.dumps(metadata))
            self.assertTrue(current_service_window(window, service, now))
            for key, value in (
                ("owner_pid", 111),
                ("owner_start_ticks", 899),
                ("machine", "other"),
                ("published_wall_time", now - 1),
            ):
                path.write_text(json.dumps({**metadata, key: value}))
                self.assertFalse(current_service_window(window, service, now), key)
            path.unlink()
            self.assertFalse(current_service_window(window, service, now))
