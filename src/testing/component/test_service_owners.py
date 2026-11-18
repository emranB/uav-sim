"""Real child restart is acknowledged by generation; no Docker or flight services."""

import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock

from integrations.runtime_services import RuntimeServiceOwners


class ServiceOwnerTests(unittest.TestCase):
    def test_actual_owned_child_restart_and_shutdown(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            environment = dict(os.environ)
            environment["PYTHONPATH"] = str(Path(__file__).resolve().parents[2])
            process = subprocess.Popen(
                [
                    sys.executable,
                    "-m",
                    "common.lifecycle.supervisor",
                    "--role",
                    "camera-streams",
                    "--runtime-root",
                    str(root),
                    "--request-root",
                    str(root / "requests"),
                    "--",
                    sys.executable,
                    "-c",
                    "import time; time.sleep(60)",
                ],
                env=environment,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            owners = RuntimeServiceOwners(("camera-streams",), root, root / "requests")
            try:
                deadline = time.monotonic() + 4
                while True:
                    try:
                        before = owners.state("camera-streams")["generation"]
                        break
                    except (OSError, RuntimeError):
                        if time.monotonic() >= deadline:
                            self.fail("Actual owner did not publish live child status")
                        time.sleep(0.05)
                safety = Mock()
                after = owners.restart("camera-streams", safety)
                self.assertNotEqual(before, after)
                safety.assert_called_once_with()
                self.assertTrue(owners.state("camera-streams")["running"])
            finally:
                process.terminate()
                process.wait(timeout=5)
                if process.stderr:
                    process.stderr.close()
            with self.assertRaises(RuntimeError):
                owners.state("camera-streams")

    def test_unknown_host_service_is_outside_restore_scope(self) -> None:
        with self.assertRaises(ValueError):
            RuntimeServiceOwners(("docker",))
