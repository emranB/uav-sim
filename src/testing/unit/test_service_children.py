"""A detached, uncooperative child cannot survive a service-generation handoff."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux service ownership is required")
class ServiceChildOwnershipTests(unittest.TestCase):
    def test_detached_orphan_is_reaped_without_touching_an_unrelated_peer(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            peer = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])

            def close_peer() -> None:
                if peer.poll() is None:
                    peer.terminate()
                peer.wait(timeout=3)

            self.addCleanup(close_peer)
            source = r"""
import json
import subprocess
import sys
from pathlib import Path
from common.lifecycle.children import enable_service_subreaper, retire_service_children

root = Path(sys.argv[1])
enable_service_subreaper()
child_code = '''
import os
import signal
import time
from pathlib import Path
signal.signal(signal.SIGTERM, signal.SIG_IGN)
Path(%r).write_text(str(os.getpid()))
time.sleep(30)
''' % str(root / "child.pid")
launcher_code = '''
import subprocess
import sys
import time
from pathlib import Path
subprocess.Popen([sys.executable, "-c", %r], start_new_session=True)
deadline = time.monotonic() + 5
while not Path(%r).exists() and time.monotonic() < deadline:
    time.sleep(.01)
''' % (child_code, str(root / "child.pid"))
launcher = subprocess.Popen([sys.executable, "-c", launcher_code])
launcher.wait(timeout=6)
pid = int((root / "child.pid").read_text())
assert Path("/proc/" + str(pid)).exists()
retire_service_children(grace_seconds=.1)
print(json.dumps({"orphan_reaped": not Path("/proc/" + str(pid)).exists()}))
"""
            result = subprocess.run(
                [sys.executable, "-c", source, str(root)],
                capture_output=True,
                text=True,
                timeout=10,
                check=True,
            )
            self.assertTrue(json.loads(result.stdout)["orphan_reaped"])
            self.assertIsNone(peer.poll())
