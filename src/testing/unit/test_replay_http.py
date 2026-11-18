"""Global replay HTTP commands never enter focused fleet flight dispatch."""

import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from unittest.mock import Mock
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from common.simulation_manager.server import Handler


class ReplayHttpTests(unittest.TestCase):
    def test_replay_dispatch_ignores_focus_and_preserves_origin_guard(self) -> None:
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.replay = Mock()
        server.restore = None
        server.fleet = Mock()
        server.replay.inventory.return_value = {"capture": {"state": "idle"}, "items": []}
        server.replay.execute.return_value = {"accepted": True, "state": "capturing"}
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with urlopen(base + "/replay?vehicle_id=boreal", timeout=2) as response:
                self.assertEqual(json.load(response)["capture"]["state"], "idle")
            body = {"target_ids": ["atlas"]}
            with urlopen(
                Request(
                    base + "/replay/start",
                    json.dumps(body).encode(),
                    {
                        "Content-Type": "application/json",
                    },
                ),
                timeout=2,
            ) as response:
                self.assertTrue(json.load(response)["accepted"])
            server.replay.execute.assert_called_once_with("replay/start", body)
            server.fleet.execute.assert_not_called()
            with self.assertRaises(HTTPError) as blocked:
                urlopen(
                    Request(
                        base + "/replay/stop",
                        b"{}",
                        {
                            "Content-Type": "application/json",
                            "Origin": "http://untrusted",
                        },
                    ),
                    timeout=2,
                )
            self.assertEqual(blocked.exception.code, 403)
            self.assertEqual(server.replay.execute.call_count, 1)
        finally:
            server.shutdown()
            server.server_close()
            worker.join(2)
