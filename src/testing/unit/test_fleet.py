"""Vehicle isolation and collection failure semantics without simulation dependencies."""

import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock

from common.configuration.fleet import load_fleet
from common.simulation_manager.fleet import FleetOperations


class FleetTests(unittest.TestCase):
    def test_stereo_start_option_reaches_only_the_explicit_vehicle(self) -> None:
        self.services["vehicle_1"].flight_actions.return_value = {
            "start": {"enabled": True, "reason": ""},
        }
        self.fleet.execute(
            "flight/start", {"target_ids": ["vehicle_1"], "stereo_horizontal_reference": True}
        )
        self.services["vehicle_1"].execute.assert_called_once_with(
            "flight/start", {"stereo_horizontal_reference": True}
        )
        self.services["vehicle_2"].execute.assert_not_called()

    def test_peer_recovery_remains_available_during_reset(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        def reset(operation: str, body: dict) -> dict:
            entered.set()
            release.wait(2)
            return {"accepted": True}

        self.services["vehicle_1"].execute.side_effect = reset
        self.services["vehicle_2"].flight_actions.return_value = {
            "land": {"enabled": True, "reason": None},
        }
        worker = threading.Thread(
            target=self.fleet.execute,
            args=("reset", {"vehicle_id": "vehicle_1"}),
        )
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            for operation in ("stop", "flight/land"):
                result = self.fleet.execute(operation, {"target_ids": ["vehicle_2"]})
                self.assertTrue(result["accepted"])
            self.assertEqual(self.services["vehicle_2"].execute.call_count, 2)
        finally:
            release.set()
            worker.join(timeout=2)

    def test_read_only_comparison_does_not_block_fleet_pause(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        def execute(operation: str, body: dict) -> dict:
            if operation == "recordings/compare":
                entered.set()
                release.wait(2)
            return {"accepted": True}

        self.services["vehicle_1"].execute.side_effect = execute
        worker = threading.Thread(
            target=self.fleet.execute,
            args=("recordings/compare", {"target_ids": ["vehicle_1"]}),
        )
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            result = self.fleet.execute("pause", {"target_ids": ["vehicle_2"]})
            self.assertTrue(result["accepted"])
        finally:
            release.set()
            worker.join(timeout=2)

    def setUp(self) -> None:
        self.services = {name: Mock() for name in ("vehicle_1", "vehicle_2")}
        for service in self.services.values():
            service.execute.return_value = {"accepted": True}
        self.fleet = FleetOperations(self.services)

    def test_single_target_never_calls_its_peer(self) -> None:
        self.fleet.execute("interference/link", {"target_ids": ["vehicle_2"], "loss_percent": 100})
        self.services["vehicle_1"].execute.assert_not_called()
        self.services["vehicle_2"].execute.assert_called_once_with(
            "interference/link", {"loss_percent": 100}
        )

    def test_unknown_target_rejected_before_any_mutation(self) -> None:
        with self.assertRaises(ValueError):
            self.fleet.execute("reset", {"target_ids": ["vehicle_1", "missing"]})
        for service in self.services.values():
            service.execute.assert_not_called()

    def test_partial_collection_failure_is_not_reported_as_success(self) -> None:
        self.services["vehicle_1"].execute.side_effect = TimeoutError("No acknowledgement")
        result = self.fleet.execute("reset", {"target_ids": list(self.services)})
        self.assertFalse(result["accepted"])
        self.assertFalse(result["results"]["vehicle_1"]["accepted"])
        self.assertTrue(result["results"]["vehicle_2"]["accepted"])

    def test_duplicate_port_allocation_is_rejected(self) -> None:
        payload = json.loads(Path("configs/vehicles/fleet.json").read_text(encoding="utf-8"))
        payload["vehicles"][1]["application_port"] = payload["vehicles"][0]["vehicle_port"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fleet.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unique"):
                load_fleet(path)
