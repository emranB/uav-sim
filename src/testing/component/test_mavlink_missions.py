"""Mission downloads retry lost replies and ignore duplicate sequence responses."""

import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from integrations.mavlink.connection import SimulatedVehicleConnection


class MissionDownloadTests(unittest.TestCase):
    def create_vehicle(self, directory: str) -> SimulatedVehicleConnection:
        with (
            patch("integrations.mavlink.connection.SimulatedLink"),
            patch(
                "integrations.mavlink.connection.mavutil.mavlink_connection", return_value=Mock()
            ),
        ):
            return SimulatedVehicleConnection(Path(directory), vehicle_id="vehicle_2")

    def test_lost_item_is_requested_again_without_silent_success(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            vehicle = self.create_vehicle(directory)
            send = Mock()
            item = Mock(seq=2)
            with patch.object(vehicle, "wait_reply", side_effect=[TimeoutError(), item]):
                result = vehicle._request_mission_reply(
                    send,
                    {"MISSION_ITEM_INT"},
                    time.monotonic() + 10,
                    sequence=2,
                )
            self.assertIs(result, item)
            self.assertEqual(send.call_count, 2)
            with patch.object(vehicle, "wait_reply", side_effect=TimeoutError()):
                with self.assertRaisesRegex(TimeoutError, "vehicle_2.*sequence 2"):
                    vehicle._request_mission_reply(
                        send,
                        {"MISSION_ITEM_INT"},
                        time.monotonic() + 10,
                        sequence=2,
                    )
            vehicle.close()

    def test_duplicate_previous_item_cannot_satisfy_current_sequence(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            vehicle = self.create_vehicle(directory)
            previous, current = Mock(seq=0), Mock(seq=1)
            for item in (previous, current):
                item.get_type.return_value = "MISSION_ITEM_INT"
                vehicle.replies.append(item)
            self.assertIs(vehicle.wait_reply({"MISSION_ITEM_INT"}, 1, sequence=1), current)
            self.assertIn(previous, vehicle.replies)
            vehicle.close()
