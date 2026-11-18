"""Storage admission and failures must preserve records and transport liveness."""

import errno
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from common.logging.storage import (
    BoundedRecording,
    StorageCapacityError,
    ensure_capacity,
    write_atomic,
)


class StorageRecordingTests(unittest.TestCase):
    def test_capacity_checks_bytes_and_inodes_before_admission(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with patch(
                "common.logging.storage.os.statvfs",
                return_value=SimpleNamespace(
                    f_bavail=10,
                    f_frsize=1024,
                    f_favail=2000,
                ),
            ):
                ensure_capacity(Path(directory), required_bytes=4096, reserve_bytes=6144)
                with self.assertRaises(StorageCapacityError) as caught:
                    ensure_capacity(Path(directory), required_bytes=4097, reserve_bytes=6144)
                self.assertEqual(caught.exception.errno, errno.ENOSPC)
                self.assertIn("required", str(caught.exception))
            with (
                patch(
                    "common.logging.storage.os.statvfs",
                    return_value=SimpleNamespace(
                        f_bavail=100,
                        f_frsize=1024,
                        f_favail=0,
                    ),
                ),
                self.assertRaisesRegex(StorageCapacityError, "inode"),
            ):
                ensure_capacity(Path(directory), reserve_bytes=0)

    def test_failed_atomic_publication_preserves_previous_ledger(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ledger.json"
            path.write_text('{"complete":true}')
            with patch("pathlib.Path.write_bytes", side_effect=OSError(errno.ENOSPC, "full")):
                with self.assertRaisesRegex(OSError, "ledger.json"):
                    write_atomic(path, b'{"complete":false}')
            self.assertEqual(json.loads(path.read_text()), {"complete": True})
            self.assertEqual(list(Path(directory).glob("*.tmp")), [])

    def test_recording_is_bounded_without_deleting_earlier_lines(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "console.jsonl"
            with BoundedRecording(path, maximum_bytes=8) as recording:
                self.assertTrue(recording.write("first\n"))
                self.assertFalse(recording.write("second\n"))
                self.assertFalse(recording.write("x\n"))
                self.assertIn("limit", recording.error)
            self.assertEqual(path.read_text(), "first\n")

    def test_write_failure_is_visible_without_breaking_console_drain(self) -> None:
        from common.lifecycle.process import record_console

        with (
            tempfile.TemporaryDirectory() as directory,
            patch.dict("os.environ", {"OUTPUT_DIR": directory}),
            patch("common.logging.storage.Path.open", side_effect=OSError(errno.ENOSPC, "full")),
            patch("sys.stdout", new_callable=io.StringIO) as output,
            patch("sys.stderr", new_callable=io.StringIO) as errors,
        ):
            record_console(io.StringIO("first\nsecond\nthird\n"), "px4")
        self.assertIn("third", output.getvalue())
        self.assertIn("recording", errors.getvalue().lower())
        self.assertIn("full", errors.getvalue())

    def test_telemetry_write_failure_preserves_acknowledgements_and_disarm_observation(
        self,
    ) -> None:
        from unittest.mock import Mock

        from pymavlink import mavutil

        from integrations.mavlink.connection import SimulatedVehicleConnection

        armed = mavutil.mavlink.MAVLink_heartbeat_message(2, 12, 128, 0, 3, 3)
        ack = mavutil.mavlink.MAVLink_command_ack_message(21, 0)
        disarmed = mavutil.mavlink.MAVLink_heartbeat_message(2, 12, 0, 0, 3, 3)
        messages = [armed, ack, disarmed]
        for message in messages:
            message._header.srcSystem = 1
        transport = Mock()
        with (
            tempfile.TemporaryDirectory() as directory,
            patch("integrations.mavlink.connection.SimulatedLink"),
            patch.object(SimulatedVehicleConnection, "_new_connection", return_value=transport),
        ):
            vehicle = SimulatedVehicleConnection(Path(directory), vehicle_id="vehicle_1")

            def receive(**kwargs):
                message = messages.pop(0)
                if not messages:
                    vehicle.stop_event.set()
                return message

            transport.recv_match.side_effect = receive
            with patch(
                "common.logging.storage.Path.open", side_effect=OSError(errno.ENOSPC, "full")
            ):
                vehicle.receive()
            self.assertIn("full", vehicle.snapshot()["recording"]["error"])
            self.assertTrue(vehicle.snapshot()["connected"])
            self.assertFalse(vehicle.navigation_armed())
            self.assertEqual(list(vehicle.replies), [ack])

    def test_execution_admission_failure_does_not_arm_or_take_authority(self) -> None:
        import threading

        from common.navigation.session import NavigationSession
        from testing.unit.test_navigation_session import EstimatedVehicle, MeasuredRanges

        with tempfile.TemporaryDirectory() as directory:
            vehicle = EstimatedVehicle()
            lease = threading.Lock()
            session = NavigationSession(
                "vehicle_1", vehicle, MeasuredRanges(), lease, Path(directory)
            )
            with (
                patch(
                    "common.navigation.session.ensure_run_capacity",
                    side_effect=StorageCapacityError(errno.ENOSPC, "insufficient capacity"),
                ),
                self.assertRaises(StorageCapacityError),
            ):
                session.start({"goal_offset_enu_m": [0, 10]})
            self.assertFalse(vehicle.armed)
            self.assertIsNone(session.thread)
            self.assertTrue(lease.acquire(blocking=False))
            lease.release()

    def test_capacity_loss_during_execution_lands_and_uses_report_reserve(self) -> None:
        import threading

        from common.navigation.session import NavigationSession
        from testing.unit.test_navigation_session import EstimatedVehicle, MeasuredRanges

        with tempfile.TemporaryDirectory() as directory:
            vehicle = EstimatedVehicle()
            session = NavigationSession(
                "vehicle_1", vehicle, MeasuredRanges(), threading.Lock(), Path(directory)
            )
            with patch(
                "common.navigation.session.ensure_capacity",
                side_effect=StorageCapacityError(errno.ENOSPC, "recording reserve reached"),
            ):
                session.start({"goal_offset_enu_m": [0, 10]})
                session.thread.join(timeout=4)
            self.assertFalse(session.thread.is_alive())
            self.assertFalse(vehicle.armed)
            self.assertEqual(vehicle.landings, 1)
            self.assertEqual(session.snapshot()["outcome"], "failed")
            self.assertIn("reserve", session.snapshot()["error"])
            self.assertTrue(session.snapshot()["report_persisted"])
            self.assertTrue(list(Path(directory).glob("*.json")))

    def test_camera_generation_requires_capacity_before_subscribing_or_archiving(self) -> None:
        from unittest.mock import Mock

        from common.configuration.fleet import VehicleDefinition
        from gazebo.camera_streams import VehicleCameras

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            definition = VehicleDefinition("vehicle_1", "Atlas", 0, 1, 14540, 14542, (0, 0, 0))
            node = Mock()
            with (
                patch(
                    "gazebo.camera_streams.ensure_capacity",
                    side_effect=StorageCapacityError(errno.ENOSPC, "insufficient archive budget"),
                ),
                self.assertRaises(StorageCapacityError),
            ):
                VehicleCameras(node, definition, root)
            node.subscribe.assert_not_called()
            self.assertFalse((root / "perception").exists())

    def test_health_distinguishes_recording_failure_from_heartbeat_loss(self) -> None:
        from unittest.mock import Mock

        from common.configuration.runtime import RuntimeSettings
        from common.simulation_manager.server import Handler

        vehicle = Mock()
        vehicle.snapshot.return_value = {
            "ready": False,
            "connected": True,
            "recording": {"available": False, "error": "Recording byte limit reached"},
        }
        service = SimpleNamespace(vehicle=vehicle, settings=RuntimeSettings())
        handler = Handler.__new__(Handler)
        handler.path = "/health"
        handler.server = SimpleNamespace(services={"vehicle_1": service})
        handler.respond = Mock()
        with patch("common.simulation_manager.server.ensure_run_capacity", return_value={}):
            handler.do_GET()
        code, payload = handler.respond.call_args.args
        self.assertEqual(code, 503)
        self.assertEqual(payload["status"], "recording-unavailable")
        self.assertEqual(
            payload["recording_failures"], {"vehicle_1": "Recording byte limit reached"}
        )
