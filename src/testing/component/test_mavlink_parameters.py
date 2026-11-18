"""Parameter transaction behavior against recorded replies, without a running vehicle."""

import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from integrations.mavlink.connection import SimulatedVehicleConnection


class ParameterTests(unittest.TestCase):
    def test_deployment_timeout_remains_bounded_and_expires(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with (
                patch("integrations.mavlink.connection.SimulatedLink"),
                patch(
                    "integrations.mavlink.connection.mavutil.mavlink_connection",
                    return_value=Mock(),
                ),
            ):
                vehicle = SimulatedVehicleConnection(Path(directory), heartbeat_timeout_seconds=10)
            vehicle.last_heartbeat = time.monotonic() - 7
            self.assertTrue(vehicle.snapshot()["connected"])
            vehicle.last_heartbeat = time.monotonic() - 11
            self.assertFalse(vehicle.snapshot()["connected"])
            vehicle.close()

    def test_vehicle_is_not_ready_until_px4_reports_preflight_health(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with (
                patch("integrations.mavlink.connection.SimulatedLink"),
                patch(
                    "integrations.mavlink.connection.mavutil.mavlink_connection",
                    return_value=Mock(),
                ),
            ):
                vehicle = SimulatedVehicleConnection(Path(directory))
            vehicle.last_heartbeat = time.monotonic()
            vehicle.messages = {
                "GPS_RAW_INT": {"fix_type": 3},
                "LOCAL_POSITION_NED": {},
                "GLOBAL_POSITION_INT": {"lat": 446488000},
                "SYS_STATUS": {"onboard_control_sensors_health": 0},
            }
            vehicle.received_at = {kind: time.monotonic() for kind in vehicle.messages}
            self.assertTrue(vehicle.snapshot()["connected"])
            self.assertFalse(vehicle.snapshot()["ready"])
            vehicle.messages["SYS_STATUS"]["onboard_control_sensors_health"] = 268435456
            self.assertTrue(vehicle.snapshot()["ready"])
            vehicle.close()

    def test_restart_pending_cannot_be_satisfied_by_old_telemetry(self) -> None:
        runtime = Mock()
        runtime.generation.return_value = "first-run"
        with tempfile.TemporaryDirectory() as directory:
            with (
                patch("integrations.mavlink.connection.SimulatedLink"),
                patch(
                    "integrations.mavlink.connection.mavutil.mavlink_connection",
                    return_value=Mock(),
                ),
            ):
                vehicle = SimulatedVehicleConnection(Path(directory), runtime)
            vehicle.last_heartbeat = time.monotonic()
            self.assertTrue(vehicle.snapshot()["connected"])
            vehicle.restart()
            vehicle.last_heartbeat = time.monotonic()
            self.assertTrue(vehicle.snapshot()["restart_pending"])
            self.assertFalse(vehicle.snapshot()["connected"])
            self.assertFalse(vehicle.snapshot()["ready"])
            runtime.restart.assert_called_once()

    def test_stale_parameter_values_do_not_reject_a_later_confirmed_change(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with (
                patch("integrations.mavlink.connection.SimulatedLink"),
                patch(
                    "integrations.mavlink.connection.mavutil.mavlink_connection",
                    return_value=Mock(),
                ),
            ):
                vehicle = SimulatedVehicleConnection(Path(directory))
            with (
                patch.object(vehicle, "wait_connected"),
                patch.object(
                    vehicle, "_parameter_reply", side_effect=[3.0, 5.0, 1.5, 3.0, 5.0, 3.0, 10]
                ),
            ):
                configuration = vehicle.configure_limits(5.0, 3.0)
            self.assertEqual(configuration["cruise_speed_m_s"], 5.0)
            self.assertEqual(configuration["climb_speed_m_s"], 3.0)
            self.assertEqual(vehicle.connection.mav.param_set_send.call_count, 2)
