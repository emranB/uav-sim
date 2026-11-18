"""Concrete restore adapter keeps paused recovery and vehicle scope explicit."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from integrations.restore import WorkspaceRestoreRuntime


class RestoreAdapterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.services = {identity: Mock() for identity in ("atlas", "boreal")}
        for service in self.services.values():
            service.vehicle.snapshot.return_value = {
                "connected": True,
                "armed": False,
                "run_id": "run-1",
                "heartbeat_age_seconds": 0.1,
            }
            service.navigation.snapshot.return_value = {"active": False}
        with patch.dict("os.environ", {"UAV_RESTORE_SERVICES": ""}):
            self.runtime = WorkspaceRestoreRuntime(self.services, Mock(), Path(self.temporary.name))

    def test_paused_world_resumes_and_only_application_link_fault_is_cleared_before_landing(
        self,
    ) -> None:
        self.runtime.prepare(("atlas", "boreal"))
        for service in self.services.values():
            service.world.start.assert_called_once_with()
            service.vehicle.configure_link.assert_called_once_with(0, 0, 0)
            service.vehicle.set_gnss_satellites.assert_not_called()
            self.assertFalse(service.paused)
            service.vehicle.arm.assert_not_called()

    def test_idle_vehicle_never_receives_an_unnecessary_landing_or_peer_command(self) -> None:
        result = self.runtime.land("boreal")
        self.assertTrue(result["already_disarmed"])
        for service in self.services.values():
            service.execute.assert_not_called()
            service.navigation.cancel.assert_not_called()

    def test_active_navigation_uses_existing_cancel_owner_and_pending_landing_ack(self) -> None:
        self.services["boreal"].navigation.snapshot.return_value = {"active": True}
        self.services["boreal"].navigation.cancel.return_value = {"accepted": True}
        result = self.runtime.land("boreal")
        self.assertTrue(result["landing_acknowledgement_pending"])
        self.services["boreal"].navigation.cancel.assert_called_once_with()
        self.services["atlas"].navigation.cancel.assert_not_called()

    def test_unknown_or_armed_state_prevents_every_restart(self) -> None:
        self.services["atlas"].vehicle.snapshot.return_value["armed"] = None
        self.runtime.owners = Mock()
        with self.assertRaisesRegex(RuntimeError, "fresh disarmed"):
            self.runtime.restart(("atlas", "boreal"))
        self.runtime.owners.restart.assert_not_called()
        for service in self.services.values():
            service.features.reset.assert_not_called()

    def test_restart_waits_for_next_fresh_disarmed_heartbeat_without_relaxing_age(self) -> None:
        service = self.services["atlas"]
        old = dict(service.vehicle.snapshot.return_value)
        fresh = dict(old)
        old["heartbeat_age_seconds"] = 2.0
        fresh["heartbeat_age_seconds"] = 0.1
        service.vehicle.snapshot.side_effect = [old, fresh]
        self.runtime._safe()
        self.assertEqual(service.vehicle.snapshot.call_count, 2)
        self.assertEqual(self.services["boreal"].vehicle.snapshot.call_count, 2)
