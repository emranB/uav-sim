"""Lifecycle policy tests with no PX4, Gazebo, ROS, Qt, or sockets."""

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from common.configuration.runtime import RuntimeSettings
from common.simulation_manager.server import SimulationService


class RecordedVehicle:
    def __init__(self) -> None:
        self.armed = False
        self.operations = []
        self.items = []
        self.generation = 0

    def start(self) -> None:
        pass

    def snapshot(self) -> dict:
        return {
            "armed": self.armed,
            "connected": True,
            "ready": True,
            "run_id": f"controller-{self.generation}",
        }

    def wait_connected(self, timeout_seconds: float) -> None:
        pass

    def download(self) -> dict:
        return {"mission": {"items": self.items}}

    def arm(self) -> None:
        self.armed = True
        self.operations.append("arm")

    def start_mission(self) -> None:
        self.operations.append("mission")

    def land(self) -> None:
        self.operations.append("land")

    def restart(self) -> None:
        self.operations.append("restart")
        self.generation += 1


class RecordedWorld:
    def __init__(self) -> None:
        self.operations = []

    def start(self) -> None:
        self.operations.append("start")

    def pause(self) -> None:
        self.operations.append("pause")

    def reset(self) -> None:
        self.operations.append("reset")


class RecordedScene:
    def __init__(self) -> None:
        self.epoch = 0

    def begin_run(self) -> None:
        self.epoch += 1


class SimulationServiceTests(unittest.TestCase):
    def test_declared_gnss_fault_can_run_while_navigation_retains_authority(self) -> None:
        self.service.navigation = Mock()
        self.service.navigation.snapshot.return_value = {"active": True}
        self.service.features = Mock()
        self.service.features.execute.return_value = {"accepted": True, "gnss_enabled": False}
        with self.service.operation_lock:
            result = self.service.execute("sensors/gnss", {"enabled": False})
            with self.assertRaisesRegex(RuntimeError, "operation is in progress"):
                self.service.execute("vehicles/configure", {})
        self.assertFalse(result["gnss_enabled"])
        self.service.features.execute.assert_called_once_with("sensors/gnss", {"enabled": False})
        self.assertEqual(self.vehicle.operations, [])

    def test_native_start_retains_explicit_stereo_reference_option(self) -> None:
        mission = SimpleNamespace(name="saved", sha256="digest", goal_offset_enu_m=(0, 9))
        self.service.visual_mission_name = "saved"
        self.service.features = Mock()
        self.service.features.missions.load_with_digest.return_value = ({}, "digest")
        self.service.navigation = Mock()
        with (
            patch.object(self.service, "_visual_mission", return_value=mission),
            patch("common.simulation_manager.server.VisualMission.from_plan", return_value=mission),
        ):
            self.service.execute("flight/start", {"stereo_horizontal_reference": True})
        body = self.service.navigation.start.call_args.args[0]
        self.assertTrue(body["stereo_horizontal_reference"])
        self.assertIs(self.service.navigation.start.call_args.kwargs["mission"], mission)

    def test_recording_analysis_does_not_block_world_pause(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        def compare(operation: str, body: dict) -> dict:
            entered.set()
            release.wait(2)
            return {"accepted": True}

        self.service.features = Mock()
        self.service.features.execute.side_effect = compare
        worker = threading.Thread(target=self.service.execute, args=("recordings/compare", {}))
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.service.execute("pause", {})
            self.assertEqual(self.world.operations, ["pause"])
        finally:
            release.set()
            worker.join(timeout=2)

    def test_unavailable_capability_never_changes_vehicle_or_world(self) -> None:
        from common.interfaces.capabilities import CapabilityUnavailable

        with self.assertRaises(CapabilityUnavailable) as caught:
            self.service.execute("perception/check", {"capability_id": "visual_odometry"})
        self.assertEqual(caught.exception.result()["code"], "CAPABILITY_UNAVAILABLE")
        self.assertEqual(self.vehicle.operations, [])
        self.assertEqual(self.world.operations, [])

    def test_camera_does_not_compete_with_physics_commands(self) -> None:
        self.service.viewer = Mock()
        self.service.scene_source = Mock()
        self.service.scene_source.snapshot.return_value = {
            "poses": [{"vehicle_id": "vehicle_1", "position_enu_m": [1, 2, 3]}]
        }
        with self.service.operation_lock:
            result = self.service.execute("viewer/camera", {"preset": "top", "follow": True})
        self.assertTrue(result["accepted"])
        self.service.viewer.set_camera.assert_called_once_with("top", True, (1, 2, 3))

    def setUp(self) -> None:
        self.vehicle = RecordedVehicle()
        self.world = RecordedWorld()
        self.scene = RecordedScene()
        self.service = SimulationService(RuntimeSettings(), self.vehicle, self.world, self.scene)

    def test_start_without_mission_never_arms(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "Upload a mission"):
            self.service.execute("start", {})
        self.assertFalse(self.vehicle.armed)

    def test_reset_while_armed_is_rejected(self) -> None:
        self.vehicle.armed = True
        with self.assertRaisesRegex(RuntimeError, "Land"):
            self.service.execute("reset", {})
        self.assertEqual(self.world.operations, [])
        self.assertEqual(self.vehicle.operations, [])

    def test_reset_uses_adapters_and_advances_scene_epoch(self) -> None:
        self.service.execute("reset", {})
        self.assertEqual(self.world.operations, ["start", "reset"])
        self.assertEqual(self.vehicle.operations, ["restart"])
        self.assertEqual(self.scene.epoch, 1)

    def test_stop_on_ground_is_idempotent(self) -> None:
        self.service.execute("stop", {})
        self.service.execute("stop", {})
        self.assertEqual(self.vehicle.operations, [])

    def test_reset_rechecks_armed_state_after_reconnection(self) -> None:
        def reconnect(timeout_seconds: float) -> None:
            self.vehicle.armed = True

        with patch.object(self.vehicle, "wait_connected", side_effect=reconnect):
            with self.assertRaisesRegex(RuntimeError, "Land"):
                self.service.execute("reset", {})
        self.assertEqual(self.world.operations, ["start"])
        self.assertEqual(self.vehicle.operations, [])

    def test_reset_without_connection_never_resets_the_world(self) -> None:
        with patch.object(self.vehicle, "wait_connected", side_effect=TimeoutError("disconnected")):
            with self.assertRaises(TimeoutError):
                self.service.execute("reset", {})
        self.assertEqual(self.world.operations, ["start"])
        self.assertEqual(self.vehicle.operations, [])

    def test_stop_unpauses_and_waits_for_connection_before_landing(self) -> None:
        self.vehicle.armed = True

        def reconnect(timeout_seconds: float) -> None:
            self.assertEqual(self.world.operations, ["start"])
            self.assertEqual(self.vehicle.operations, [])

        with patch.object(self.vehicle, "wait_connected", side_effect=reconnect) as wait:
            self.service.execute("stop", {})
        wait.assert_called_once_with(10)
        self.assertEqual(self.vehicle.operations, ["land"])

    def test_stop_reports_reconnection_timeout_instead_of_success(self) -> None:
        self.vehicle.armed = True
        with patch.object(self.vehicle, "wait_connected", side_effect=TimeoutError("disconnected")):
            with self.assertRaisesRegex(TimeoutError, "disconnected"):
                self.service.execute("stop", {})
        self.assertEqual(self.vehicle.operations, [])

    def test_conflicting_operations_are_rejected(self) -> None:
        self.service.operation_lock.acquire()
        try:
            with self.assertRaisesRegex(RuntimeError, "in progress"):
                self.service.execute("pause", {})
        finally:
            self.service.operation_lock.release()
        self.assertEqual(self.world.operations, [])

    def test_navigation_reset_observes_disarm_before_controller_restart(self) -> None:
        self.service.navigation = Mock()

        def reset_navigation() -> None:
            self.assertFalse(self.vehicle.armed)
            self.assertNotIn("restart", self.vehicle.operations)
            self.assertNotIn("reset", self.world.operations)

        self.service.navigation.reset.side_effect = reset_navigation
        self.service.execute("reset", {})
        self.service.navigation.reset.assert_called_once_with()
        self.assertEqual(self.vehicle.operations, ["restart"])

    def test_reset_does_not_claim_completion_or_reset_views_when_readiness_fails(self) -> None:
        self.service.reset_views = Mock()
        with patch(
            "common.simulation_manager.server.wait_for_reset_readiness",
            side_effect=TimeoutError("new controller not ready"),
        ):
            with self.assertRaisesRegex(TimeoutError, "not ready"):
                self.service.execute("reset", {})
        self.service.reset_views.assert_not_called()
        self.assertFalse(self.service.operation_lock.locked())

    def test_stop_cancels_navigation_during_observation_wait(self) -> None:
        self.service.navigation = Mock()
        self.service.navigation.snapshot.return_value = {
            "state": "waiting_for_fresh_observation",
            "active": True,
        }
        with self.service.operation_lock:
            self.service.execute("stop", {})
        self.service.navigation.cancel.assert_called_once_with()
        self.assertEqual(self.world.operations, [])
