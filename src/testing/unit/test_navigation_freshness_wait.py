"""Session waits send no route movement and only resume after fresh replanning."""

import math
import tempfile
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from common.interfaces.navigation import NavigationPose
from common.interfaces.obstacles import StaleObservationError
from common.navigation.freshness import ObservationRecovery, ObservationWaitPolicy
from common.navigation.occupancy import GridConfiguration, ObservedGrid
from common.navigation.session import NavigationSession
from testing.unit.test_navigation_session import EstimatedVehicle, MeasuredRanges


def await_condition(condition, timeout_seconds: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_seconds
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError("Controlled session did not reach its expected state")
        time.sleep(0.005)


class WaitingVehicle(EstimatedVehicle):
    def __init__(self) -> None:
        super().__init__()
        self.pose_failed = False
        self.controller_run_id = "controller"
        self.command_times = []

    def begin_navigation(self) -> None:
        super().begin_navigation()
        self.position = (0.0, 0.0, 4.0)

    def navigation_pose(self) -> NavigationPose:
        if self.pose_failed:
            raise RuntimeError("Estimated local pose unavailable")
        return replace(
            super().navigation_pose(),
            run_id=self.controller_run_id,
            simulation_timestamp_seconds=time.monotonic(),
        )

    def send_navigation_velocity(self, velocity_enu_m_s: tuple) -> None:
        self.velocities.append(velocity_enu_m_s)
        self.command_times.append(time.monotonic())
        self.position = tuple(self.position[i] + velocity_enu_m_s[i] * 0.1 for i in range(3))


class ControlledSensing:
    def __init__(self) -> None:
        self.blocked = False
        self.fatal = None
        self.metrics = {}
        self.plan_times = []
        self.grid = ObservedGrid((0, 0), GridConfiguration())
        self.grid.free = {(x, y): 0.0 for x in range(-10, 11) for y in range(-10, 31)}
        original_plan = self.grid.plan

        def plan(*args):
            self.plan_times.append(time.monotonic())
            return original_plan(*args)

        self.grid.plan = plan
        self.health()

    def health(self):
        if self.fatal:
            raise RuntimeError(self.fatal)
        if self.blocked:
            raise StaleObservationError(
                "vehicle_1",
                "Stereo sensor timestamp stale",
                timestamp_seconds=self.observation_timestamp_seconds,
                sensor_age_sim_seconds=0.904,
                delivery_age_wall_seconds=0.975,
            )
        now = time.monotonic()
        self.observation_timestamp_seconds = math.floor(now * 5) / 5 - 0.05
        self.observation_received_monotonic = now - 0.1
        self.observation_deadline = now + 1.8

    def update(self):
        self.health()
        self.grid.timestamp_seconds = self.observation_timestamp_seconds
        return self.grid


class NavigationFreshnessWaitTests(unittest.TestCase):
    def setUp(self) -> None:
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.vehicle = WaitingVehicle()
        self.sensing = ControlledSensing()
        self.lease = threading.Lock()
        self.session = NavigationSession(
            "vehicle_1",
            self.vehicle,
            MeasuredRanges(),
            self.lease,
            Path(self.folder.name),
            stereo=object(),
            freshness_policy=ObservationWaitPolicy(
                maximum_wait_seconds=0.35,
                maximum_total_wait_seconds=0.7,
                maximum_wait_episodes=2,
            ),
        )
        self.factory = patch(
            "common.navigation.session.NavigationSensing", return_value=self.sensing
        )
        self.factory.start()
        self.addCleanup(self.factory.stop)
        self.addCleanup(self.stop)
        self.session.start({"obstacle_mode": "camera", "goal_offset_enu_m": [0, 10]})
        await_condition(lambda: any(math.hypot(*v[:2]) > 0 for v in self.vehicle.velocities))

    def stop(self) -> None:
        self.session.cancel()
        self.session.thread.join(timeout=5)

    def wait_for_zero(self) -> int:
        self.sensing.blocked = True
        await_condition(lambda: self.session.snapshot().get("freshness_waiting"))
        await_condition(
            lambda: any(
                e["kind"] == "zero_velocity_command"
                for e in self.session.snapshot()["freshness_wait_events"]
            )
        )
        return len(self.vehicle.velocities) - 1

    def test_brief_delay_zero_commands_then_new_route_before_resuming(self) -> None:
        index = self.wait_for_zero()
        deadline = self.session.snapshot()["navigation_deadline_monotonic"]
        self.assertEqual(self.vehicle.velocities[index], (0.0, 0.0, 0.0))
        release = time.monotonic()
        self.sensing.blocked = False
        await_condition(lambda: self.session.snapshot().get("freshness_recovery_count") == 1)
        await_condition(
            lambda: any(math.hypot(*v[:2]) > 0 for v in self.vehicle.velocities[index + 1 :])
        )
        self.assertTrue(any(t >= release for t in self.sensing.plan_times))
        self.assertEqual(self.session.snapshot()["navigation_deadline_monotonic"], deadline)
        self.stop()
        self.assertEqual(self.session.snapshot()["outcome"], "cancelled")
        self.assertEqual(self.session.snapshot()["freshness_wait_count"], 1)
        self.assertLessEqual(self.session.snapshot()["maximum_control_sensor_age_sim_seconds"], 0.9)
        self.assertFalse(self.vehicle.armed)

    def test_sustained_loss_lands_without_claiming_arrival(self) -> None:
        index = self.wait_for_zero()
        self.session.thread.join(timeout=3)
        self.assertFalse(self.session.thread.is_alive())
        self.assertEqual(self.session.snapshot()["outcome"], "failed")
        self.assertIn("deadline", self.session.snapshot()["error"])
        self.assertTrue(all(v == (0.0, 0.0, 0.0) for v in self.vehicle.velocities[index:]))
        self.assertEqual(self.vehicle.landings, 1)
        self.assertFalse(self.vehicle.armed)
        self.assertTrue(self.lease.acquire(blocking=False))
        self.lease.release()

    def test_local_pose_failure_during_wait_never_resumes(self) -> None:
        index = self.wait_for_zero()
        self.vehicle.pose_failed = True
        self.sensing.blocked = False
        self.session.thread.join(timeout=3)
        self.assertEqual(self.session.snapshot()["outcome"], "failed")
        self.assertIn("pose unavailable", self.session.snapshot()["error"])
        self.assertEqual(self.session.snapshot()["freshness_recovery_count"], 0)
        self.assertTrue(all(v == (0.0, 0.0, 0.0) for v in self.vehicle.velocities[index:]))

    def test_generation_change_during_wait_is_fatal(self) -> None:
        self.wait_for_zero()
        self.vehicle.controller_run_id = "new-controller"
        self.sensing.blocked = False
        self.session.thread.join(timeout=3)
        self.assertEqual(self.session.snapshot()["outcome"], "failed")
        self.assertIn("generation", self.session.snapshot()["error"].lower())
        self.assertEqual(self.session.snapshot()["freshness_recovery_count"], 0)

    def test_calibration_failure_is_not_retried(self) -> None:
        self.wait_for_zero()
        self.sensing.fatal = "Stereo calibration mismatch"
        self.sensing.blocked = False
        self.session.thread.join(timeout=3)
        self.assertEqual(self.session.snapshot()["outcome"], "failed")
        self.assertIn("calibration", self.session.snapshot()["error"])
        self.assertEqual(self.session.snapshot()["freshness_recovery_count"], 0)

    def test_cancel_and_active_reset_during_wait(self) -> None:
        self.wait_for_zero()
        with self.assertRaisesRegex(RuntimeError, "inactive|disarmed"):
            self.session.reset()
        self.stop()
        self.assertEqual(self.session.snapshot()["outcome"], "cancelled")
        self.assertEqual(self.vehicle.landings, 1)
        self.assertFalse(self.vehicle.armed)
        self.assertTrue(list(Path(self.folder.name).glob("*.json")))

    def test_repeated_waits_cannot_reset_episode_or_mission_budgets(self) -> None:
        deadline = self.session.snapshot()["navigation_deadline_monotonic"]
        for count in (1, 2):
            self.sensing.blocked = True
            await_condition(lambda: self.session.snapshot().get("freshness_wait_count") == count)
            self.sensing.blocked = False
            await_condition(
                lambda: self.session.snapshot().get("freshness_recovery_count") == count
            )
        self.sensing.blocked = True
        self.session.thread.join(timeout=3)
        self.assertEqual(self.session.snapshot()["outcome"], "failed")
        self.assertIn("episode", self.session.snapshot()["error"])
        self.assertEqual(self.session.snapshot()["navigation_deadline_monotonic"], deadline)
        self.assertEqual(self.session.snapshot()["freshness_wait_count"], 2)

    def test_slow_perception_does_not_block_zero_velocity_heartbeat(self) -> None:
        index = self.wait_for_zero()
        entered = threading.Event()
        release = threading.Event()
        original_update = self.sensing.update

        def slow_update() -> ObservedGrid:
            entered.set()
            if not release.wait(1):
                raise RuntimeError("Controlled perception deadline")
            return original_update()

        with patch.object(self.sensing, "update", side_effect=slow_update):
            self.sensing.blocked = False
            try:
                self.assertTrue(entered.wait(0.2))
                time.sleep(0.22)
                self.assertGreaterEqual(len(self.vehicle.velocities) - index, 3)
                self.assertTrue(all(v == (0.0, 0.0, 0.0) for v in self.vehicle.velocities[index:]))
                gaps = [
                    b - a
                    for a, b in zip(
                        self.vehicle.command_times[index:], self.vehicle.command_times[index + 1 :]
                    )
                ]
                self.assertTrue(gaps)
                self.assertLess(max(gaps), 0.18)
                time.sleep(0.2)
            finally:
                release.set()
            self.session.thread.join(timeout=3)
        self.assertEqual(self.session.snapshot()["outcome"], "failed")
        self.assertEqual(self.session.snapshot()["freshness_recovery_count"], 0)
        self.assertFalse(self.vehicle.armed)


class ObservationRecoveryEvidenceTests(unittest.TestCase):
    def test_episode_limit_records_exact_triggering_observation(self) -> None:
        recovery = ObservationRecovery("vehicle_1", ObservationWaitPolicy())
        for episode in range(1, 4):
            recovery.reject(
                StaleObservationError(
                    "vehicle_1", "Old mapped evidence", timestamp_seconds=episode
                ),
                float(episode),
            )
            self.assertTrue(recovery.route_ready(episode + 0.2, episode + 0.1))

        with self.assertRaisesRegex(RuntimeError, "episode limit exceeded"):
            recovery.reject(
                StaleObservationError(
                    "vehicle_1",
                    "Old mapped evidence",
                    timestamp_seconds=4.0,
                    sensor_age_sim_seconds=0.94,
                    delivery_age_wall_seconds=1.03,
                    source_session_id="source",
                ),
                4.2,
                origin="velocity_stream",
                pose_simulation_seconds=4.94,
            )

        snapshot = recovery.snapshot(4.2)
        self.assertEqual(snapshot["freshness_wait_count"], 3)
        self.assertEqual(snapshot["freshness_recovery_count"], 3)
        self.assertEqual(snapshot["freshness_abort_count"], 1)
        self.assertEqual(
            snapshot["freshness_wait_events"][-1],
            {
                "kind": "wait_aborted",
                "vehicle_id": "vehicle_1",
                "monotonic": 4.2,
                "reason": "episode limit",
                "observation_reason": "Old mapped evidence",
                "sensor_timestamp_seconds": 4.0,
                "sensor_age_sim_seconds": 0.94,
                "delivery_age_wall_seconds": 1.03,
                "source_session_id": "source",
                "origin": "velocity_stream",
                "pose_simulation_seconds": 4.94,
            },
        )
