"""Navigation authority, cancellation and input-loss outcomes without GUI or simulator."""

import math
import tempfile
import threading
import time
import unittest
from pathlib import Path

from common.interfaces.navigation import NavigationPose, RangeScan
from common.navigation.session import NavigationSession


class EstimatedVehicle:
    def __init__(self) -> None:
        self.position = (0, 0, 0)
        self.armed = False
        self.mode = "HOLD"
        self.landings = 0
        self.velocities = []

    def navigation_pose(self) -> NavigationPose:
        return NavigationPose(
            self.position, 0, time.monotonic(), "controller", self.armed, self.mode
        )

    def send_navigation_velocity(self, velocity_enu_m_s: tuple) -> None:
        self.velocities.append(velocity_enu_m_s)
        self.position = tuple(self.position[i] + velocity_enu_m_s[i] for i in range(3))

    def navigation_armed(self) -> bool:
        return self.armed

    def begin_navigation(self) -> None:
        self.armed, self.mode = True, "OFFBOARD"

    def end_navigation(self) -> None:
        self.mode = "LOITER"

    def land(self) -> None:
        self.armed = False
        self.landings += 1


class MeasuredRanges:
    def __init__(self, fail_after: int = 100) -> None:
        self.calls, self.fail_after = 0, fail_after
        self.enabled = True

    def latest(self) -> RangeScan:
        self.calls += 1
        if self.calls > self.fail_after:
            raise RuntimeError("Range stream stale")
        return RangeScan(
            time.monotonic(), time.monotonic(), -math.pi, 2 * math.pi / 719, 20, (math.inf,) * 720
        )

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled

    def close(self) -> None:
        pass


class NavigationSessionTests(unittest.TestCase):
    def test_stale_sensor_lands_and_releases_authority(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            vehicle, lease = EstimatedVehicle(), threading.Lock()
            session = NavigationSession(
                "vehicle_2", vehicle, MeasuredRanges(2), lease, Path(directory)
            )
            session.start({"goal_offset_enu_m": [0, 10]})
            self.assertFalse(lease.acquire(blocking=False))
            session.thread.join(timeout=10)
            self.assertEqual(session.snapshot()["state"], "failed")
            self.assertIn("stale", session.snapshot()["error"])
            self.assertEqual(vehicle.landings, 1)
            self.assertTrue(lease.acquire(blocking=False))
            lease.release()

    def test_cancellation_does_not_report_goal_reached(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            vehicle = EstimatedVehicle()
            session = NavigationSession(
                "vehicle_1", vehicle, MeasuredRanges(), threading.Lock(), Path(directory)
            )
            session.start({"goal_offset_enu_m": [0, 10]})
            time.sleep(1.6)
            session.cancel()
            session.thread.join(timeout=10)
            self.assertEqual(session.snapshot()["state"], "cancelled")
            self.assertFalse(vehicle.armed)

    def test_pose_loss_requests_landing_and_never_claims_arrival(self) -> None:
        class LostPoseVehicle(EstimatedVehicle):
            def navigation_pose(self) -> NavigationPose:
                if self.armed:
                    self.armed = False
                    raise RuntimeError("Estimated local pose stale")
                return super().navigation_pose()

        with tempfile.TemporaryDirectory() as directory:
            vehicle = LostPoseVehicle()
            session = NavigationSession(
                "vehicle_1", vehicle, MeasuredRanges(), threading.Lock(), Path(directory)
            )
            session.start({"goal_offset_enu_m": [0, 10]})
            session.thread.join(timeout=10)
            self.assertEqual(session.snapshot()["outcome"], "failed")
            self.assertIn("pose stale", session.snapshot()["error"])
            self.assertEqual(vehicle.landings, 1)

    def test_clear_observed_goal_arrives_with_bounded_motion(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            vehicle = EstimatedVehicle()
            session = NavigationSession(
                "vehicle_1", vehicle, MeasuredRanges(1000), threading.Lock(), Path(directory)
            )
            session.start({"goal_offset_enu_m": [0, 3]})
            session.thread.join(timeout=15)
            self.assertEqual(session.snapshot()["outcome"], "reached")
            self.assertFalse(vehicle.armed)
            self.assertLessEqual(max(math.hypot(*v[:2]) for v in vehicle.velocities), 0.8)
            self.assertLessEqual(
                max(math.dist(a, b) for a, b in zip(vehicle.velocities, vehicle.velocities[1:])),
                0.050001,
            )

    def test_reset_clears_runtime_state_but_preserves_completed_record(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            record = Path(directory) / "previous.json"
            record.write_text("historical record")
            ranges = MeasuredRanges()
            session = NavigationSession(
                "vehicle_2", EstimatedVehicle(), ranges, threading.Lock(), Path(directory)
            )
            session.update(
                state="blocked",
                goal_local_enu_m=[0, 14],
                position_enu_m=[0, 10, 4],
                error="old error",
                report=str(record),
            )
            ranges.set_enabled(False)
            session.reset()
            self.assertTrue(ranges.enabled)
            self.assertEqual(
                session.snapshot(),
                {
                    "schema_version": 1,
                    "state": "idle",
                    "vehicle_id": "vehicle_2",
                    "available": True,
                    "active": False,
                },
            )
            self.assertEqual(record.read_text(), "historical record")

    def test_goal_rejects_nonfinite_and_unbounded_values(self) -> None:
        session = NavigationSession(
            "vehicle_1", EstimatedVehicle(), MeasuredRanges(), threading.Lock(), Path("unused")
        )
        for goal in ([0, math.nan], [0, 100], [0, True], [0], "goal"):
            with self.assertRaises(ValueError):
                session.start({"goal_offset_enu_m": goal})

    def test_stereo_goal_outside_envelope_is_rejected_before_source_or_authority_use(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            vehicle, lease = EstimatedVehicle(), threading.Lock()
            session = NavigationSession(
                "vehicle_1", vehicle, MeasuredRanges(), lease, Path(directory), stereo=object()
            )
            with self.assertRaisesRegex(ValueError, "2..10m"):
                session.start({"goal_offset_enu_m": [0, 14], "obstacle_mode": "camera"})
            self.assertFalse(vehicle.armed)
            self.assertFalse(vehicle.velocities)
            self.assertTrue(lease.acquire(blocking=False))
            lease.release()
