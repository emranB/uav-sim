"""Replay translation uses measured identities without invoking environment or commands."""

import time
import unittest
from unittest.mock import Mock

from common.interfaces.navigation import NavigationPose
from common.simulation_manager.perception import vehicle_perception
from integrations.replay import WorkspaceReplaySource


class ReplaySourceTests(unittest.TestCase):
    def test_native_generation_and_truth_remain_separate(self) -> None:
        service = Mock()
        service.perception = vehicle_perception()
        service.navigation.ranges.latest.side_effect = RuntimeError("No range observation fixture")
        service.features.configuration = {"label": "Atlas", "display_launch_enu_m": [6, 0, 0]}
        service.vehicle.snapshot.return_value = {
            "run_id": "new",
            "armed": True,
            "recording": {"available": True, "bytes": 123, "path": "diagnostic.jsonl"},
        }
        service.vehicle.navigation_pose.return_value = NavigationPose(
            (1, 2, 3),
            0,
            time.monotonic(),
            "new",
            True,
            "OFFBOARD",
            10,
        )
        service.navigation.snapshot.return_value = {
            "controller_run_id": "new",
            "run_id": "mission-1",
            "home_local_enu_m": [0, 0, 0],
            "state": "navigating",
            "leg": "outbound",
            "saved_mission": {"name": "flight", "sha256": "digest"},
            "visual_localization": {"aiding": {"controller_run_id": "old", "gnss_xy_active": True}},
        }
        service.scene_source.snapshot.return_value = {
            "epoch": 2,
            "stale": False,
            "pose_received_monotonic": {"atlas": time.monotonic()},
            "poses": [
                {
                    "vehicle_id": "atlas",
                    "simulation_time_seconds": 10,
                    "position_enu_m": [7, 2, 3],
                    "orientation_xyzw": [0, 0, 0, 1],
                }
            ],
        }
        observed = WorkspaceReplaySource({"atlas": service}).observe("atlas")
        self.assertIsNone(observed.aiding.gnss_xy_active)
        self.assertEqual(observed.pose.position_enu_m, (1, 2, 3))
        self.assertEqual(observed.evaluation_truth.position_enu_m, [7, 2, 3])
        self.assertEqual(observed.frame_binding.scene_launch_enu_m, (6, 0, 0))
        self.assertEqual(observed.recorder.bytes_written, 123)
        self.assertEqual(observed.mission.execution_id, "mission-1")
        self.assertEqual(observed.mission.leg, "outbound")
        self.assertIsNotNone(observed.evaluation_truth_received_monotonic)
        service.execute.assert_not_called()
        service.features.environment.snapshot.assert_not_called()
        service.vehicle.send_navigation_velocity.assert_not_called()
