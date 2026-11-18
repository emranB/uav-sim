"""Check optical-to-PX4 frame conversion and admission of current aiding evidence."""

import math
import socket
import time
import unittest
from unittest.mock import Mock

import numpy as np
import pytest

from common.configuration.cameras import StereoConfiguration
from common.interfaces.navigation import NavigationPose
from integrations.mavlink.visual_odometry import LocalVisualFrame
from integrations.px4.visual_aiding import VisualAidingGuard, send_status
from integrations.px4.visual_navigation import StereoHorizontalNavigation


class VisualFrameTest(unittest.TestCase):
    def test_horizontal_camera_motion_and_body_lever_arm(self) -> None:
        configuration = StereoConfiguration(mount_pitch_rad=0, baseline_m=0.4, mount_height_m=0.5)
        frame = LocalVisualFrame(configuration, [0, 0, 0], [0, 0, 0, 1], (1, 0, 0, 0))
        # At a north-facing launch, right/down/forward optical displacements
        # correspond to east/down/north in NED.
        position, _ = frame.to_ned([2, 3, 4], [0, 0, 0, 1])
        np.testing.assert_allclose(position, [4, 2, 3], atol=1e-9)
        # Rotate the vehicle 90 degrees clockwise about its body centre. The
        # offset camera moves even though the body centre does not.
        half_turn = math.sqrt(0.5)
        position, orientation = frame.to_ned([0.6, 0, -0.2], [0, half_turn, 0, half_turn])
        np.testing.assert_allclose(position, [0, 0, 0], atol=1e-9)
        np.testing.assert_allclose(orientation, [half_turn, 0, 0, half_turn], atol=1e-9)


class VisualAidingTest(unittest.TestCase):
    def test_health_rejects_stale_or_inactive_aiding(self) -> None:
        guard = VisualAidingGuard(port=0)
        self.addCleanup(guard.close)
        evidence = {
            "controller_run_id": "run-1",
            "monotonic": time.monotonic(),
            "vision_xy_active": True,
            "source_samples_fresh": True,
            "xy_valid": True,
            "z_valid": True,
            "capture_stamp_valid": True,
            "xy_reset_counter": 4,
        }
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            send_status(sender, guard.port, {**evidence, "kind": "aiding"})
            send_status(
                sender,
                guard.port,
                {
                    **evidence,
                    "kind": "pose",
                    "healthy": True,
                    "map_generation": 2,
                },
            )
            self.assertEqual(guard("run-1"), (2, 4))
            with self.assertRaisesRegex(RuntimeError, "another controller"):
                guard("run-2")
            send_status(
                sender,
                guard.port,
                {
                    **evidence,
                    "kind": "aiding",
                    "vision_xy_active": False,
                },
            )
            with self.assertRaisesRegex(RuntimeError, "healthy synchronized"):
                guard("run-1")
        stale_guard = VisualAidingGuard(port=0)
        self.addCleanup(stale_guard.close)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            send_status(
                sender,
                stale_guard.port,
                {
                    **evidence,
                    "kind": "aiding",
                    "monotonic": time.monotonic() - 2,
                },
            )
            send_status(
                sender,
                stale_guard.port,
                {
                    **evidence,
                    "kind": "pose",
                    "healthy": True,
                    "map_generation": 2,
                },
            )
        with self.assertRaisesRegex(RuntimeError, "stale"):
            stale_guard("run-1")


def test_stereo_reference_ignores_only_expected_gnss_horizontal_reset() -> None:
    controller = Mock()
    controller.navigation_pose.return_value = NavigationPose(
        (100, 200, 4),
        0.4,
        1.0,
        "run-1",
        False,
        "HOLD",
        50.0,
    )
    aiding = Mock()
    status = {
        "gps_fix_type": 3,
        "gnss_xy_active": True,
        "xy_reset_counter": 4,
        "heading_reset_counter": 1,
        "z_reset_counter": 1,
        "gps_loss_injected": False,
    }
    stream = {"map_generation": 0}
    aiding.healthy_sources.return_value = (status, stream)
    aiding.horizontal_enu_at.return_value = (1.5, -0.5)
    route = StereoHorizontalNavigation(controller, aiding)
    assert route.navigation_pose().position_enu_m == (1.5, -0.5, 4)
    status.update(heading_reset_counter=2, delta_heading_rad=-0.005)
    assert route.navigation_pose().yaw_enu_rad == pytest.approx(0.395)
    route.send_navigation_velocity((0.5, 0, 0))
    assert controller.send_navigation_velocity.call_args.args[0] == pytest.approx(
        (0.5 * math.cos(0.005), 0.5 * math.sin(0.005), 0)
    )

    status.update(gps_fix_type=0, gnss_xy_active=False, xy_reset_counter=5, gps_loss_injected=True)
    controller.navigation_pose.return_value = NavigationPose(
        (101, 198, 4),
        0.4,
        1.1,
        "run-1",
        True,
        "OFFBOARD",
        50.05,
    )
    assert route.navigation_pose().position_enu_m == (1.5, -0.5, 4)
    status["xy_reset_counter"] = 6
    with pytest.raises(RuntimeError, match="Unexpected PX4 position reset"):
        route.navigation_pose()


@pytest.mark.parametrize("delta", [0.03258, -0.03258])
def test_reported_initial_magnetic_alignment_rotates_stereo_commands(delta: float) -> None:
    controller = Mock()
    controller.navigation_pose.return_value = NavigationPose(
        (0, 0, 0),
        0.4,
        1.0,
        "run-1",
        False,
        "HOLD",
        50.0,
    )
    aiding = Mock()
    status = {
        "gps_fix_type": 3,
        "gnss_xy_active": True,
        "xy_reset_counter": 4,
        "heading_reset_counter": 1,
        "z_reset_counter": 1,
        "gps_loss_injected": False,
        "mag_aligned_in_flight": False,
    }
    aiding.healthy_sources.return_value = status, {"map_generation": 0}
    aiding.horizontal_enu_at.return_value = (0.2, 0.1)
    route = StereoHorizontalNavigation(controller, aiding)
    route.navigation_pose()
    controller.navigation_pose.return_value = NavigationPose(
        (0.2, 0.1, 1.8),
        0.4 - delta,
        1.1,
        "run-1",
        True,
        "OFFBOARD",
        50.1,
    )
    status.update(heading_reset_counter=2, delta_heading_rad=delta, mag_aligned_in_flight=True)
    assert route.navigation_pose().yaw_enu_rad == pytest.approx(0.4)
    route.send_navigation_velocity((0.5, 0.1, 0.2))
    assert controller.send_navigation_velocity.call_args.args[0] == pytest.approx(
        (
            0.5 * math.cos(delta) + 0.1 * math.sin(delta),
            -0.5 * math.sin(delta) + 0.1 * math.cos(delta),
            0.2,
        )
    )
    status.update(heading_reset_counter=3)
    with pytest.raises(RuntimeError, match="Unexpected PX4 heading reset"):
        route.navigation_pose()


@pytest.mark.parametrize(
    "reported, movement_m, altitude_m, gps_loss",
    [
        (False, 0.2, 1.8, False),
        (True, 0.8, 1.8, False),
        (True, 0.2, 2.6, False),
        (True, 0.2, 1.8, True),
    ],
)
def test_unqualified_heading_change_remains_rejected(
    reported: bool,
    movement_m: float,
    altitude_m: float,
    gps_loss: bool,
) -> None:
    controller = Mock()
    controller.navigation_pose.return_value = NavigationPose(
        (0, 0, 0),
        0.4,
        1.0,
        "run-1",
        False,
        "HOLD",
        50.0,
    )
    aiding = Mock()
    status = {
        "gps_fix_type": 3,
        "gnss_xy_active": True,
        "xy_reset_counter": 4,
        "heading_reset_counter": 1,
        "z_reset_counter": 1,
        "gps_loss_injected": False,
        "mag_aligned_in_flight": False,
    }
    aiding.healthy_sources.return_value = status, {"map_generation": 0}
    aiding.horizontal_enu_at.return_value = (0.2, 0.1)
    route = StereoHorizontalNavigation(controller, aiding)
    route.navigation_pose()
    controller.navigation_pose.return_value = NavigationPose(
        (movement_m, 0, altitude_m),
        0.4,
        1.1,
        "run-1",
        True,
        "OFFBOARD",
        50.1,
    )
    status.update(
        heading_reset_counter=2,
        delta_heading_rad=0.03258,
        mag_aligned_in_flight=reported,
        gps_loss_injected=gps_loss,
    )
    with pytest.raises(RuntimeError, match="Unexpected PX4 heading reset"):
        route.navigation_pose()
