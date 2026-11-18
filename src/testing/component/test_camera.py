"""Camera acknowledgement must follow observed animation completion."""

import unittest
from unittest.mock import Mock, patch

from gazebo.viewer_control import GazeboViewer


class CameraTests(unittest.TestCase):
    def test_follow_toggle_preserves_current_camera_offset(self) -> None:
        scene = Mock()
        scene.snapshot.return_value = {
            "viewer": {
                "available": True,
                "camera_position_enu_m": [2, -3, 5],
            }
        }
        with patch("gazebo.viewer_control.GazeboControlConnection") as connection:
            viewer = GazeboViewer(scene)
            viewer.set_follow(True, (0, 0, 1))
        request = connection.return_value.request.call_args
        self.assertEqual(request.args[0], "camera_follow")
        self.assertEqual(request.args[1]["offset"], (2, -3, 4))

    def test_follow_starts_after_requested_pose_is_observed(self) -> None:
        scene = Mock()
        scene.snapshot.return_value = {
            "viewer": {
                "available": True,
                "camera_position_enu_m": [-2.5, -2.5, 1.8],
            }
        }
        with patch("gazebo.viewer_control.GazeboControlConnection") as connection:
            viewer = GazeboViewer(scene)
            viewer.set_camera("chase", True, (0, 0, 0))
        self.assertEqual(
            connection.return_value.request.call_args_list[-1].args[0], "camera_follow"
        )
        self.assertGreaterEqual(scene.snapshot.call_count, 3)

    def test_missing_observed_pose_never_enables_follow(self) -> None:
        scene = Mock()
        scene.snapshot.return_value = {"viewer": {"available": False}}
        with (
            patch("gazebo.viewer_control.GazeboControlConnection") as connection,
            patch("gazebo.viewer_control.time.monotonic", side_effect=[0, 11]),
        ):
            viewer = GazeboViewer(scene)
            with self.assertRaisesRegex(TimeoutError, "did not reach"):
                viewer.set_camera("chase", True, (0, 0, 0))
        self.assertEqual(connection.return_value.request.call_count, 1)
