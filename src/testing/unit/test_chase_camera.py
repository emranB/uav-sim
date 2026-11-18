"""Chase presentation remains bounded at heading wrap, turns and position discontinuities."""

import math
import unittest

from integrations.blender.camera_policy import advance_chase_camera


class ChaseCameraTests(unittest.TestCase):
    def test_frontal_camera_aims_beyond_aircraft_in_its_view_direction(self) -> None:
        camera = advance_chase_camera((0, 0, 4), 0, None, 0, 4, 10, back_m=-5.5)
        self.assertGreater(camera.position[0], 0)
        self.assertLess(camera.target[0], 0)
        self.assertGreater(camera.target[2], 4)

    def test_heading_wrap_uses_short_turn_without_flipping(self) -> None:
        initial = advance_chase_camera((0, 0, 4), math.pi - 0.02, None, 3, 4, 10)
        following = advance_chase_camera((0, 0, 4), -math.pi + 0.02, initial, 3, 4, 10)
        self.assertLess(abs(following.heading_rad - initial.heading_rad), 0.05)

    def test_position_jump_and_turn_are_bounded_per_frame(self) -> None:
        initial = advance_chase_camera((0, 0, 4), 0, None, 3, 4, 10)
        following = advance_chase_camera((20, 30, 50), math.pi, initial, 3, 4, 10)
        self.assertLessEqual(math.dist(initial.position, following.position), 0.50000001)
        self.assertLessEqual(abs(following.heading_rad - initial.heading_rad), 0.08000001)

    def test_separate_aircraft_states_and_constant_safe_side(self) -> None:
        atlas = advance_chase_camera((0, 0, 4), 0, None, -3, 6, 10)
        boreal = advance_chase_camera((6, 0, 4), 0, None, 3, 4, 10)
        self.assertEqual(atlas.position, (-5.5, -3, 10))
        self.assertEqual(boreal.position, (0.5, 3, 8))
        self.assertEqual(atlas.target, (0.4, 0.0, 4.4))
