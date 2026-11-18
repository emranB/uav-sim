"""A path's existence is distinct from sustained destination progress."""

import unittest

from common.navigation.progress import GoalProgress


class GoalProgressTests(unittest.TestCase):
    def test_bounded_detour_and_true_progress_reset_timer(self) -> None:
        progress = GoalProgress(14, 0, timeout_seconds=25)
        self.assertFalse(progress.blocked(16, 20))
        self.assertFalse(progress.blocked(13.5, 24))
        self.assertFalse(progress.blocked(13.4, 40))
        self.assertTrue(progress.blocked(13.4, 50))

    def test_small_oscillations_are_not_goal_progress(self) -> None:
        progress = GoalProgress(5.5, 0, timeout_seconds=25)
        for stamp, error in ((5, 5.6), (10, 5.4), (15, 5.5), (20, 5.35)):
            self.assertFalse(progress.blocked(error, stamp))
        self.assertTrue(progress.blocked(5.4, 25))

    def test_new_observed_route_cells_allow_a_long_detour_away_from_goal(self) -> None:
        progress = GoalProgress(9, 0, initial_route_cell=(0, 0))
        self.assertFalse(progress.blocked(10, 30, (1, 0)))
        self.assertFalse(progress.blocked(10.5, 60, (2, 0)))
        self.assertFalse(progress.blocked(10.6, 95, (2, 0)))
        self.assertTrue(progress.blocked(10.6, 100, (2, 0)))

    def test_revisited_cells_cannot_keep_an_oscillating_route_alive(self) -> None:
        progress = GoalProgress(9, 0, initial_route_cell=(0, 0))
        self.assertFalse(progress.blocked(9.1, 10, (1, 0)))
        self.assertFalse(progress.blocked(9.2, 20, (0, 0)))
        self.assertFalse(progress.blocked(9.1, 30, (1, 0)))
        self.assertTrue(progress.blocked(9.2, 50, (0, 0)))
