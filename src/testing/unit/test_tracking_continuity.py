"""Frozen points and sparse UI sampling cannot hide tracking losses."""

import unittest

from testing.simulation.tracking import TrackingContinuity


class TrackingContinuityTests(unittest.TestCase):
    def test_counter_deltas_include_losses_between_samples(self) -> None:
        measurement = TrackingContinuity()
        for frames, tracked in ((10, 10), (110, 80)):
            measurement.observe(
                {
                    "processed_frames": frames,
                    "tracked_frames": tracked,
                    "session_id": "one",
                    "map_id": 0,
                    "map_generation": 0,
                    "position_valid": True,
                    "stale": False,
                }
            )
        self.assertFalse(measurement.result()["passed"])
        self.assertEqual(measurement.result()["tracked_fraction"], 0.7)

    def test_frozen_geometry_and_map_switch_do_not_pass(self) -> None:
        measurement = TrackingContinuity()
        for frames, map_id in ((1, 0), (30, 1)):
            measurement.observe(
                {
                    "processed_frames": frames,
                    "tracked_frames": frames,
                    "session_id": "one",
                    "map_id": map_id,
                    "map_generation": 0,
                    "position_valid": True,
                    "stale": False,
                }
            )
        self.assertFalse(measurement.result()["passed"])
        stable = TrackingContinuity()
        for frames in (1, 30):
            stable.observe(
                {
                    "processed_frames": frames,
                    "tracked_frames": frames,
                    "session_id": "one",
                    "map_id": 0,
                    "map_generation": 0,
                    "position_valid": True,
                    "stale": False,
                }
            )
        self.assertTrue(stable.result()["passed"])
        stable.observe(
            {
                "processed_frames": 1000,
                "tracked_frames": 999,
                "session_id": "one",
                "map_id": 0,
                "map_generation": 0,
                "position_valid": True,
                "stale": False,
                "tracking_loss_events": 1,
            }
        )
        self.assertFalse(stable.result()["passed"])
        stable.observe(
            {
                "processed_frames": 1030,
                "tracked_frames": 1029,
                "session_id": "one",
                "map_id": 0,
                "map_generation": 0,
                "position_valid": True,
                "stale": False,
                "input_gap_events": 1,
            }
        )
        self.assertEqual(stable.result()["input_gap_events"], 1)
        self.assertFalse(stable.result()["passed"])

    def test_reused_map_id_with_new_origin_does_not_pass(self) -> None:
        measurement = TrackingContinuity()
        for frames, generation in ((1, 0), (30, 1)):
            measurement.observe(
                dict(
                    processed_frames=frames,
                    tracked_frames=frames,
                    session_id="one",
                    map_id=0,
                    map_generation=generation,
                    position_valid=True,
                    stale=False,
                )
            )
        self.assertFalse(measurement.result()["passed"])
        self.assertEqual(len(measurement.result()["coordinate_frames"]), 2)
