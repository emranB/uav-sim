"""A processing stall must not turn regular capture into alternating frame intervals."""

import unittest

from integrations.vision.cadence import live_frame_rate, next_stereo_frame


class StereoCadenceTests(unittest.TestCase):
    def test_estimator_rate_cannot_exceed_capture_or_accept_invalid_rates(self) -> None:
        self.assertEqual(live_frame_rate(10, 10), 10)
        self.assertEqual(live_frame_rate(3, 10), 3)
        for rate in (0, 11, True, 2.5):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                live_frame_rate(10, rate)

    def test_backlog_is_processed_in_order_at_a_fixed_rate(self) -> None:
        status = {
            "available_frames": [
                {"simulation_timestamp_ns": index * 100_000_000, "received_wall_time": index}
                for index in range(1, 10)
            ]
        }
        selected = next_stereo_frame(status, 200_000_000, 5)
        self.assertEqual(selected["simulation_timestamp_ns"], 400_000_000)
        self.assertEqual(selected["received_wall_time"], 4)
        self.assertEqual(
            next_stereo_frame(status, 400_000_000, 5)["simulation_timestamp_ns"], 600_000_000
        )
        self.assertIsNone(next_stereo_frame(status, 900_000_000, 5))

    def test_startup_uses_the_latest_pair_and_quantized_steps_are_accepted(self) -> None:
        status = {
            "available_frames": [{"simulation_timestamp_ns": 333_000_000, "received_wall_time": 1}]
        }
        self.assertEqual(next_stereo_frame(status, 0, 3)["simulation_timestamp_ns"], 333_000_000)
        self.assertEqual(next_stereo_frame(status, -1, 3), status["available_frames"][-1])
