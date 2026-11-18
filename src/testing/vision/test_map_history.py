"""Retained geometry is historical data, not a recovered localization estimate."""

import unittest

from integrations.vision.live import retain_tracked_geometry


class MapHistoryTests(unittest.TestCase):
    def test_lost_tracking_retains_geometry_but_not_pose_validity(self) -> None:
        tracked = {
            "position_valid": True,
            "timestamp_seconds": 1,
            "map_id": 0,
            "map_generation": 2,
            "trajectory": [[0, 0, 0]],
            "landmarks": [[1, 2, 3]],
            "surfaces": [[2, 3, 4]],
        }
        _, retained = retain_tracked_geometry(tracked, {})
        lost, _ = retain_tracked_geometry(
            {
                "position_valid": False,
                "timestamp_seconds": 2,
                "map_id": -1,
                "trajectory": [],
                "landmarks": [],
                "surfaces": [],
            },
            retained,
        )
        self.assertFalse(lost["position_valid"])
        self.assertTrue(lost["map_frozen"])
        self.assertEqual(lost["display_map_generation"], 2)
        self.assertEqual(lost["last_tracked_timestamp_seconds"], 1)
        self.assertEqual(lost["timestamp_seconds"], 2)
        self.assertEqual(lost["landmarks"], [[1, 2, 3]])

    def test_new_session_cannot_inherit_geometry_without_explicit_history(self) -> None:
        current, retained = retain_tracked_geometry(
            {
                "position_valid": False,
                "timestamp_seconds": 1,
                "trajectory": [],
                "landmarks": [],
                "surfaces": [],
            },
            {},
        )
        self.assertFalse(current.get("map_frozen", False))
        self.assertEqual(current["landmarks"], [])
        self.assertEqual(retained, {})
