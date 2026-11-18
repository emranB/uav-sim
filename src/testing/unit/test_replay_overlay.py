"""Replay trails reveal observed motion only, with north-up projection and explicit gaps."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from common.logging.replay_timeline import RecordedReplay, ReplaySample
from integrations.blender.overlay import ReplayOverlay, ReplayViewport, progressive_trail


class ReplayOverlayTests(unittest.TestCase):
    def test_progressive_trail_never_appears_before_receipt_or_joins_reset_or_stale_pose(
        self,
    ) -> None:
        rows = []
        for elapsed, east, segment, received in (
            (0.05, 0, 1, 100.05),
            (0.155, 1, 1, 100.155),
            (0.25, 2, 2, 100.25),
            (1, 3, 2, 100.25),
        ):
            rows.append(
                ReplaySample(
                    elapsed,
                    {"pose": {"position_enu_m": [east, 0, 0], "received_monotonic": received}},
                    segment,
                )
            )
        replay = SimpleNamespace(
            samples={"atlas": rows}, started_monotonic=100, duration_seconds=2, bindings={}
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "overlay.ass"
            overlay = ReplayOverlay(path, 1600, 900)
            count = progressive_trail(
                overlay,
                replay,
                "atlas",
                ReplayViewport((0, 4, -1, 1), (0, 0, 400, 200)),
                scene=False,
                colour="FFFFFF",
            )
            overlay.close()
            self.assertEqual(count, 1)
            events = [line for line in path.read_text().splitlines() if line.startswith("Dialogue")]
            self.assertEqual(len(events), 2)
            self.assertTrue(all(",0:00:00.16," in event for event in events))

    def test_scene_and_estimated_positions_remain_distinct_and_identity_scoped(self) -> None:
        observation = {
            "vehicle_id": "atlas",
            "pose": {"position_enu_m": [0, 0, 0]},
            "evaluation_truth": {"vehicle_id": "atlas", "position_enu_m": [6, 2, 3]},
        }
        self.assertEqual(RecordedReplay.scene_position(observation), (6, 2, 3))
        observation["evaluation_truth"]["vehicle_id"] = "boreal"
        with self.assertRaisesRegex(ValueError, "another vehicle"):
            RecordedReplay.scene_position(observation)

    def test_projection_preserves_scale_with_north_up(self) -> None:
        viewport = ReplayViewport((0, 10, 0, 10), (10, 20, 200, 100))
        self.assertEqual(viewport.project((0, 0)), (60, 120))
        self.assertEqual(viewport.project((10, 10)), (160, 20))
