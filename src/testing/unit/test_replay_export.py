"""Offline export preserves exact captured intervals, identities and original evidence."""

import csv
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock

from common.interfaces.replay import ReplayCaptureLimits
from common.logging.replay_export import ReplayExportJobs
from common.logging.replay_store import ReplayCaptureStore
from common.logging.replay_timeline import RecordedReplay
from common.logging.storage import write_json_atomic


class ReplayExportTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        self.store = ReplayCaptureStore(root / "captures", ReplayCaptureLimits())
        self.identifier, path, lease = self.store.allocate()
        write_json_atomic(path / "context.json", {"complete_render_scene_retained": False})
        records = []
        for offset in (0.1, 1.0, 2.0, 3.0, 3.9):
            records.append(
                {
                    "kind": "observation",
                    "vehicle_id": "atlas",
                    "observed_monotonic": 100 + offset,
                    "segment_changed": offset in (0.1, 3.0),
                    "observation": {
                        "vehicle_id": "atlas",
                        "controller_generation": "run-1",
                        "world_epoch": 1,
                        "aiding": {"map_generation": 1},
                        "evaluation_truth": None,
                        "pose": {
                            "position_enu_m": [offset, 0, 0],
                            "yaw_enu_rad": 0,
                            "received_monotonic": 100 + offset,
                            "run_id": "run-1",
                            "armed": True,
                            "flight_mode": "OFFBOARD",
                            "simulation_timestamp_seconds": offset,
                        },
                    },
                }
            )
        (path / "vehicle-0.jsonl").write_text("".join(json.dumps(row) + "\n" for row in records))
        self.manifest = self.store.seal(
            self.identifier,
            {
                "vehicle_ids": ["atlas"],
                "state": "stopped",
                "duration_wall_seconds": 4,
                "started_monotonic": 100,
                "stopped_monotonic": 104,
            },
        )
        lease.close()
        self.jobs = ReplayExportJobs(self.store, root / "exports")
        self.addCleanup(self.jobs.close)

    def finished(self) -> dict:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            status = self.jobs.snapshot()
            if status["state"] not in {"queued", "running", "rendering"}:
                return status
            time.sleep(0.01)
        self.fail("Offline export did not finish")

    def test_exact_subinterval_and_truth_separation(self) -> None:
        self.jobs.start(
            {"capture_id": self.identifier, "start_offset_seconds": 1, "end_offset_seconds": 3}
        )
        status = self.finished()
        self.assertEqual(status["state"], "completed")
        path = Path(status["path"])
        with (path / "estimated-trajectories.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual([float(row["east_m"]) for row in rows], [1, 2, 3])
        self.assertEqual([float(row["elapsed_wall_seconds"]) for row in rows], [0, 1, 2])
        self.assertEqual([row["segment"] for row in rows], ["1", "1", "2"])
        self.assertEqual(status["estimated_pose_count"], 3)
        self.assertEqual(self.store.manifest(self.identifier, verify_data=True), self.manifest)
        export = json.loads((path / "export-manifest.json").read_text())
        self.assertFalse(export["camera_video_recorded"])
        self.assertFalse(export["scene_complete"])

    def test_invalid_interval_and_missing_renderer_fail_before_admission(self) -> None:
        for start, end in ((-1, 2), (2, 1), (0, 5), (0, float("nan")), (False, 1)):
            with self.assertRaises(ValueError):
                self.jobs.start(
                    {
                        "capture_id": self.identifier,
                        "start_offset_seconds": start,
                        "end_offset_seconds": end,
                    }
                )
        with self.assertRaisesRegex(RuntimeError, "not configured"):
            self.jobs.start({"capture_id": self.identifier}, render=True)
        self.assertEqual(self.jobs.snapshot()["state"], "idle")

    def test_modified_evidence_is_an_error_never_render_success(self) -> None:
        path = self.store.path(self.identifier) / "vehicle-0.jsonl"
        path.chmod(0o644)
        path.write_text("changed")
        self.jobs.start({"capture_id": self.identifier})
        result = self.finished()
        self.assertEqual(result["state"], "error")
        self.assertIn("changed", result["error"])

    def test_cancel_releases_only_offline_job_and_preserves_capture(self) -> None:
        renderer = Mock()
        renderer.availability.return_value = {"available": True}

        def render(destination, cancelled, check, progress):
            while not cancelled.wait(0.01):
                check()
            check()

        renderer.render.side_effect = render
        self.jobs.renderer = renderer
        self.jobs.start({"capture_id": self.identifier}, render=True)
        deadline = time.monotonic() + 2
        while self.jobs.snapshot()["state"] != "rendering" and time.monotonic() < deadline:
            time.sleep(0.01)
        self.jobs.cancel()
        self.assertEqual(self.finished()["state"], "cancelled")
        self.store.manifest(self.identifier, verify_data=True)

    def test_output_budget_failure_preserves_original_evidence(self) -> None:
        self.jobs.maximum_output_bytes = 256
        self.jobs.start({"capture_id": self.identifier})
        result = self.finished()
        self.assertEqual(result["state"], "error")
        self.assertIn("byte limit", result["error"])
        self.store.manifest(self.identifier, verify_data=True)

    def test_timeline_never_uses_future_observation_and_keeps_reset_segments(self) -> None:
        self.jobs.start({"capture_id": self.identifier})
        timeline = RecordedReplay(Path(self.finished()["path"]))
        self.assertIsNone(timeline.at("atlas", 0.099))
        self.assertAlmostEqual(timeline.at("atlas", 0.999).elapsed_wall_seconds, 0.1)
        self.assertEqual(timeline.at("atlas", 2.99).segment, 1)
        self.assertEqual(timeline.at("atlas", 3).segment, 2)
        self.assertEqual(timeline.position("atlas", timeline.at("atlas", 1).observation), (1, 0, 0))
        self.assertFalse(timeline.shared_frame)
        self.assertIsNone(timeline.planned_markers("atlas"))
        self.assertEqual(timeline.obstacle_footprints(), [])

    def test_timeline_uses_explicit_binding_and_planned_targets_separately(self) -> None:
        self.jobs.start({"capture_id": self.identifier})
        timeline = RecordedReplay(Path(self.finished()["path"]))
        timeline.shared_frame = True
        timeline.bindings["atlas"] = {
            "estimated_home_enu_m": [1, 2, 3],
            "scene_launch_enu_m": [10, 20, 0],
        }
        timeline.missions["atlas"] = {"planned_goal_offset_enu_m": [2.5, 1.5, 0]}
        observed = timeline.at("atlas", 1).observation
        self.assertEqual(timeline.position("atlas", observed), (10, 18, -3))
        self.assertEqual(timeline.planned_markers("atlas"), ((10, 20, 0), (12.5, 21.5, 0)))
        timeline.context["retained_world_sdf"] = {"contents": "<sdf/>"}
        self.assertEqual(timeline.obstacle_footprints(), [])
