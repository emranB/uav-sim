"""Observed interval, writer acknowledgement and retention behavior without flight adapters."""

import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from common.interfaces.navigation import NavigationPose
from common.interfaces.replay import ReplayCaptureLimits, ReplayObservation, ReplayRecorderHealth
from common.logging.replay_capture import ReplayCapture
from common.logging.replay_store import ReplayCaptureStore


class Observations:
    def __init__(self) -> None:
        self.generation = "controller-1"
        self.epoch = 1
        self.missing = set()

    def vehicle_ids(self) -> tuple[str, ...]:
        return ("atlas", "boreal")

    def context(self) -> dict:
        return {"source_revision": "test", "world": "retained test fixture"}

    def observe(self, identifier: str) -> ReplayObservation:
        pose = (
            None
            if identifier in self.missing
            else NavigationPose(
                (1.0, 2.0, 3.0),
                0.1,
                time.monotonic(),
                self.generation,
                True,
                "OFFBOARD",
                10.0,
            )
        )
        return ReplayObservation(
            identifier,
            identifier.title(),
            self.generation,
            self.epoch,
            pose,
            "Pose unavailable" if pose is None else None,
            ReplayRecorderHealth(True),
        )


class ReplayCaptureTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.store = ReplayCaptureStore(
            Path(self.temporary.name),
            ReplayCaptureLimits(
                sample_period_seconds=0.05,
            ),
        )
        self.source = Observations()
        self.capture = ReplayCapture(self.source, self.store)
        self.addCleanup(self.capture.close)

    def wait_samples(self, count: int = 1) -> None:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if any(row["samples"] >= count for row in self.capture.snapshot()["vehicles"].values()):
                return
            time.sleep(0.01)
        self.fail("Capture did not publish expected samples")

    def finalized(self) -> dict:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            status = self.capture.snapshot()
            if status["state"] not in {"capturing", "finalizing"}:
                return self.store.manifest(status["capture_id"], verify_data=True)
            time.sleep(0.01)
        self.fail("Capture did not finalize")

    def test_explicit_selection_stop_midflight_and_restart(self) -> None:
        selection = ["atlas"]
        started = self.capture.start(selection)
        selection.append("boreal")
        self.assertEqual(started["state"], "capturing")
        self.assertTrue(started["vehicles"]["atlas"]["writer_available"])
        self.wait_samples()
        self.capture.stop()
        manifest = self.finalized()
        self.assertEqual(manifest["vehicle_ids"], ["atlas"])
        self.assertTrue(manifest["partial_mission"])
        self.assertFalse(manifest["camera_video_recorded"])
        path = self.store.path(manifest["capture_id"])
        lines = [json.loads(line) for line in (path / "vehicle-0.jsonl").read_text().splitlines()]
        for line in lines[1:]:
            stamp = line["observation"]["pose"]["received_monotonic"]
            self.assertLessEqual(manifest["started_monotonic"], stamp)
            self.assertLessEqual(stamp, manifest["stopped_monotonic"])
            self.assertTrue(line["observation"]["pose"]["armed"])
        self.store.keep(manifest["capture_id"])
        self.assertTrue(self.store.inventory()[0]["retained"])
        self.assertEqual(self.store.manifest(manifest["capture_id"]), manifest)
        self.capture.start(["boreal"])
        self.wait_samples()
        self.capture.stop()
        self.assertNotEqual(self.finalized()["capture_id"], manifest["capture_id"])

    def test_rejection_and_duplicate_start_do_not_expand_capture(self) -> None:
        for identifiers in ([], ["unknown"], ["atlas", "atlas"]):
            with self.assertRaises(ValueError):
                self.capture.start(identifiers)
        self.capture.start(["atlas"])
        with self.assertRaises(RuntimeError):
            self.capture.start(["boreal"])
        self.assertEqual(self.capture.snapshot()["vehicle_ids"], ["atlas"])

    def test_unavailable_peer_does_not_remove_measured_peer_evidence(self) -> None:
        self.source.missing.add("boreal")
        self.capture.start(["atlas", "boreal"])
        self.wait_samples()
        self.capture.stop()
        manifest = self.finalized()
        self.assertGreater(manifest["vehicles"]["atlas"]["samples"], 0)
        self.assertEqual(manifest["vehicles"]["boreal"]["samples"], 0)
        self.assertEqual(manifest["vehicles"]["boreal"]["observation_error"], "Pose unavailable")

    def test_shutdown_retains_interrupted_evidence(self) -> None:
        self.capture.start(["atlas"])
        self.wait_samples()
        self.capture.close()
        self.assertEqual(self.finalized()["state"], "interrupted")
        self.assertTrue(self.store.inventory()[0]["retained"])

    def test_generation_change_creates_explicit_segment(self) -> None:
        self.capture.start(["atlas"])
        self.wait_samples()
        self.source.generation = "controller-2"
        self.wait_samples(2)
        self.capture.stop()
        manifest = self.finalized()
        rows = [
            json.loads(line)
            for line in (self.store.path(manifest["capture_id"]) / "vehicle-0.jsonl")
            .read_text()
            .splitlines()
        ][1:]
        self.assertTrue(rows[-1]["segment_changed"])
        self.assertEqual(rows[-1]["observation"]["controller_generation"], "controller-2")

    def test_capacity_failure_never_acknowledges_capture(self) -> None:
        with patch("common.logging.replay_store.ensure_capacity", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                self.capture.start(["atlas"])
        self.assertEqual(self.capture.snapshot()["state"], "idle")

    def test_partial_writer_failure_preserves_peer(self) -> None:
        original_open = Path.open

        def open_selected(path: Path, *args, **kwargs):
            if path.name == "vehicle-1.jsonl" and args and args[0] == "x":
                raise OSError("Boreal writer rejected")
            return original_open(path, *args, **kwargs)

        with patch.object(Path, "open", open_selected):
            status = self.capture.start(["atlas", "boreal"])
        self.assertTrue(status["vehicles"]["atlas"]["writer_available"])
        self.assertFalse(status["vehicles"]["boreal"]["writer_available"])
        self.wait_samples()
        self.capture.stop()
        manifest = self.finalized()
        self.assertTrue(manifest["partial_capture"])
        self.assertIn("Boreal writer rejected", manifest["vehicles"]["boreal"]["error"])
        self.assertGreater(manifest["vehicles"]["atlas"]["samples"], 0)

    def test_disk_failure_latches_error_and_retains_interval(self) -> None:
        self.capture.start(["atlas"])
        self.wait_samples()
        with patch(
            "common.logging.replay_capture.ensure_capacity", side_effect=OSError("disk full")
        ):
            manifest = self.finalized()
        self.assertEqual(manifest["state"], "error")
        self.assertTrue(self.store.inventory()[0]["retained"])
        self.assertIn("disk full", manifest["vehicles"]["atlas"]["error"])

    def test_orphan_recovery_and_integrity(self) -> None:
        identifier, path, lease = self.store.allocate()
        (path / "vehicle-0.jsonl").write_text('{"kind":"header"}\n')
        self.assertEqual(self.store.recover_interrupted(), [])
        lease.close()
        self.assertEqual(self.store.recover_interrupted(), [identifier])
        self.assertTrue(self.store.inventory()[0]["retained"])
        self.store.manifest(identifier, verify_data=True)
        (path / "vehicle-0.jsonl").chmod(0o644)
        (path / "vehicle-0.jsonl").write_text("changed")
        with self.assertRaisesRegex(ValueError, "changed"):
            self.store.manifest(identifier, verify_data=True)

    def test_retained_store_budget_is_admission_failure_not_deletion(self) -> None:
        limits = ReplayCaptureLimits(maximum_bytes=1024, maximum_store_bytes=1024)
        store = ReplayCaptureStore(Path(self.temporary.name) / "small", limits)
        identifier, path, lease = store.allocate()
        (path / "vehicle-0.jsonl").write_text("x" * 1024)
        store.seal(identifier, {"state": "stopped"})
        lease.close()
        store.keep(identifier)
        with self.assertRaisesRegex(OSError, "budget"):
            store.allocate()
        self.assertTrue(path.exists())
