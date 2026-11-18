"""Controlled transport callbacks must not replace the newest complete stereo batch."""

import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from gz.msgs10.image_pb2 import Image

from common.configuration.fleet import VehicleDefinition
from integrations.vision.navigation_stereo import VehicleStereoObstacles


class FakeNode:
    def subscribe(self, *args: object) -> bool:
        return True

    def unsubscribe(self, topic: str) -> None:
        pass


class FakeGeometry:
    def __init__(self, *args: object, **kwargs: object) -> None:
        self.sample_stride = 4
        self.vertical_sample_stride = 2


class ObservedWorker(VehicleStereoObstacles):
    def __init__(self, *args: object) -> None:
        self.selected = []
        self.completed = threading.Event()
        super().__init__(*args)

    def process(self, batch: tuple) -> None:
        self.selected.append((batch[0], batch[1]))
        self.completed.set()


def deliver(worker: ObservedWorker, stamp_seconds: int) -> None:
    message = Image(width=320, height=240, step=960, data=b"\0" * (320 * 240 * 3))
    message.header.stamp.sec = stamp_seconds
    for rig in range(6):
        for side in ["left", "right"]:
            worker.receive(rig, side, message)


class StereoBatchOrderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        (root / "px4.run").write_text("controller-a")
        self.marker = root / "px4.run"
        self.runtime_patch = patch.object(
            VehicleDefinition, "runtime_directory", property(lambda _: root)
        )
        self.geometry_patch = patch(
            "integrations.vision.navigation_stereo.StereoGeometry", FakeGeometry
        )
        self.runtime_patch.start()
        self.geometry_patch.start()
        self.addCleanup(self.runtime_patch.stop)
        self.addCleanup(self.geometry_patch.stop)
        definition = VehicleDefinition("vehicle_1", "Fixture", 0, 1, 14540, 14640, (0, 0, 0))
        self.worker = ObservedWorker(FakeNode(), definition, "cpu")
        self.addCleanup(self.worker.close)
        self.worker.tick()

    def test_late_old_completion_cannot_replace_newer_and_reset_allows_new_epoch(self) -> None:
        deliver(self.worker, 101)
        deliver(self.worker, 100)
        self.worker.tick()
        self.assertTrue(self.worker.completed.wait(2))
        self.assertEqual(self.worker.selected[0][0], 101_000_000_000)
        self.worker.completed.clear()
        self.marker.write_text("controller-b")
        self.worker.tick()
        deliver(self.worker, 10)
        self.worker.tick()
        self.assertTrue(self.worker.completed.wait(2))
        self.assertEqual(self.worker.selected[-1][0], 10_000_000_000)

    def test_duplicate_completion_preserves_original_first_delivery_clock(self) -> None:
        with patch("integrations.vision.navigation_stereo.time.monotonic", return_value=1.0):
            deliver(self.worker, 101)
        with patch("integrations.vision.navigation_stereo.time.monotonic", return_value=2.0):
            deliver(self.worker, 101)
        self.worker.tick()
        self.assertTrue(self.worker.completed.wait(2))
        self.assertEqual(self.worker.selected[0][1], 1.0)
