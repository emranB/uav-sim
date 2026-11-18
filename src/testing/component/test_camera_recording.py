"""Observer buffers respect vehicle generations and release their subscriptions."""

import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock

from gz.msgs10.image_pb2 import Image

from gazebo.camera_streams import CameraRecordingClock, VehicleCameras


class CameraRecordingTests(unittest.TestCase):
    def test_prior_epoch_images_are_not_recorded_as_a_new_vehicle_session(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            definition = Mock()
            definition.vehicle_id = "vehicle_2"
            definition.model_name = "x500_1"
            definition.runtime_directory = root / "runtime/vehicle_2"
            definition.output_directory.return_value = root / "vehicles/vehicle_2"
            node = Mock()
            observer = VehicleCameras(node, definition, root, minimum_timestamp_ns=2_000_000_000)

            def frame(seconds: int) -> Image:
                message = Image(width=320, height=240, step=960, data=b"\x00" * (320 * 240 * 3))
                message.header.stamp.sec = seconds
                return message

            for side in ("left", "right"):
                observer.receive(side, frame(1))
            observer.flush()
            status = json.loads(observer.status_path.read_text())
            self.assertEqual(status["recorded_pairs"], 0)
            release_archive = threading.Event()
            persist_pair = observer.persist_pair

            def delayed_archive(*arguments) -> None:
                release_archive.wait(5)
                persist_pair(*arguments)

            observer.persist_pair = delayed_archive
            for side in ("left", "right"):
                observer.receive(side, frame(2))
            observer.flush()
            # Host storage can stall while the live pair is already available.
            try:
                self.assertFalse(observer.archive_future.done())
                status = json.loads(observer.status_path.read_text())
                self.assertEqual(status["simulation_timestamp_ns"], 2_000_000_000)
                self.assertTrue((observer.preview_directory / "latest-left.ppm").is_file())
            finally:
                release_archive.set()
            observer.archive_future.result(timeout=5)
            manifest = json.loads((observer.directory / "report.json").read_text())
            self.assertEqual(manifest["vehicle_id"], "vehicle_2")
            self.assertEqual(len(manifest["pairs"]), 1)
            self.assertEqual(manifest["pairs"][0]["simulation_timestamp_ns"], 2_000_000_000)
            for seconds in range(3, 16):
                observer.receive("left", frame(seconds))
            observer.flush()
            status = json.loads(observer.status_path.read_text())
            self.assertGreater(status["dropped_pairs"], 0)
            observer.close()
            self.assertFalse(observer.status_path.exists())
            self.assertEqual(node.unsubscribe.call_count, 2)

    def test_completed_burst_preserves_observer_cadence_and_counts_overflow(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            definition = Mock()
            definition.vehicle_id = "vehicle_1"
            definition.model_name = "x500_0"
            definition.runtime_directory = root / "runtime"
            definition.output_directory.return_value = root
            observer = VehicleCameras(Mock(), definition, root)
            try:

                def deliver(index: int) -> None:
                    message = Image(width=320, height=240, step=960, data=b"\x00" * (320 * 240 * 3))
                    message.header.stamp.sec, message.header.stamp.nsec = divmod(
                        index * 100_000_000, 1_000_000_000
                    )
                    for side in ("left", "right"):
                        observer.receive(side, message)

                for index in range(1, 7):
                    deliver(index)
                observer.flush()
                status = json.loads(observer.status_path.read_text())
                self.assertEqual(
                    [frame["simulation_timestamp_ns"] for frame in status["available_frames"]],
                    [index * 100_000_000 for index in range(1, 7)],
                )
                self.assertEqual(status["dropped_pairs"], 0)
                # Late duplicates never replace a pending new pair or its receipt clock.
                deliver(6)
                observer.flush()
                status = json.loads(observer.status_path.read_text())
                self.assertEqual(len(status["available_frames"]), 6)
                for index in range(7, 23):
                    deliver(index)
                observer.flush()
                status = json.loads(observer.status_path.read_text())
                self.assertEqual(status["dropped_pairs"], 4)
                self.assertEqual(
                    [frame["simulation_timestamp_ns"] for frame in status["available_frames"]],
                    [index * 100_000_000 for index in range(11, 23)],
                )
            finally:
                observer.close()

    def test_clock_reset_replaces_old_minimum_timestamp_even_before_controller_change(self) -> None:
        clock = CameraRecordingClock()
        self.assertIsNone(clock.snapshot())
        clock.observe(900_000_000_000)
        old_epoch, old_minimum = clock.snapshot()
        clock.observe(100_000_000)
        epoch, minimum = clock.snapshot()
        self.assertEqual(epoch, old_epoch + 1)
        self.assertLess(minimum, old_minimum)
        self.assertNotEqual(("same-controller", epoch), ("same-controller", old_epoch))
        clock.observe(100_000_000)
        self.assertEqual(clock.snapshot(), (epoch, minimum))
        clock.observe(200_000_000)
        self.assertEqual(clock.snapshot(), (epoch, 200_000_000))
