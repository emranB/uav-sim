"""Freshness and vehicle isolation are required for camera previews."""

import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from common.logging.camera_status import CameraStatus


class CameraStatusTests(unittest.TestCase):
    def test_secondary_service_uses_shared_recording_root_for_preview_paths(self) -> None:
        from common.configuration.runtime import RuntimeSettings
        from common.simulation_manager.server import SimulationService

        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            runtime = root / "runtime"
            status_directory = runtime / "vehicle_2"
            status_directory.mkdir(parents=True)
            images = root / "vehicles/vehicle_2/perception/run-test"
            images.mkdir(parents=True)
            (images / "latest-left.ppm").write_bytes(b"secondary pixels")
            (status_directory / "stereo.json").write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "vehicle_id": "vehicle_2",
                        "error": "",
                        "received_wall_time": time.time(),
                        "directory": "vehicles/vehicle_2/perception/run-test",
                    }
                ),
                encoding="utf-8",
            )
            with patch(
                "common.simulation_manager.server.CameraStatus",
                side_effect=lambda output, vehicle: CameraStatus(output, vehicle, runtime),
            ):
                service = SimulationService(
                    RuntimeSettings(output_directory=root / "vehicles/vehicle_2"),
                    Mock(),
                    Mock(),
                    Mock(),
                    vehicle_id="vehicle_2",
                    shared_output_directory=root,
                )
            self.assertEqual(service.camera_status.image("left"), b"secondary pixels")

    def test_fresh_frames_from_a_previous_controller_generation_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            (runtime / "px4.run").write_text("current-run", encoding="utf-8")
            (runtime / "stereo.json").write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "vehicle_id": "vehicle_1",
                        "controller_run_id": "old-run",
                        "received_wall_time": time.time(),
                        "directory": "perception/run-old",
                        "error": "",
                    }
                ),
                encoding="utf-8",
            )
            status = CameraStatus(root, "vehicle_1", runtime).snapshot()
            self.assertFalse(status["ready"])
            self.assertIn("previous controller generation", status["error"])

    def test_camera_diagnostics_are_shared_but_keep_vehicle_identity(self) -> None:
        from common.logging.records import RecordingCatalog

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "logs").mkdir()
            (root / "logs/camera-streams.jsonl").write_text(
                json.dumps({"vehicle_id": "vehicle_2", "message": "Recording started"}) + "\n",
                encoding="utf-8",
            )
            catalog = RecordingCatalog(root / "vehicles/vehicle_2", root)
            result = catalog.logs("camera-streams", "vehicle_2")
            self.assertEqual(len(result["records"]), 1)
            self.assertEqual(result["records"][0]["vehicle_id"], "vehicle_2")

    def test_stale_frame_is_rejected_and_other_vehicle_data_is_not_exposed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            output = root / "outputs"
            images = output / "perception/run-test"
            images.mkdir(parents=True)
            (images / "latest-left.ppm").write_bytes(b"pixels")
            status = {
                "schema_version": 1,
                "vehicle_id": "vehicle_1",
                "error": "",
                "received_wall_time": time.time(),
                "directory": "perception/run-test",
            }
            path = runtime / "stereo.json"
            path.write_text(json.dumps(status), encoding="utf-8")
            reader = CameraStatus(output, "vehicle_1", runtime)
            self.assertTrue(reader.snapshot()["ready"])
            self.assertEqual(reader.image("left"), b"pixels")
            # Live previews can remain on runtime storage while recordings stay on the host.
            preview = runtime / "stereo-preview"
            preview.mkdir()
            (preview / "latest-left.ppm").write_bytes(b"live pixels")
            status["runtime_previews"] = True
            path.write_text(json.dumps(status), encoding="utf-8")
            self.assertEqual(reader.image("left"), b"live pixels")
            status["received_wall_time"] = time.time() - 10
            path.write_text(json.dumps(status), encoding="utf-8")
            with self.assertRaises(RuntimeError):
                reader.image("left")
            status.update(received_wall_time=time.time(), directory="vehicles/vehicle_2/perception")
            path.write_text(json.dumps(status), encoding="utf-8")
            self.assertFalse(reader.snapshot()["ready"])

    def test_subscriber_topics_do_not_establish_model_existence(self) -> None:
        from unittest.mock import Mock, patch

        from integrations.px4.launcher import model_exists

        with patch("integrations.px4.launcher.subprocess.run") as query:
            query.return_value = Mock(returncode=0, stdout='model {\n name: "ground_plane"\n}')
            self.assertFalse(model_exists("x500_0"))
            query.return_value = Mock(returncode=0, stdout='model {\n name: "x500_0"\n}')
            self.assertTrue(model_exists("x500_0"))
            query.return_value = Mock(returncode=1, stdout='name: "x500_0"')
            self.assertFalse(model_exists("x500_0"))
