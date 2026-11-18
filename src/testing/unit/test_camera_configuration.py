"""Acquisition calibration and rendered stereo geometry must agree."""

import os
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

from common.configuration.cameras import StereoConfiguration
from gazebo.stereo_model import prepare_stereo_model


class CameraConfigurationTests(unittest.TestCase):
    def test_rendered_baseline_and_fov_match_acquisition_configuration(self) -> None:
        with (
            tempfile.TemporaryDirectory() as name,
            patch.dict(
                os.environ,
                {
                    "UAV_CAMERA_BASELINE_M": "0.4",
                    "UAV_CAMERA_FOV_RAD": "1.4",
                    "UAV_CAMERA_PITCH_RAD": "1.1",
                    "UAV_CAMERA_HEIGHT_M": "0.5",
                },
            ),
        ):
            root = Path(name)
            source = root / "source.sdf"
            source.write_text(
                '<sdf version="1.9"><model name="x500"><link name="base_link"/></model></sdf>'
            )
            model = ET.parse(prepare_stereo_model(source, root)).find("model")
            configuration = StereoConfiguration.configured()
            poses = [
                list(map(float, link.findtext("pose").split()))
                for link in model.findall("link")
                if link.get("name", "").startswith("stereo_")
            ]
            self.assertAlmostEqual(poses[0][1] - poses[1][1], configuration.baseline_m)
            self.assertEqual(poses[0][2], configuration.mount_height_m)
            for camera in model.findall("link/sensor/camera"):
                self.assertEqual(
                    float(camera.findtext("horizontal_fov")), configuration.horizontal_fov_rad
                )

    def test_invalid_optics_are_rejected_before_model_startup(self) -> None:
        for key, value in (
            ("UAV_CAMERA_BASELINE_M", "nan"),
            ("UAV_CAMERA_FOV_RAD", "inf"),
            ("UAV_CAMERA_PITCH_RAD", "-1"),
        ):
            with self.subTest(key=key), patch.dict(os.environ, {key: value}):
                with self.assertRaises(ValueError):
                    StereoConfiguration.configured()
