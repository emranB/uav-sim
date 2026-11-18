"""Verify deployment selection and cross-machine configuration boundaries."""

import unittest

from common.configuration.runtime import RuntimeSettings


class RuntimeSettingsTests(unittest.TestCase):
    def test_defaults_do_not_require_a_gpu_or_display(self) -> None:
        settings = RuntimeSettings.from_environment({})
        self.assertEqual(settings.render_mode, "cpu")
        self.assertTrue(settings.headless)
        environment = settings.rendering_environment({"DISPLAY": ":0"})
        self.assertEqual(environment["LIBGL_ALWAYS_SOFTWARE"], "true")

    def test_gpu_selection_removes_inherited_software_override(self) -> None:
        settings = RuntimeSettings.from_environment({"UAV_RENDER_MODE": "gpu"})
        inherited = {"LIBGL_ALWAYS_SOFTWARE": "true", "DISPLAY": ":1"}
        environment = settings.rendering_environment(inherited)
        self.assertNotIn("LIBGL_ALWAYS_SOFTWARE", environment)
        self.assertEqual(environment["DISPLAY"], ":1")
        self.assertEqual(inherited["LIBGL_ALWAYS_SOFTWARE"], "true")

    def test_headless_and_render_device_are_independent(self) -> None:
        settings = RuntimeSettings.from_environment(
            {
                "UAV_RENDER_MODE": "gpu",
                "UAV_HEADLESS": "true",
                "ROS_DOMAIN_ID": "7",
            }
        )
        self.assertTrue(settings.headless)
        self.assertEqual(settings.ros_domain_id, 7)

    def test_invalid_environment_fails_before_startup(self) -> None:
        for environment in (
            {"UAV_RENDER_MODE": "automatic"},
            {"UAV_HEADLESS": "maybe"},
            {"ROS_DOMAIN_ID": "-1"},
            {"ROS_DOMAIN_ID": "233"},
            {"ROS_DOMAIN_ID": "not-a-number"},
            {"OUTPUT_DIR": " "},
            {"RMW_IMPLEMENTATION": ""},
            {"UAV_HEARTBEAT_TIMEOUT_SECONDS": "nan"},
            {"UAV_HEARTBEAT_TIMEOUT_SECONDS": "31"},
            {"UAV_REAL_TIME_FACTOR": "nan"},
            {"UAV_REAL_TIME_FACTOR": "0"},
            {"UAV_REAL_TIME_FACTOR": "3"},
        ):
            with self.subTest(environment=environment):
                with self.assertRaises(ValueError):
                    RuntimeSettings.from_environment(environment)
