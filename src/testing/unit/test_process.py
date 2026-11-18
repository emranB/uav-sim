"""Native display failures must not crash process supervision."""

import subprocess
import unittest
from unittest.mock import patch

from common.lifecycle.process import query_native_window


class NativeWindowQueryTests(unittest.TestCase):
    def test_display_timeout_is_reported_as_an_unavailable_observation(self) -> None:
        with (
            patch(
                "common.lifecycle.process.subprocess.run",
                side_effect=subprocess.TimeoutExpired("xdotool", 5),
            ),
            self.assertLogs(level="WARNING") as logs,
        ):
            result = query_native_window(["xdotool", "search", "--class", "Gazebo"])
        self.assertIsNone(result)
        self.assertIn("timed out", logs.output[0])


class ProcessGenerationTests(unittest.TestCase):
    def test_changed_generation_retires_child_before_requesting_restart(self) -> None:
        import tempfile
        from pathlib import Path
        from unittest.mock import Mock

        from common.lifecycle.process import run_process

        with tempfile.TemporaryDirectory() as directory:
            generation = Path(directory) / "world.epoch"
            generation.write_text("world-one")
            child = Mock(stdout=None, pid=43210)
            child.poll.return_value = None
            child.wait.return_value = 0

            def launch(*args, **kwargs):
                generation.write_text("world-two")
                return child

            with (
                patch("common.lifecycle.process.subprocess.Popen", side_effect=launch),
                patch("common.lifecycle.process.signal.signal"),
                patch("common.lifecycle.process.os.killpg") as stop,
            ):
                result = run_process(["viewer"], generation_file=generation)
            self.assertEqual(result, 75)
            stop.assert_called_once()
            child.wait.assert_called_once_with(timeout=10)

    def test_unchanged_generation_does_not_restart_a_cleanly_exited_child(self) -> None:
        import tempfile
        from pathlib import Path
        from unittest.mock import Mock

        from common.lifecycle.process import run_process

        with tempfile.TemporaryDirectory() as directory:
            generation = Path(directory) / "world.epoch"
            generation.write_text("world-one")
            child = Mock(stdout=None, returncode=0)
            child.poll.return_value = 0
            with (
                patch("common.lifecycle.process.subprocess.Popen", return_value=child),
                patch("common.lifecycle.process.signal.signal"),
                patch("common.lifecycle.process.os.killpg") as stop,
            ):
                result = run_process(["viewer"], generation_file=generation)
            self.assertEqual(result, 0)
            stop.assert_not_called()


class NativeWindowOwnershipTests(unittest.TestCase):
    def test_matching_window_title_and_pid_from_another_machine_is_rejected(self) -> None:
        from common.lifecycle.process import owned_native_window_pid

        answers = [
            subprocess.CompletedProcess([], 0, "222\n"),
            subprocess.CompletedProcess([], 0, 'WM_CLIENT_MACHINE(STRING) = "other"'),
        ]
        with (
            patch("common.lifecycle.process.query_native_window", side_effect=answers),
            patch("common.lifecycle.process.socket.gethostname", return_value="owner"),
            patch("common.lifecycle.process.descendant_process_ids", return_value={222}),
        ):
            self.assertIsNone(owned_native_window_pid("123", 111))

    def test_only_this_child_tree_and_client_machine_can_publish_a_window(self) -> None:
        from common.lifecycle.process import owned_native_window_pid

        for descendants, expected in (({222}, 222), ({333}, None)):
            answers = [
                subprocess.CompletedProcess([], 0, "222\n"),
                subprocess.CompletedProcess([], 0, 'WM_CLIENT_MACHINE(STRING) = "owner"'),
            ]
            with (
                patch("common.lifecycle.process.query_native_window", side_effect=answers),
                patch("common.lifecycle.process.socket.gethostname", return_value="owner"),
                patch("common.lifecycle.process.descendant_process_ids", return_value=descendants),
            ):
                self.assertEqual(owned_native_window_pid("123", 111), expected)
