"""Acknowledgement and retry behavior with a recorded Gazebo transport."""

import unittest
from unittest.mock import Mock

from gz.msgs10.boolean_pb2 import Boolean

from gazebo.control_worker import execute_control


class WorldControlTests(unittest.TestCase):
    def test_pause_retries_a_missing_acknowledgement(self) -> None:
        node = Mock()
        node.request.side_effect = [(False, Boolean()), (True, Boolean(data=True))]
        execute_control(node, "pause")
        self.assertEqual(node.request.call_count, 2)

    def test_explicit_rejection_is_reported_without_success(self) -> None:
        node = Mock()
        node.request.return_value = (True, Boolean(data=False))
        with self.assertRaisesRegex(RuntimeError, "rejected"):
            execute_control(node, "pause")
