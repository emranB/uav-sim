"""Check saved mission formats, protocol conversion and validation boundaries."""

import json
import math
import unittest
from pathlib import Path

from common.configuration.missions import parse_plan


class MissionTests(unittest.TestCase):
    def plan(self, **changes: object) -> dict:
        item = {
            "type": "SimpleItem",
            "frame": 3,
            "command": 16,
            "params": [0, 0, 0, None, 47.4, 8.5, 10],
        }
        item.update(changes)
        return {"fileType": "Plan", "version": 1, "mission": {"items": [item]}}

    def test_unspecified_yaw_is_preserved(self) -> None:
        mission = parse_plan(self.plan())
        self.assertTrue(math.isnan(mission[0].parameters[3]))
        self.assertEqual(mission[0].parameters[6], 10)

    def test_qgc_created_route_remains_loadable(self) -> None:
        plan_path = Path(__file__).resolve().parents[3] / "configs/missions/qgc-ui-validation.plan"
        plan = json.loads(plan_path.read_text())
        self.assertEqual([item.command for item in parse_plan(plan)], [22, 16, 16, 20])

    def test_unsupported_commands_and_complex_items_are_rejected(self) -> None:
        for changes in ({"command": 999}, {"type": "ComplexItem"}, {"frame": 0}):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    parse_plan(self.plan(**changes))

    def test_invalid_geographic_values_are_rejected(self) -> None:
        for parameters in (
            [0, 0, 0, None, 91, 8, 10],
            [0, 0, 0, None, 47, 8, -2],
            [0, 0, 0, None, math.nan, 8, 10],
        ):
            with self.subTest(parameters=parameters):
                with self.assertRaises(ValueError):
                    parse_plan(self.plan(params=parameters))

    def test_non_integer_protocol_identifiers_are_rejected(self) -> None:
        for changes in ({"command": 22.0}, {"frame": 3.0}, {"command": [16]}):
            with self.subTest(changes=changes):
                with self.assertRaisesRegex(ValueError, "must be integers"):
                    parse_plan(self.plan(**changes))

    def test_manual_progression_is_rejected_instead_of_silently_changed(self) -> None:
        with self.assertRaisesRegex(ValueError, "automatic progression"):
            parse_plan(self.plan(autoContinue=False))

    def test_non_finite_yaw_and_oversized_numbers_are_rejected(self) -> None:
        for index, value in ((3, math.inf), (3, math.nan), (6, 10**400)):
            parameters = [0, 0, 0, None, 47.4, 8.5, 10]
            parameters[index] = value
            with self.subTest(index=index, value=value):
                with self.assertRaises(ValueError):
                    parse_plan(self.plan(params=parameters))
