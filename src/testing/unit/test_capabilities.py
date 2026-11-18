"""Unavailable capabilities are observable failures, never successful placeholders."""

import unittest
from unittest.mock import Mock

from common.interfaces.capabilities import Capability, CapabilityCatalog, CapabilityUnavailable
from common.simulation_manager.fleet import FleetOperations
from common.simulation_manager.perception import vehicle_perception


class CapabilityTests(unittest.TestCase):
    def test_missing_perception_target_is_rejected_before_dispatch(self) -> None:
        services = {key: Mock() for key in ("vehicle_1", "vehicle_2")}
        with self.assertRaisesRegex(ValueError, "explicit vehicle targets"):
            FleetOperations(services).execute("perception/check", {"capability_id": "visual_slam"})
        for service in services.values():
            service.execute.assert_not_called()

    def test_missing_feature_reports_identity_and_machine_readable_failure(self) -> None:
        catalog = vehicle_perception()
        with self.assertRaises(CapabilityUnavailable) as caught:
            catalog.require("visual_slam", "vehicle_2")
        result = caught.exception.result()
        self.assertFalse(result["accepted"])
        self.assertEqual(result["code"], "CAPABILITY_UNAVAILABLE")
        self.assertEqual(result["vehicle_id"], "vehicle_2")
        self.assertEqual(result["capability_id"], "visual_slam")
        self.assertIn("unavailable", result["error"])

    def test_unknown_identifier_is_not_treated_as_an_implemented_feature(self) -> None:
        for key in (None, [], "missing"):
            with self.subTest(key=key), self.assertRaises(ValueError):
                vehicle_perception().require(key, "vehicle_1")

    def test_duplicate_declarations_are_rejected(self) -> None:
        item = Capability("images", "Images", False, "No image stream")
        with self.assertRaises(ValueError):
            CapabilityCatalog((item, item))

    def test_collection_retains_individual_failure_details(self) -> None:
        services = {key: Mock() for key in ("vehicle_1", "vehicle_2")}
        for key, service in services.items():
            try:
                vehicle_perception().require("visual_slam", key)
            except CapabilityUnavailable as error:
                service.execute.side_effect = error
        result = FleetOperations(services).execute(
            "perception/check", {"target_ids": list(services), "capability_id": "visual_slam"}
        )
        self.assertFalse(result["accepted"])
        for key, item in result["results"].items():
            self.assertEqual(item["vehicle_id"], key)
            self.assertEqual(item["code"], "CAPABILITY_UNAVAILABLE")
