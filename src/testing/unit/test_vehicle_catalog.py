"""Catalog lifecycle preserves records and rejects invalid removal before mutation."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from common.configuration.fleet import load_fleet
from common.vehicle_manager.catalog import VehicleCatalog


class VehicleCatalogTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.services = {"vehicle_1": Mock(), "vehicle_2": Mock()}
        for service in self.services.values():
            service.vehicle.snapshot.return_value = {"armed": False}
        self.create, self.retire, self.rename = Mock(), Mock(), Mock()
        self.catalog = VehicleCatalog(
            Path(self.directory.name), self.services, self.create, self.retire, self.rename
        )

    def test_add_allocates_unique_endpoints_and_remove_retires_identity(self) -> None:
        created = self.catalog.execute("vehicles/add", {"label": "Cedar", "model": "x500"})
        identifier = created["vehicle_id"]
        self.create.return_value.vehicle.snapshot.return_value = {"armed": False}
        self.catalog.execute("vehicles/remove", {"vehicle_id": identifier})
        definitions = load_fleet(self.catalog.path)
        self.assertFalse(definitions[-1].enabled)
        self.assertNotIn(identifier, self.services)
        self.retire.assert_called_once()
        self.assertTrue((Path(self.directory.name) / "configuration-history").is_dir())
        replacement = self.catalog.execute("vehicles/add", {"label": "Dawn"})
        replacement_definition = load_fleet(self.catalog.path)[-1]
        self.assertNotEqual(replacement["vehicle_id"], identifier)
        self.assertEqual(replacement_definition.instance, 2)

    def test_armed_vehicle_is_never_removed(self) -> None:
        self.services["vehicle_2"].vehicle.snapshot.return_value = {"armed": True}
        with self.assertRaisesRegex(RuntimeError, "disarm"):
            self.catalog.execute("vehicles/remove", {"vehicle_id": "vehicle_2"})
        self.retire.assert_not_called()
        self.assertFalse(self.catalog.path.exists())

    def test_failed_retirement_retains_entry_for_cleanup_retry(self) -> None:
        self.retire.side_effect = TimeoutError("Still stopping")
        with self.assertRaises(TimeoutError):
            self.catalog.execute("vehicles/remove", {"vehicle_id": "vehicle_2"})
        self.assertIn("vehicle_2", self.services)
        self.assertFalse(load_fleet(self.catalog.path)[1].enabled)

    def test_failed_persistence_releases_the_unregistered_vehicle(self) -> None:
        from unittest.mock import patch

        with patch.object(self.catalog, "_persist", side_effect=OSError("Disk full")):
            with self.assertRaises(OSError):
                self.catalog.execute("vehicles/add", {"label": "Cedar"})
        self.create.return_value.vehicle.close.assert_called_once()
        self.assertEqual(len(self.services), 2)
