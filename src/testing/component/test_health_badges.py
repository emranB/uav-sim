"""Persistent fleet health and focused sensor details use measured identity-scoped status."""

import unittest

from PySide6.QtWidgets import QApplication, QLabel

from control_panel.health import VehicleHealthBadges
from control_panel.sensors.panel import SensorsPanel
from testing.component.test_panels import RecordedClient


class HealthBadgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_health_identity_staleness_and_distinct_absence_states(self) -> None:
        badges = VehicleHealthBadges()
        self.addCleanup(badges.close)
        rows = [
            {
                "vehicle_id": identity,
                "label": name,
                "run_id": "run-1",
                "health": [
                    {
                        "vehicle_id": identity,
                        "controller_generation": "run-1",
                        "system": "GPS fix",
                        "state": state,
                        "reason": "Measured native fix",
                        "evidence_age_seconds": 0.1,
                    }
                ],
            }
            for identity, name, state in (
                ("atlas", "Atlas", "unavailable"),
                ("boreal", "Boreal", "healthy"),
            )
        ]
        badges.present(rows, 0.1)
        atlas = badges.findChild(QLabel, "health_atlas_GPS_fix")
        boreal = badges.findChild(QLabel, "health_boreal_GPS_fix")
        self.assertIn("unavailable", atlas.toolTip())
        self.assertIn("healthy", boreal.toolTip())
        rows[0]["run_id"] = "run-2"
        badges.present(rows, 0.1)
        self.assertIn("unknown", atlas.toolTip())
        self.assertIn("healthy", boreal.toolTip())
        badges.present(rows, 2)
        self.assertIn("unknown", boreal.toolTip())

    def test_confirmed_no_fix_is_red_and_stale_or_absent_report_is_unknown(self) -> None:
        client = RecordedClient()
        panel = SensorsPanel(client)
        self.addCleanup(panel.close)
        snapshot = {
            "vehicle_id": "atlas",
            "run_id": "new",
            "connected": True,
            "health": [
                {
                    "vehicle_id": "atlas",
                    "controller_generation": "new",
                    "system": "GPS fix",
                    "state": "unavailable",
                    "reason": "Native GPS reports no fix",
                    "evidence_age_seconds": 0.1,
                }
            ],
        }
        panel.update_health(snapshot)
        self.assertEqual(panel.table.item(0, 1).text(), "No fix")
        self.assertEqual(panel.table.item(0, 1).foreground().color().name(), "#ff7b85")
        snapshot["run_id"] = "newer"
        panel.update_health(snapshot)
        self.assertEqual(panel.table.item(0, 1).text(), "Stale / unknown")
        self.assertNotEqual(panel.table.item(0, 1).foreground().color().name(), "#ff7b85")
