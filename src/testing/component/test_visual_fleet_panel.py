"""Fleet comparison keeps requested GPS state distinct from actual aiding."""

import time
import unittest

from PySide6.QtWidgets import QApplication, QLabel

from control_panel.navigation.fleet import FleetComparisonPanel
from testing.component.test_panels import RecordedClient


class VisualFleetPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_health_badges_use_identity_generation_and_display_freshness(self) -> None:
        panel = FleetComparisonPanel(RecordedClient())
        self.addCleanup(panel.close)
        self.addCleanup(panel.timer.stop)
        rows = [
            {
                "vehicle_id": identity,
                "label": name,
                "controller_generation": "run-1",
                "display_launch_enu_m": [offset, 0, 0],
                "armed": False,
                "navigation": {},
                "health": [
                    {
                        "vehicle_id": identity,
                        "controller_generation": "run-1",
                        "system": "GPS fix",
                        "state": state,
                        "reason": "Measured fix",
                        "evidence_age_seconds": 0.1,
                        "requested_state": "enabled",
                    }
                ],
            }
            for identity, name, offset, state in (
                ("atlas", "Atlas", 0, "unavailable"),
                ("boreal", "Boreal", 6, "healthy"),
            )
        ]
        panel.receive("navigation/fleet", {"items": rows})
        atlas = panel.findChild(QLabel, "health_atlas_GPS_fix")
        boreal = panel.findChild(QLabel, "health_boreal_GPS_fix")
        self.assertIn("unavailable", atlas.toolTip())
        self.assertIn("healthy", boreal.toolTip())
        self.assertIn("atlas", atlas.toolTip())
        rows[0]["controller_generation"] = "run-2"
        panel.receive("navigation/fleet", {"items": rows})
        self.assertIn("unknown", atlas.toolTip())
        self.assertIn("healthy", boreal.toolTip())
        panel.last_received_monotonic = time.monotonic() - 2
        panel.render({"items": rows})
        self.assertIn("unknown", boreal.toolTip())

    def test_transition_flags_paths_and_stale_freeze_are_visible(self) -> None:
        panel = FleetComparisonPanel(RecordedClient())
        panel.resize(600, 650)
        now = time.monotonic()
        rows = []
        for identity, label, offset in (("vehicle_1", "Atlas", 0), ("vehicle_2", "Boreal", 6)):
            rows.append(
                {
                    "vehicle_id": identity,
                    "label": label,
                    "display_launch_enu_m": [offset, 0, 0],
                    "armed": True,
                    "navigation": {
                        "state": "navigating",
                        "current_waypoint": "outbound-1-hold",
                        "saved_mission": {"name": "comparison", "goal_offset_enu_m": [2.5, 1.5]},
                        "visual_localization": {
                            "pose": {
                                "healthy": True,
                                "monotonic": now,
                                "controller_run_id": identity,
                                "map_generation": 1,
                                "horizontal_ned_m": [1, 2],
                            },
                            "aiding": {
                                "monotonic": now,
                                "gps_requested_enabled": False,
                                "gps_fix_type": 0,
                                "satellites_used": 0,
                                "gnss_xy_active": True,
                                "vision_xy_active": True,
                            },
                        },
                    },
                }
            )
        panel.receive("navigation/fleet", {"items": rows, "scene_id": "visual-odometry-probe"})
        self.assertIn("GPS LOSS TRANSITION", panel.sources.text())
        self.assertIn("GPS requested off", panel.sources.text())
        self.assertIn("Atlas", panel.sources.text())
        self.assertIn("Boreal", panel.sources.text())
        self.assertEqual(panel.paths.paths["vehicle_2"][-1], (8, 1))
        panel.show()
        self.application.processEvents()
        self.assertFalse(panel.grab().isNull())
        for row in rows:
            row["navigation"]["visual_localization"]["pose"]["monotonic"] = now - 2
        panel.receive("navigation/fleet", {"items": rows})
        self.assertIn("STALE / FROZEN", panel.sources.text())
        self.assertEqual(len(panel.paths.paths["vehicle_2"]), 1)
        panel.timer.stop()
        panel.close()
