"""Panel domain behavior with temporary records and an acknowledged fake vehicle."""

import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

from common.configuration.environment import EnvironmentControls
from common.configuration.mission_store import MissionStore
from common.configuration.preferences import DesktopPreferences
from common.configuration.world import WorldLocation
from common.logging.records import RecordingCatalog
from common.simulation_manager.workspace import WorkspaceFeatures
from gazebo.world import prepare_world


class RecordedVehicle:
    def __init__(self) -> None:
        self.link = {"latency_ms": 0, "loss_percent": 0, "seed": 0}
        self.reject = False

    def snapshot(self) -> dict:
        return {"communication": self.link}

    def configure_link(self, latency_ms: int, loss_percent: int, seed: int) -> None:
        self.link = {"latency_ms": latency_ms, "loss_percent": loss_percent, "seed": seed}

    def set_gnss_satellites(self, satellites: int) -> None:
        if self.reject:
            raise RuntimeError("Parameter was rejected")


class WorkspaceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_firmware_identity_distinguishes_patched_and_unpatched_builds(self) -> None:
        (self.directory / "source-revision").write_text("upstream-revision\n")
        with patch("common.simulation_manager.workspace.Path", return_value=self.directory):
            self.assertEqual(WorkspaceFeatures.versions(), {"source-revision": "upstream-revision"})
            (self.directory / "firmware-patches.sha256").write_text("patch-digest  timer.patch\n")
            self.assertEqual(
                WorkspaceFeatures.versions(),
                {
                    "source-revision": "upstream-revision",
                    "firmware-patches.sha256": "patch-digest  timer.patch",
                },
            )

    def mission(self, altitude_m: float = 8) -> dict:
        return {
            "fileType": "Plan",
            "version": 1,
            "mission": {
                "items": [
                    {
                        "type": "SimpleItem",
                        "command": 16,
                        "frame": 3,
                        "params": [0, 0, 0, None, 44.6488, -63.5752, altitude_m],
                    }
                ]
            },
        }

    def test_rejected_sensor_change_does_not_change_applied_state(self) -> None:
        vehicle = RecordedVehicle()
        controls = EnvironmentControls(vehicle, self.directory / "logs")
        vehicle.reject = True
        with self.assertRaises(RuntimeError):
            controls.execute("sensors/gnss", {"enabled": False})
        self.assertTrue(controls.snapshot()["gnss_enabled"])

    def test_sensor_state_survives_backend_restart_and_clear_restores_defaults(self) -> None:
        vehicle = RecordedVehicle()
        controls = EnvironmentControls(vehicle, self.directory / "logs")
        controls.execute("sensors/gnss", {"enabled": False})
        controls = EnvironmentControls(vehicle, self.directory / "logs")
        self.assertFalse(controls.snapshot()["gnss_enabled"])
        controls.clear()
        self.assertTrue(controls.snapshot()["gnss_enabled"])
        self.assertEqual(controls.snapshot()["link"]["loss_percent"], 0)

    def test_invalid_sensor_values_are_rejected(self) -> None:
        controls = EnvironmentControls(RecordedVehicle(), self.directory / "logs")
        for operation, body in (
            ("sensors/gnss", {"enabled": 1}),
            ("interference/gnss", {"satellites": -1}),
        ):
            with self.assertRaises(ValueError):
                controls.execute(operation, body)

    def test_mission_names_cannot_escape_storage(self) -> None:
        store = MissionStore(self.directory)
        for name in ("../outside", "", "/absolute", "one/two"):
            with self.assertRaises(ValueError):
                store.save(name, self.mission())

    def test_mission_replacement_archives_previous_version(self) -> None:
        store = MissionStore(self.directory)
        store.save("Halifax", self.mission(8))
        store.save("Halifax", self.mission(12))
        self.assertEqual(store.load("Halifax")["mission"]["items"][0]["params"][6], 12)
        self.assertEqual(len(list((self.directory / "missions/named").glob("*.archived-*"))), 1)
        store.archive("Halifax")
        self.assertEqual(store.list(), [])

    def test_repeated_archive_timestamp_preserves_every_mission_version(self) -> None:
        store = MissionStore(self.directory)
        with patch("common.configuration.mission_store.time.time_ns", return_value=123):
            store.save("Halifax", self.mission(8))
            store.save("Halifax", self.mission(12))
            store.archive("Halifax")
        archives = list((self.directory / "missions/named").glob("*.archived-*"))
        self.assertEqual(len(archives), 2)
        altitudes = {
            json.loads(path.read_text(encoding="utf-8"))["mission"]["items"][0]["params"][6]
            for path in archives
        }
        self.assertEqual(altitudes, {8, 12})

    def test_preferences_validate_before_replacing_saved_values(self) -> None:
        preferences = DesktopPreferences(self.directory / "preferences.json")
        saved = preferences.save({"poll_interval_ms": 750})
        with self.assertRaises(ValueError):
            preferences.save({"poll_interval_ms": 0})
        self.assertEqual(preferences.load(), saved)

    def test_console_search_returns_matching_records(self) -> None:
        folder = self.directory / "logs"
        folder.mkdir()
        (folder / "px4.jsonl").write_text(
            json.dumps({"message": "Ready for takeoff"})
            + "\n"
            + json.dumps({"message": "GPS unavailable"})
            + "\n",
            encoding="utf-8",
        )
        records = RecordingCatalog(self.directory).logs("px4", "GPS")["records"]
        self.assertEqual([record["message"] for record in records], ["GPS unavailable"])

    def test_world_generation_changes_geographic_origin_and_preserves_physics(self) -> None:
        template = self.directory / "template.sdf"
        template.write_text(
            '<sdf><world name="default"><gravity>0 0 -9.8</gravity>'
            "<spherical_coordinates><latitude_deg>0</latitude_deg>"
            "<longitude_deg>0</longitude_deg><elevation>0</elevation>"
            "</spherical_coordinates></world></sdf>"
        )
        location = WorldLocation("Downtown Halifax", 44.6488, -63.5752, 20)
        result = prepare_world(
            template, self.directory / "world.sdf", location, real_time_factor=0.2
        )
        tree = ET.parse(result)
        self.assertEqual(float(tree.findtext("world/spherical_coordinates/latitude_deg")), 44.6488)
        self.assertEqual(tree.findtext("world/gravity"), "0 0 -9.8")
        self.assertEqual(tree.findtext("world/physics/real_time_factor"), "0.2")
        self.assertIsNotNone(tree.find("world/model[@name='flight_reference']"))
        self.assertIsNotNone(tree.find("world/model[@name='reference_grid']"))
        self.assertEqual(tree.findall("world/model[@name='flight_reference']//collision"), [])

    def test_recording_comparison_reports_observed_position_statistics(self) -> None:
        folder = self.directory / "telemetry"
        folder.mkdir()
        rows = []
        for seconds, altitude in ((0, 0), (10, 8000)):
            rows.append(
                json.dumps(
                    {
                        "timestamp_utc": f"2026-11-07T12:00:{seconds:02d}+00:00",
                        "message": {
                            "mavpackettype": "GLOBAL_POSITION_INT",
                            "lat": 446488000,
                            "lon": -635752000,
                            "relative_alt": altitude,
                            "vx": 120,
                            "vy": 0,
                        },
                    }
                )
            )
        (folder / "telemetry-1.jsonl").write_text("\n".join(rows) + "\n", encoding="utf-8")
        catalog = RecordingCatalog(self.directory)
        result = catalog.compare("telemetry-1.jsonl", "telemetry-1.jsonl")
        self.assertEqual(result["comparison"]["left"]["maximum_altitude_m"], 8)
        self.assertEqual(result["comparison"]["left"]["duration_utc_seconds"], 10)
        self.assertEqual(result["comparison"]["delta"]["maximum_altitude_m"], 0)
        with self.assertRaises(ValueError):
            catalog.statistics("../telemetry-1.jsonl")
