"""Headless UI behavior checks; no simulator, display, or network is required."""

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from PySide6.QtCore import QObject, QProcess, QSettings, Signal
from PySide6.QtWidgets import (
    QApplication,
    QPushButton,
    QStackedWidget,
    QTabWidget,
    QWidget,
)

from common.configuration.preferences import DEFAULT_PREFERENCES
from control_panel.logs.panel import LogsPanel
from control_panel.panel import default_registry
from control_panel.telemetry.panel import TelemetryPanel
from desktop.viewer import ViewerPanel
from desktop.window import MainWindow
from qgroundcontrol.launcher import prepare_preferences, remove_stale_instance_locks


class RecordedClient(QObject):
    telemetry_received = Signal(dict)
    error_received = Signal(str)
    operation_finished = Signal(str)
    mission_received = Signal(dict)
    workspace_reset = Signal()
    resource_received = Signal(str, dict)
    operation_result = Signal(str, dict)
    operation_failed = Signal(str, str)
    preferences_changed = Signal(dict)
    selection_changed = Signal(str)
    camera_image_received = Signal(str, bytes)
    camera_image_failed = Signal(str, str)
    target_operation_result = Signal(str, dict)
    replay_operation_result = Signal(str, dict)
    restore_operation_result = Signal(str, dict)
    restore_completed = Signal(dict)

    def set_action_targets(self, vehicle_ids: tuple[str, ...]) -> None:
        self.target_ids = list(vehicle_ids)

    def __init__(self) -> None:
        super().__init__()
        self.vehicle_id = "vehicle_1"
        self.preferences = dict(DEFAULT_PREFERENCES)
        self.requests = []

    def fetch(self, resource: str) -> None:
        self.requests.append((resource, None))

    def execute(self, operation: str, body: dict | None = None) -> None:
        self.requests.append((operation, body))

    def execute_targets(self, operation: str, target_ids: list[str], request_id: str) -> None:
        self.requests.append(
            (operation, {"target_ids": list(target_ids), "request_id": request_id})
        )

    def execute_replay(self, operation: str, body: dict, request_id: str) -> None:
        self.requests.append((operation, {**body, "request_id": request_id}))

    def execute_restore(self, request_id: str, *, cancel: bool = False) -> None:
        operation = "restore/cancel" if cancel else "restore/start"
        self.requests.append((operation, {"request_id": request_id}))

    def save_preferences(self, values: dict) -> None:
        self.preferences = values
        self.preferences_changed.emit(values)

    def select_vehicle(self, vehicle_id: str, apply_to_all: bool = False) -> None:
        self.vehicle_id = vehicle_id
        self.selection_changed.emit(vehicle_id)

    def close(self) -> None:
        pass

    def fetch_camera_image(self, side: str) -> None:
        self.requests.append((f"camera/{side}", None))


class PanelTests(unittest.TestCase):
    def test_closing_workspace_stops_both_attachment_timers(self) -> None:
        with patch("desktop.window.SimulationClient", return_value=RecordedClient()):
            window = MainWindow()
            window.show()
            self.application.processEvents()
            panels = window.findChildren(ViewerPanel)
            self.assertEqual(len(panels), 2)
            self.assertTrue(all(panel.timer.isActive() for panel in panels))
            window.close()
            self.assertTrue(all(not panel.timer.isActive() for panel in panels))

    def test_camera_toolbar_stays_compact_and_follow_does_not_reapply_a_preset(self) -> None:
        from gazebo.panel import GazeboPanel

        client = RecordedClient()
        panel = GazeboPanel(client)
        panel.resize(440, 400)
        panel.show()
        self.application.processEvents()
        navigation = panel.findChild(QWidget, "cameraToolbar")
        self.assertEqual(navigation.height(), 38)
        self.assertLessEqual(panel.camera_status.height(), 20)
        panel.toggle_follow(True)
        self.assertEqual(client.requests[-1], ("viewer/follow", {"follow": True}))
        panel.close()

    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_every_registered_panel_constructs_with_recorded_dependencies(self) -> None:
        registry = default_registry(RecordedClient())
        self.assertEqual(len(registry.all()), 7)
        for spec in registry.all():
            with self.subTest(panel=spec.title):
                panel = spec.factory()
                self.assertIsInstance(panel.widget(), QWidget)
                panel.close()

    def test_gnss_edit_is_sent_without_claiming_it_was_applied(self) -> None:
        from control_panel.sensors.panel import SensorsPanel

        client = RecordedClient()
        panel = SensorsPanel(client)
        panel.enabled.setChecked(False)
        next(
            button
            for button in panel.findChildren(QPushButton)
            if button.text() == "Apply GNSS input"
        ).click()
        self.assertEqual(client.requests[-1], ("sensors/gnss", {"enabled": False}))
        self.assertIn("Waiting", panel.applied.text())
        panel.close()

    def test_workspace_prioritizes_world_and_moves_controls_into_a_tab(self) -> None:
        from desktop.layout import build_workspace

        with (
            patch("desktop.layout.QGroundControlPanel", return_value=QWidget()) as qgc,
            patch("desktop.layout.GazeboPanel", return_value=QWidget()) as gazebo,
        ):
            workspace = build_workspace(default_registry(RecordedClient()))
        self.assertIn(qgc.return_value, workspace.widget(0).findChildren(QWidget))
        right = workspace.widget(1)
        self.assertIsInstance(right, QTabWidget)
        self.assertIs(right.widget(0), gazebo.return_value)
        self.assertIsNotNone(right.widget(1).findChild(QStackedWidget))
        self.assertEqual(right.currentIndex(), 0)
        workspace.close()

    def test_all_control_modules_are_reachable_without_scrolling_the_tab_bar(self) -> None:
        from desktop.layout import ControlDeck

        deck = ControlDeck(default_registry(RecordedClient()))
        for index in range(7):
            deck.buttons.button(index).click()
            self.assertEqual(deck.pages.currentIndex(), index)
        deck.close()

    def test_unrelated_operation_error_does_not_complete_a_panel_request(self) -> None:
        from control_panel.sensors.panel import SensorsPanel

        client = RecordedClient()
        panel = SensorsPanel(client)
        panel.send("sensors/gnss", {"enabled": False})
        client.operation_failed.emit("vehicles/read", "Not connected")
        self.assertEqual(panel.pending, "sensors/gnss")
        client.operation_result.emit("sensors/gnss", {"accepted": True})
        self.assertIsNone(panel.pending)
        self.assertIn("acknowledged", panel.status.text())
        panel.close()

    def test_panel_feedback_blocks_duplicate_actions_and_recovers_after_failure(self) -> None:
        from control_panel.sensors.panel import SensorsPanel

        client = RecordedClient()
        panel = SensorsPanel(client)
        buttons = [button for button in panel.findChildren(QPushButton) if button.isEnabled()]
        panel.send("sensors/gnss", {"enabled": False})
        self.assertTrue(all(not button.isEnabled() for button in buttons))
        panel.send("sensors/gnss", {"enabled": True})
        self.assertEqual(sum(route == "sensors/gnss" for route, _ in client.requests), 1)
        client.operation_failed.emit("sensors/gnss", "Vehicle did not acknowledge")
        self.assertTrue(all(button.isEnabled() for button in buttons))
        self.assertEqual(panel.status.property("state"), "error")
        self.assertIn("did not acknowledge", panel.status.text())
        panel.send("sensors/gnss", {"enabled": True})
        client.operation_result.emit("sensors/gnss", {"accepted": True})
        self.assertEqual(panel.status.property("state"), "success")
        panel.close()

    def test_unavailable_perception_is_visible_and_check_failure_is_reported(self) -> None:
        from common.simulation_manager.perception import vehicle_perception
        from control_panel.sensors.panel import SensorsPanel

        client = RecordedClient()
        panel = SensorsPanel(client)
        client.resource_received.emit("perception", vehicle_perception().snapshot("vehicle_2"))
        inspector = panel.perception
        inspector.features.setCurrentIndex(inspector.features.findData("visual_slam"))
        self.assertFalse(inspector.available.isChecked())
        self.assertFalse(inspector.available.isEnabled())
        self.assertIn("No live per-vehicle mapping service", inspector.reason.text())
        inspector.check.click()
        self.assertEqual(
            client.requests[-1], ("perception/check", {"capability_id": "visual_slam"})
        )
        client.operation_failed.emit("perception/check", "Visual SLAM is unavailable")
        self.assertEqual(panel.status.property("state"), "error")
        self.assertIn("unavailable", panel.status.text())
        client.selection_changed.emit("vehicle_1")
        self.assertEqual(inspector.features.count(), 0)
        self.assertFalse(inspector.check.isEnabled())
        panel.close()

    def test_republished_native_id_is_reattached_after_viewer_restart(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "Gazebo.window"
            path.write_text("123")
            with (
                patch("desktop.viewer.QWindow.fromWinId", return_value=Mock()) as attach,
                patch(
                    "desktop.viewer.QWidget.createWindowContainer",
                    side_effect=lambda *args: QWidget(),
                ),
            ):
                panel = ViewerPanel("Gazebo", path)
                panel.timer.stop()
                panel.refresh()
                panel.refresh()
                self.assertEqual(attach.call_count, 1)
                revision = path.stat().st_mtime_ns + 1_000_000_000
                os.utime(path, ns=(revision, revision))
                panel.refresh()
                panel.refresh()
                self.assertEqual(attach.call_count, 2)
                panel.detach()
                panel.close()

    def test_tiny_saved_qgc_window_is_repaired_without_resetting_preferences(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "QGroundControl.ini"
            preferences = QSettings(str(path), QSettings.IniFormat)
            preferences.setValue("MainWindowState/width", 10)
            preferences.setValue("MainWindowState/height", 10)
            preferences.setValue("FlightMap/mapProvider", "test-provider")
            preferences.sync()
            prepare_preferences(path)
            preferences.sync()
            self.assertGreaterEqual(preferences.value("MainWindowState/width", type=int), 800)
            self.assertGreaterEqual(preferences.value("MainWindowState/height", type=int), 600)
            self.assertEqual(preferences.value("FlightMap/mapProvider"), "test-provider")

    def test_unwritable_qgc_preferences_fail_before_launch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            blocked = Path(directory) / "blocked"
            blocked.write_text("A file cannot be used as the settings directory")
            with self.assertRaisesRegex(OSError, "preferences are not writable"):
                prepare_preferences(blocked / "QGroundControl.ini")

    def test_abandoned_instance_lock_is_removed_before_pid_reuse(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "qgc-test.lock"
            path.write_text("123\nQGroundControl\ncontainer\n")
            with patch("qgroundcontrol.launcher.os.kill", return_value=None):
                remove_stale_instance_locks(Path(directory))
            self.assertTrue(path.exists())
            with patch("qgroundcontrol.launcher.os.kill", side_effect=ProcessLookupError):
                remove_stale_instance_locks(Path(directory))
            self.assertFalse(path.exists())

    def test_failed_native_repair_remains_visible_until_recovery(self) -> None:
        panel = ViewerPanel("Gazebo", Path("Gazebo.window"))
        panel.timer.stop()
        with self.assertLogs(level="ERROR"):
            panel.finish_parent_repair(1, QProcess.ExitStatus.CrashExit)
        self.assertFalse(panel.message.isHidden())
        self.assertIn("Could not attach Gazebo", panel.message.text())
        panel.finish_parent_repair(0, QProcess.ExitStatus.NormalExit)
        self.assertTrue(panel.message.isHidden())
        panel.close()

    def test_disconnect_clears_old_position(self) -> None:
        panel = TelemetryPanel(RecordedClient())
        panel.update_telemetry(
            {
                "connected": True,
                "vehicle_id": "vehicle_1",
                "armed": False,
                "latitude_deg": 47.4,
                "longitude_deg": 8.5,
                "relative_altitude_m": 8,
            }
        )
        self.assertIn("47.4000000", panel.values.text())
        panel.update_telemetry({"connected": False})
        self.assertNotIn("47.4000000", panel.values.text())
        self.assertIn("disconnected", panel.values.text())
        panel.close()

    def test_px4_status_is_not_duplicated_on_each_poll(self) -> None:
        panel = LogsPanel(RecordedClient())
        message = {"messages": {"STATUSTEXT": {"text": "Ready for takeoff"}}}
        panel.update_status(message)
        panel.update_status(message)
        self.assertEqual(panel.toPlainText().count("Ready for takeoff"), 1)
        panel.close()

    def test_saved_log_capacity_is_applied_when_panel_starts(self) -> None:
        client = RecordedClient()
        client.preferences["log_rows"] = 250
        panel = LogsPanel(client)
        self.assertEqual(panel.output.maximumBlockCount(), 250)
        panel.close()

    def test_recovery_controls_remain_available_without_telemetry(self) -> None:
        with (
            patch("desktop.window.SimulationClient", RecordedClient),
            patch(
                "desktop.window.build_workspace",
                return_value=QWidget(),
            ),
        ):
            window = MainWindow()
        window.update_connection({"connected": False, "simulation_paused": True})
        actions = dict(window.actions)
        self.assertTrue(actions["resume"].isEnabled())
        self.assertTrue(actions["reset"].isEnabled())
        self.assertTrue(actions["stop"].isEnabled())
        self.assertNotIn("start", actions)
        window.close()

    def test_save_writes_the_downloaded_vehicle_mission(self) -> None:
        with (
            patch("desktop.window.SimulationClient", RecordedClient),
            patch(
                "desktop.window.build_workspace",
                return_value=QWidget(),
            ),
        ):
            window = MainWindow()
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "saved.plan"
            window.save_destination = destination
            mission = {"fileType": "Plan", "mission": {"items": []}}
            window.write_mission(mission)
            self.assertEqual(json.loads(destination.read_text()), mission)
            self.assertIsNone(window.save_destination)
        window.close()
