"""Compose the simulation workspace and its independently owned views."""

import json
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMenu,
    QTabWidget,
    QToolButton,
    QVBoxLayout,
)

from common.configuration.preferences import DesktopPreferences
from control_panel.panel import default_registry
from control_panel.presentation import StatusBadge
from control_panel.vehicles.selection import VehicleSelection
from desktop.client import SimulationClient
from desktop.layout import ControlDeck, build_workspace
from desktop.replay_capture import ReplayCaptureRow
from desktop.restore import RestoreControl
from desktop.viewer import ViewerPanel


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Drone Simulation Platform")
        self.resize(1600, 1000)
        self.client = SimulationClient()
        self.connection_status = None
        self.restore_active = False
        header = QFrame()
        header.setObjectName("appHeader")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(12, 6, 12, 6)
        identity = QVBoxLayout()
        identity.setSpacing(2)
        brand = QLabel("DRONE SIMULATION")
        brand.setObjectName("brand")
        self.context_label = QLabel("Connecting to the local simulation")
        self.context_label.setObjectName("sectionSubtitle")
        identity.addWidget(brand)
        identity.addWidget(self.context_label)
        header_layout.addLayout(identity)
        header_layout.addStretch()
        header_layout.addWidget(VehicleSelection(self.client))
        self.connection_badge = StatusBadge()
        header_layout.addWidget(self.connection_badge)
        self.setMenuWidget(header)
        self.setCentralWidget(
            build_workspace(
                default_registry(self.client),
                self.client.preferences["map_width_percent"],
                self.client,
            )
        )
        self.client.preferences_changed.connect(self.apply_preferences)
        tools = QToolButton()
        tools.setText("Simulation ▾")
        tools.setAccessibleName("Simulation and mission tools")
        tools.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        menu = QMenu(tools)
        tools.setMenu(menu)
        self.actions = []
        for name, operation in (
            ("Pause world", "pause"),
            ("Resume world", "resume"),
            ("Request landing for targets", "stop"),
            ("Reset selected vehicles", "reset"),
        ):
            action = menu.addAction(name)
            action.setEnabled(False)
            action.triggered.connect(lambda checked=False, op=operation: self.client.execute(op))
            self.actions.append((operation, action))
        menu.addSeparator()
        menu.addAction("Load mission for selected targets…", self.load_mission)
        menu.addAction("Save mission + current recorded replay…", self.save_mission)
        header_layout.insertWidget(header_layout.count() - 1, tools)
        header_layout.insertWidget(header_layout.count() - 1, RestoreControl(self.client))
        self.save_destination = None
        self.save_vehicle_id: str | None = None
        self.client.mission_received.connect(self.write_mission)
        self.client.telemetry_received.connect(self.update_connection)
        self.client.resource_received.connect(self.update_context)
        self.client.restore_completed.connect(self.restore_presentation)
        self.client.error_received.connect(lambda message: self.statusBar().showMessage(message))
        self.client.operation_finished.connect(self.operation_finished)
        self.statusBar().showMessage("Waiting for PX4…")

    def operation_finished(self, operation: str) -> None:
        messages = {
            "start": "Mission start acknowledged",
            "pause": "Simulation paused",
            "resume": "Simulation resumed",
            "stop": "Landing requested",
            "reset": "Reset requested — waiting for the new vehicle run",
            "mission": "Mission uploaded",
            "missions/save": "Named mission saved",
            "missions/load": "Mission loaded onto the vehicle",
            "missions/archive": "Mission archived",
            "experiments/save": "Experiment configuration saved",
            "experiments/load": "Experiment restored",
            "viewer/camera": "Camera request acknowledged",
            "sensors/gnss": "GNSS input updated",
            "interference/clear": "Simulation faults cleared",
            "interference/link": "Application link settings applied",
            "interference/gnss": "GNSS degradation settings applied",
            "vehicles/read": "Vehicle parameters retrieved",
            "vehicles/configure": "Vehicle speed limits applied",
            "recordings/compare": "Recording comparison saved",
        }
        self.statusBar().showMessage(messages.get(operation, "Operation acknowledged"))

    def update_context(self, resource: str, payload: dict) -> None:
        if resource == "restore":
            self.restore_active = payload["restore"].get("active", False)
        if resource == "workspace":
            self.context_label.setText(payload["world"]["name"])

    def restore_presentation(self, payload: dict) -> None:
        """Apply acknowledged runtime presentation defaults without deleting saved records."""
        preferences = DesktopPreferences.validate(payload["defaults"]["preferences"])
        self.client.preferences = preferences
        self.client.preferences_changed.emit(preferences)
        self.client.workspace_reset.emit()
        for deck in self.findChildren(ControlDeck):
            deck.restore_defaults()
        tabs = self.findChild(QTabWidget, "rightWorkspaceTabs")
        if tabs is not None:
            tabs.setCurrentIndex(0)

    def apply_preferences(self, preferences: dict) -> None:
        workspace = self.centralWidget()
        width = workspace.width()
        left = width * min(65, max(50, preferences["map_width_percent"])) // 100
        workspace.setSizes([left, width - left])

    def update_connection(self, snapshot: dict) -> None:
        for operation, action in self.actions:
            # Recovery controls remain available when a paused clock stops telemetry.
            action.setEnabled(
                operation == "stop"
                if self.restore_active
                else operation in {"pause", "resume", "reset", "stop"}
                or snapshot.get("ready", False)
            )
        if snapshot.get("simulation_paused"):
            status = "Physics paused — Resume remains available"
            self.connection_badge.set_state("PAUSED", "warning")
        elif snapshot.get("ready"):
            status = f"{snapshot.get('label', 'Vehicle')} connected — ready"
            self.connection_badge.set_state(
                f"FLYING · {snapshot.get('flight_mode', '')}"
                if snapshot.get("armed")
                else "CONNECTED",
                "success",
            )
        elif snapshot.get("connected"):
            status = f"{snapshot.get('label', 'Vehicle')} connected — waiting for position and sensor readiness"
            fixed = snapshot.get("messages", {}).get("GPS_RAW_INT", {}).get("fix_type", 0) >= 3
            self.connection_badge.set_state(
                "PREFLIGHT CHECKS" if fixed else "GNSS DEGRADED", "warning"
            )
        else:
            status = "Waiting for PX4 connection…"
            self.connection_badge.set_state(
                "RESTARTING" if snapshot.get("restart_pending") else "LINK OFFLINE", "warning"
            )
        if status != self.connection_status:
            self.connection_status = status
            self.statusBar().showMessage(status)

    def load_mission(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Load QGroundControl mission",
            "/workspace/configs/missions",
            "Mission (*.plan)",
        )
        if filename:
            try:
                self.client.execute("mission", json.loads(Path(filename).read_text()))
            except (OSError, ValueError) as error:
                self.statusBar().showMessage(str(error))

    def closeEvent(self, event: QCloseEvent) -> None:
        self.client.close()
        for timer in self.findChildren(QTimer):
            timer.stop()
        for panel in self.findChildren(ViewerPanel):
            panel.close()
        super().closeEvent(event)
        # Foreign windows can remain visible after the host closes. They must not
        # keep this event loop alive or let attachment timers recreate a hidden host.
        application = QApplication.instance()
        if application is not None:
            application.quit()

    def save_mission(self) -> None:
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save vehicle mission",
            "/workspace/configs/missions/mission.plan",
            "Mission (*.plan)",
        )
        if filename:
            self.save_destination = Path(filename)
            self.save_vehicle_id = self.client.vehicle_id
            self.client.download_mission()

    def write_mission(self, mission: dict) -> None:
        if self.save_destination:
            try:
                self.save_destination.write_text(json.dumps(mission, indent=2), encoding="utf-8")
                replay = self.findChild(ReplayCaptureRow)
                queued = bool(
                    replay
                    and self.save_vehicle_id
                    and replay.queue_current_saved_run(self.save_vehicle_id)
                )
                self.statusBar().showMessage(
                    f"Saved {self.save_destination.name}"
                    + (
                        "; current recorded run requested for 3D replay"
                        if queued
                        else "; no current finalized run to render"
                    )
                )
            except OSError as error:
                self.statusBar().showMessage(str(error))
            finally:
                self.save_destination = None
                self.save_vehicle_id = None
