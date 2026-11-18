"""Workspace composition: full-height mission map and stacked right-side tools."""

import os
from typing import TYPE_CHECKING

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QButtonGroup,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSplitter,
    QStackedWidget,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from control_panel.panel import PanelRegistry
from control_panel.sensors.reconstruction import LiveReconstructionView
from desktop.flight_actions import FlightActionRow
from desktop.replay_capture import ReplayCaptureRow
from gazebo.panel import GazeboPanel
from qgroundcontrol.panel import QGroundControlPanel

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class ControlDeck(QWidget):
    """Two-row module tabs keep all features reachable in a narrow side pane."""

    def __init__(self, registry: PanelRegistry) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        navigation = QGridLayout()
        navigation.setSpacing(4)
        self.pages = QStackedWidget()
        self.buttons = QButtonGroup(self)
        aliases = {"Interference": "Faults", "PX4 Logs": "Logs"}
        selected = 0
        for index, spec in enumerate(registry.all()):
            button = QPushButton(aliases.get(spec.title, spec.title))
            button.setProperty("role", "module")
            button.setObjectName(f"module_{spec.key}")
            button.setToolTip(spec.title)
            button.setCheckable(True)
            self.buttons.addButton(button, index)
            navigation.addWidget(button, index // 4, index % 4)
            self.pages.addWidget(spec.factory())
            if spec.key == "telemetry":
                selected = index
            if spec.key == "visual_fleet" and os.environ.get("UAV_VISUAL_ODOMETRY_PROBE") == "true":
                selected = index
        self.buttons.idClicked.connect(self.pages.setCurrentIndex)
        self.buttons.button(selected).setChecked(True)
        self.pages.setCurrentIndex(selected)
        self.default_index = selected
        layout.addLayout(navigation)
        layout.addWidget(self.pages, 1)

    def restore_defaults(self) -> None:
        """Return module presentation to this deployment's startup page."""
        self.buttons.button(self.default_index).setChecked(True)
        self.pages.setCurrentIndex(self.default_index)


def section(title: str, subtitle: str, content: QWidget) -> QFrame:
    frame = QFrame()
    frame.setObjectName("workspaceSection")
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(8, 8, 8, 8)
    layout.setSpacing(8)
    heading = QHBoxLayout()
    label = QLabel(title)
    label.setObjectName("sectionTitle")
    description = QLabel(subtitle)
    description.setObjectName("sectionSubtitle")
    heading.addWidget(label)
    heading.addStretch()
    heading.addWidget(description)
    layout.addLayout(heading)
    layout.addWidget(content, 1)
    return frame


def build_workspace(
    registry: PanelRegistry,
    map_width_percent: int = 80,
    client: "SimulationClient | None" = None,
) -> QSplitter:
    tabs = QTabWidget()
    tabs.setObjectName("rightWorkspaceTabs")
    tabs.addTab(GazeboPanel(client), "World")
    tools = QWidget()
    tools_layout = QVBoxLayout(tools)
    tools_layout.setContentsMargins(8, 8, 8, 8)
    scope = QLabel("Readings follow Inspect. Choose command targets below the mission map.")
    scope.setWordWrap(True)
    scope.setObjectName("sectionSubtitle")
    tools_layout.addWidget(scope)
    tools_layout.addWidget(ControlDeck(registry), 1)
    tabs.addTab(tools, "Tools")
    workspace = QSplitter(Qt.Orientation.Horizontal)
    workspace.setHandleWidth(8)
    mission_area = QWidget()
    mission_layout = QVBoxLayout(mission_area)
    mission_layout.setContentsMargins(0, 0, 0, 0)
    mission_layout.setSpacing(4)
    qgc_view = QGroundControlPanel()
    qgc_view.setObjectName("qgcEmbeddedView")
    mission_layout.addWidget(qgc_view, 1)
    if client is not None:
        flight_actions = FlightActionRow(client)
        replay_capture = ReplayCaptureRow(client)
        flight_actions.targets_changed.connect(replay_capture.set_targets)
        flight_actions.targets_changed.connect(client.set_action_targets)
        client.set_action_targets(())
        client.restore_completed.connect(lambda payload: flight_actions.clear_selection())
        mission_layout.addWidget(flight_actions)
        mission_layout.addWidget(replay_capture)
        replay_scroll = QScrollArea()
        replay_scroll.setWidgetResizable(True)
        replay_scroll.setWidget(replay_capture.offline_page)
        replay_index = tabs.addTab(replay_scroll, "Replay")
        replay_capture.offline_toggle.clicked.connect(lambda: tabs.setCurrentIndex(replay_index))
        tabs.addTab(LiveReconstructionView(client), "Reconstruction")

        def show_scope() -> None:
            inspected = flight_actions.vehicles.get(client.vehicle_id, {}).get("label", "vehicle")
            names = [
                flight_actions.vehicles[key].get("label", key)
                for key in sorted(flight_actions.selected_ids)
            ]
            scope.setText(
                f"Readings: {inspected} · Edits: "
                + (", ".join(names) or "none")
                + ". Choose targets below the mission map."
            )

        flight_actions.targets_changed.connect(lambda targets: show_scope())
        client.selection_changed.connect(lambda vehicle_id: show_scope())
    workspace.addWidget(section("Mission", "", mission_area))
    workspace.addWidget(tabs)
    tabs.setMinimumWidth(410)
    workspace.setStretchFactor(0, 3)
    workspace.setStretchFactor(1, 2)
    # Keep the world view usable even when older saved preferences requested a narrow tools pane.
    initial_percent = min(65, max(50, map_width_percent))
    workspace.setSizes([12 * initial_percent, 12 * (100 - initial_percent)])
    workspace.setChildrenCollapsible(False)
    return workspace
