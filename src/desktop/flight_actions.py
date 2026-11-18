"""Explicit fleet shortcuts presented immediately below the QGC view."""

import time
import uuid
from typing import TYPE_CHECKING

from PySide6.QtCore import QTimer, Signal
from PySide6.QtGui import QAction, QCloseEvent
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QMenu,
    QPushButton,
    QToolButton,
    QVBoxLayout,
)

from control_panel.health import VehicleHealthBadges

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class FlightActionRow(QFrame):
    """Present server-owned capabilities and acknowledge immutable dispatch targets."""

    targets_changed = Signal(tuple)

    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.setObjectName("qgcQuickActions")
        self.client = client
        self.vehicles: dict[str, dict] = {}
        self.checkboxes: dict[str, QAction] = {}
        self.selected_ids: set[str] = set()
        self.restore_active = False
        self.pending_id = ""
        self.pending_names: dict[str, str] = {}
        self.stereo_horizontal_reference = False
        self.last_received_monotonic = 0.0
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 7, 10, 7)
        layout.setSpacing(5)
        heading = QHBoxLayout()
        self.selector = QToolButton()
        self.selector.setObjectName("flightTargetSelector")
        self.selector.setText("Choose vehicles ▾")
        self.selector.setAccessibleName("Quick-action vehicle checkboxes")
        self.selector.setToolTip(
            "Choose quick-action targets explicitly. Viewed vehicle and QGC focus stay independent."
        )
        self.selector.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        self.menu = QMenu(self.selector)
        self.selector.setMenu(self.menu)
        heading.addWidget(self.selector)
        self.targets = QLabel("0 selected · No vehicles")
        self.targets.setObjectName("targetSummary")
        self.targets.setWordWrap(True)
        self.targets.hide()
        self.health = VehicleHealthBadges(
            columns=7,
            systems=(
                "GPS fix",
                "GPS fusion",
                "Vision fusion",
                "Diagnostic log",
                "Link",
                "Camera",
            ),
        )
        self.extra = QFrame()
        extra_layout = QVBoxLayout(self.extra)
        extra_layout.setContentsMargins(0, 0, 0, 0)
        buttons = QHBoxLayout()
        self.buttons: dict[str, QPushButton] = {}
        for action, text in (
            ("start", "Start mission"),
            ("land", "Land"),
            ("takeoff", "Take off"),
            ("return", "Return home"),
            ("pause", "Pause mission"),
        ):
            button = QPushButton(text)
            button.setObjectName(f"quick_{action}")
            if action == "start":
                button.setProperty("role", "primary")
            if action == "land":
                button.setProperty("role", "landing")
            button.clicked.connect(lambda checked=False, key=action: self.dispatch(key))
            (heading if action in {"start", "land"} else buttons).addWidget(button)
            self.buttons[action] = button
        self.more = QToolButton()
        self.more.setText("More ▾")
        self.more.setToolTip("Additional flight capabilities and detailed sensor health")
        self.more.setCheckable(True)
        self.more.toggled.connect(self.extra.setVisible)
        heading.addWidget(self.more)
        layout.addLayout(heading)
        layout.addWidget(self.health)
        extra_layout.addLayout(buttons)
        self.detailed_health = VehicleHealthBadges()
        extra_layout.addWidget(self.detailed_health)
        layout.addWidget(self.extra)
        self.extra.hide()
        self.readiness = QLabel()
        self.readiness.setObjectName("sectionSubtitle")
        self.readiness.setWordWrap(True)
        layout.addWidget(self.readiness)
        self.feedback = QLabel()
        self.feedback.setObjectName("operationFeedback")
        self.feedback.setWordWrap(True)
        layout.addWidget(self.feedback)
        client.resource_received.connect(self.receive)
        client.target_operation_result.connect(self.finished)
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.poll)
        self.timer.start()
        self.refresh()

    def poll(self) -> None:
        """Refresh fleet readiness independently of the focused vehicle's panel polling."""
        self.client.fetch("vehicles")
        self.refresh()

    def closeEvent(self, event: QCloseEvent) -> None:
        self.timer.stop()
        super().closeEvent(event)

    def receive(self, resource: str, payload: dict) -> None:
        """Refresh readiness without selecting newly discovered vehicles."""
        if resource == "restore":
            self.restore_active = payload["restore"].get("active", False)
            self.refresh()
            return
        if resource != "vehicles":
            return
        self.last_received_monotonic = time.monotonic()
        self.vehicles = {row["vehicle_id"]: row for row in payload["items"]}
        self.selected_ids.intersection_update(self.vehicles)
        self.targets_changed.emit(tuple(sorted(self.selected_ids)))
        if set(self.checkboxes) != set(self.vehicles):
            # Preserve open-menu actions across telemetry refreshes. Replacing them
            # can discard the focused action between a pointer move and its click.
            self.menu.close()
            self.menu.clear()
            self.checkboxes = {}
            for identity in self.vehicles:
                action = self.menu.addAction(identity)
                action.setData(identity)
                action.setCheckable(True)
                action.toggled.connect(
                    lambda checked, target=identity: self.select(target, checked)
                )
                self.checkboxes[identity] = action
            self.menu.addSeparator()
            self.menu.addAction("Select all listed vehicles", self.select_all)
            self.menu.addAction("Clear selection", self.clear_selection)
        for identity, row in self.vehicles.items():
            action = self.checkboxes[identity]
            action.setText(row.get("label", identity))
            action.blockSignals(True)
            action.setChecked(identity in self.selected_ids)
            action.blockSignals(False)
        self.refresh()

    def select(self, vehicle_id: str, selected: bool) -> None:
        """Change future dispatch selection without altering any pending command."""
        if selected:
            self.selected_ids.add(vehicle_id)
        else:
            self.selected_ids.discard(vehicle_id)
        self.targets_changed.emit(tuple(sorted(self.selected_ids)))
        self.refresh()

    def select_all(self) -> None:
        """Explicitly select the currently listed fleet, without future auto-expansion."""
        for action in self.checkboxes.values():
            action.setChecked(True)

    def clear_selection(self) -> None:
        """Return to the safe empty selection."""
        for action in self.checkboxes.values():
            action.setChecked(False)

    def blockers(self, action: str) -> list[str]:
        if self.restore_active and action != "land":
            return ["Master Reset owns fleet recovery; landing remains available"]
        reasons = []
        for identity in sorted(self.selected_ids):
            row = self.vehicles[identity]
            if time.monotonic() - self.last_received_monotonic > 3:
                reasons.append(f"{row.get('label', identity)}: refresh stale vehicle readiness")
                continue
            status = row.get("flight_actions", {}).get(action, {})
            if not status.get("enabled"):
                reasons.append(
                    f"{row.get('label', identity)}: "
                    + status.get("reason", "Wait for supported backend flight capabilities")
                )
        return reasons

    def refresh(self) -> None:
        """Render explicit target scope and action-specific server readiness reasons."""
        names = [self.vehicles[key].get("label", key) for key in sorted(self.selected_ids)]
        self.health.present(
            list(self.vehicles.values()), time.monotonic() - self.last_received_monotonic
        )
        self.detailed_health.present(
            list(self.vehicles.values()), time.monotonic() - self.last_received_monotonic
        )
        self.targets.setText(f"{len(names)} selected · " + (", ".join(names) or "No vehicles"))
        self.selector.setText("Targets: " + (" + ".join(names) or "none") + " ▾")
        self.selector.setToolTip(
            self.targets.text() + "; applies to flight, capture and vehicle edits"
        )
        self.feedback.setVisible(bool(self.feedback.text()))
        for action, button in self.buttons.items():
            reasons = self.blockers(action)
            enabled = bool(names) and not self.pending_id and not reasons
            button.setEnabled(enabled)
            button.setToolTip(
                "Waiting for the pending command"
                if self.pending_id
                else "; ".join(reasons)
                if reasons
                else "Choose at least one vehicle"
                if not names
                else "All selected vehicles pass preflight; commands are independent, not atomic"
            )
        reasons = self.blockers("start") if names else []
        critical = [
            f"{vehicle.get('label', vehicle['vehicle_id'])}: {status['system']} "
            + status["state"].replace("_", " ")
            for vehicle in self.vehicles.values()
            for status in vehicle.get("health", [])
            if status["state"] in {"unavailable", "degraded"}
        ]
        summary = critical or reasons
        self.readiness.setText(
            " · ".join(summary[:2])
            if summary
            else "Choose targets to enable supported actions"
            if not names
            else ""
        )
        self.readiness.setToolTip("\n".join(critical + reasons))
        self.readiness.setMaximumHeight(44)
        self.readiness.setVisible(bool(self.readiness.text()))

    def dispatch(self, action: str) -> None:
        """Snapshot the selected IDs and labels before asynchronous dispatch."""
        if action not in {"start", "land"} or self.pending_id or not self.selected_ids:
            return
        if self.blockers(action):
            self.refresh()
            return
        targets = sorted(self.selected_ids)
        self.pending_names = {key: self.vehicles[key].get("label", key) for key in targets}
        self.pending_id = uuid.uuid4().hex
        self.feedback.setText(
            f"Sending {self.buttons[action].text()} to {', '.join(self.pending_names.values())} "
            f"({len(targets)}). Waiting for each acknowledgement…"
        )
        self.feedback.setProperty("state", "pending")
        self.refresh()
        if action == "start" and self.stereo_horizontal_reference:
            self.client.execute_targets(
                "flight/start",
                targets,
                self.pending_id,
                stereo_horizontal_reference=True,
            )
        else:
            self.client.execute_targets(f"flight/{action}", targets, self.pending_id)

    def finished(self, request_id: str, payload: dict) -> None:
        """Show every original target's acknowledgement, including partial failures."""
        if request_id != self.pending_id:
            return
        results = payload["results"]
        messages = []
        passed = True
        for identity, name in self.pending_names.items():
            result = results.get(identity, {})
            accepted = result.get("accepted") is True
            passed = passed and accepted
            messages.append(
                f"{name}: acknowledged"
                if accepted
                else f"{name}: {result.get('error', 'No acknowledgement received')}"
            )
        self.feedback.setText(" · ".join(messages))
        self.feedback.setProperty("state", "success" if passed else "error")
        self.feedback.style().unpolish(self.feedback)
        self.feedback.style().polish(self.feedback)
        self.pending_id = ""
        self.refresh()
