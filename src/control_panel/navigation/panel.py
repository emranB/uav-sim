"""Focused navigation intent and observed geometry, with explicit terminal states."""

import math
from typing import TYPE_CHECKING

from PySide6.QtCore import QPointF, QTimer
from PySide6.QtGui import QColor, QPainter, QPaintEvent, QPen
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class NavigationMap(QWidget):
    """Small local ENU map; only measured occupied points and planned paths are drawn."""

    def __init__(self) -> None:
        super().__init__()
        self.snapshot = {}
        self.setMinimumHeight(200)

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#101b29"))
        painter.setPen(QColor("#aec4d5"))
        painter.drawText(
            8, 18, "Local ENU metres · dark = unknown · blue = observed free · orange = occupied"
        )
        position = self.snapshot.get("position_enu_m", [0, 0, 0])
        scale = min(self.width(), self.height()) / 36

        def pixel(point) -> QPointF:
            return QPointF(
                self.width() / 2 + (point[0] - position[0]) * scale,
                self.height() / 2 - (point[1] - position[1]) * scale,
            )

        painter.setPen(QPen(QColor("#243f53"), max(2, scale * 0.4)))
        for point in self.snapshot.get("observed_free_enu_m", []):
            painter.drawPoint(pixel(point))
        painter.setPen(QPen(QColor("#f2a54a"), 3))
        for point in self.snapshot.get("occupied_enu_m", []):
            painter.drawPoint(pixel(point))
        painter.setPen(QPen(QColor("#58dfbd"), 2))
        path = self.snapshot.get("path_enu_m", [])
        for start, end in zip(path, path[1:]):
            painter.drawLine(pixel(start), pixel(end))
        painter.setPen(QPen(QColor("#66bfff"), 5))
        painter.drawPoint(pixel(position))


class NavigationPanel(QWidget):
    """One focused vehicle; collection selection never starts multiple controllers."""

    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.client = client
        layout = QVBoxLayout(self)
        explanation = QLabel(
            "Static obstacles at one flight height; clear launch column required. "
            "Obstacle sensing is selected below. Localization uses the current profile's "
            "validated PX4 and visual sources. QGC mission commands must remain idle."
        )
        explanation.setWordWrap(True)
        layout.addWidget(explanation)
        form = QFormLayout()
        self.mode = QComboBox()
        self.mode.addItem("Horizontal range", "range")
        form.addRow("Obstacle sensing", self.mode)
        self.east, self.north = QDoubleSpinBox(), QDoubleSpinBox()
        for field in (self.east, self.north):
            field.setRange(-15, 15)
            field.setSuffix(" m")
        self.north.setValue(14)
        form.addRow("Goal east from start", self.east)
        form.addRow("Goal north from start", self.north)
        layout.addLayout(form)
        self.state = QLabel("Waiting for navigation capability")
        self.state.setWordWrap(True)
        layout.addWidget(self.state)
        self.goal_constraint = QLabel("Goal distance: 2–15m")
        layout.addWidget(self.goal_constraint)
        self._can_start = False
        self.start = QPushButton("Navigate focused vehicle")
        self.start.setEnabled(False)
        self.start.clicked.connect(self.navigate)
        layout.addWidget(self.start)
        cancel = QPushButton("Cancel and land focused vehicle")
        cancel.clicked.connect(
            lambda: self.client.execute("navigation/cancel", {"vehicle_id": self.client.vehicle_id})
        )
        layout.addWidget(cancel)
        self.mode.currentIndexChanged.connect(self.refresh_goal_constraint)
        self.east.valueChanged.connect(self.refresh_goal_constraint)
        self.north.valueChanged.connect(self.refresh_goal_constraint)
        self.map = NavigationMap()
        layout.addWidget(self.map)
        self.timer = QTimer(self)
        self.timer.setInterval(500)
        self.timer.timeout.connect(lambda: self.client.fetch("navigation"))
        self.timer.start()
        client.resource_received.connect(self.receive)
        client.selection_changed.connect(self.selection_changed)

    def refresh_goal_constraint(self) -> None:
        """Explain the selected mode's validated distance envelope before issuing intent."""
        maximum = 10 if self.mode.currentData() in {"camera", "fused"} else 15
        distance = math.hypot(self.east.value(), self.north.value())
        valid = 2 <= distance <= maximum
        self.goal_constraint.setText(
            f"Goal distance: 2–{maximum}m. Current: {distance:.1f}m"
            + ("; adjust the goal to enable navigation." if not valid else "")
        )
        self.start.setEnabled(self._can_start and valid)

    def selection_changed(self, vehicle_id: str) -> None:
        self._can_start = False
        self.start.setEnabled(False)
        self.state.setText(f"{vehicle_id}: waiting for focused navigation status")
        self.map.snapshot = {}
        self.map.update()

    def navigate(self) -> None:
        self.client.execute(
            "navigation/start",
            {
                "vehicle_id": self.client.vehicle_id,
                "goal_offset_enu_m": [self.east.value(), self.north.value()],
                "obstacle_mode": self.mode.currentData(),
            },
        )

    def receive(self, resource: str, payload: dict) -> None:
        if resource != "navigation":
            return
        state = payload.get("state", "unavailable")
        available = payload.get("supported_obstacle_modes", ["range"])
        selected = self.mode.currentData()
        labels = {
            "range": "Horizontal range",
            "camera": "Stereo cameras",
            "fused": "Stereo + range",
        }
        if [self.mode.itemData(i) for i in range(self.mode.count())] != available:
            self.mode.clear()
            for mode in available:
                self.mode.addItem(labels[mode], mode)
            self.mode.setCurrentIndex(max(0, self.mode.findData(selected)))
        active = payload.get("active", False)
        if active and payload.get("obstacle_mode") in available:
            self.mode.setCurrentIndex(self.mode.findData(payload["obstacle_mode"]))
        goal = payload.get("goal_offset_enu_m")
        if active and isinstance(goal, list) and len(goal) == 2:
            self.east.setValue(goal[0])
            self.north.setValue(goal[1])
        self.mode.setEnabled(not active)
        self.east.setEnabled(not active)
        self.north.setEnabled(not active)
        self.state.setText(
            f"{payload.get('vehicle_id')}: {state}. "
            f"{'Map frozen. ' if payload.get('map_frozen') else ''}"
            f"{payload.get('error') or payload.get('blocked_reason') or ''}"
        )
        self._can_start = (
            payload.get("available", False)
            and not payload.get("active", False)
            and state in {"idle", "reached", "blocked", "failed", "cancelled"}
        )
        self.refresh_goal_constraint()
        self.map.snapshot = payload
        self.map.update()
