"""Gazebo's Qt 5 window requires native sizing inside the Qt 6 workspace."""

import json
import math
import os
from pathlib import Path
from typing import TYPE_CHECKING

from PySide6.QtCore import QProcess
from PySide6.QtWidgets import (
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from desktop.viewer import ViewerPanel

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class GazeboPanel(ViewerPanel):
    def __init__(self, client: "SimulationClient | None" = None) -> None:
        self.resizer = None
        self.last_native_size = None
        super().__init__("Gazebo", Path("/runtime/Gazebo.window"))
        self.resizer = QProcess(self)
        self.client = client
        self.vehicle_id = "vehicle_1"
        self.preset = "chase"
        self.pending = False
        self.offset_enu_m = (-2.5, -2.5, 1.8)
        navigation = QWidget()
        navigation.setObjectName("cameraToolbar")
        navigation.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        navigation.setFixedHeight(38)
        navigation.setToolTip("ENU axes: red East, green North, blue Up. Grid spacing: 2 metres.")
        controls = QHBoxLayout(navigation)
        controls.setContentsMargins(5, 3, 5, 3)
        controls.setSpacing(5)
        for title, preset in (
            ("Close", "close"),
            ("Chase", "chase"),
            ("Top", "top"),
            ("Wide", "wide"),
            ("Both", "overview"),
        ):
            button = QToolButton()
            button.setText(title)
            button.setProperty("role", "camera")
            button.setFixedHeight(28)
            button.setToolTip(f"Center the {preset} view on the selected vehicle")
            button.clicked.connect(lambda checked=False, preset=preset: self.camera(preset))
            controls.addWidget(button)
        self.follow = QCheckBox("Follow")
        self.follow.setToolTip("Follow the selected vehicle from the current angle and distance")
        self.follow.clicked.connect(self.toggle_follow)
        controls.addWidget(self.follow)
        orbit = QToolButton()
        orbit.setText("Orbit…")
        orbit.setProperty("role", "camera")
        orbit.setFixedHeight(28)
        orbit.setToolTip("Adjust any angle and distance with Follow on or off")
        orbit.clicked.connect(self.orbit)
        controls.addWidget(orbit)
        controls.addStretch()
        self.camera_status = QLabel("●")
        self.camera_status.setFixedSize(12, 20)
        self.set_camera_status("Viewer connecting", "warning")
        controls.addWidget(self.camera_status)
        self.panel_layout.insertWidget(0, navigation)
        if client is not None:
            client.resource_received.connect(self.resource)
            client.operation_result.connect(self.camera_applied)
            client.operation_failed.connect(self.camera_failed)
            client.selection_changed.connect(self.selection_changed)

    def selection_changed(self, vehicle_id: str) -> None:
        self.vehicle_id = vehicle_id
        self.pending = False
        self.set_camera_status("Choose view for selected vehicle", "warning")

    def set_camera_status(self, description: str, state: str) -> None:
        self.camera_status.setToolTip(description)
        color = "#77d7b4" if state == "success" else "#efc47d"
        self.camera_status.setStyleSheet(f"color: {color}; font-size: 13px; padding: 0;")

    def camera(self, preset: str) -> None:
        if self.client is None or self.pending:
            return
        self.preset = preset
        self.pending = True
        self.set_camera_status("Applying view", "warning")
        self.client.execute(
            "viewer/camera",
            {
                "vehicle_id": self.vehicle_id,
                "preset": preset,
                "follow": False if preset == "overview" else self.follow.isChecked(),
                **(
                    {
                        "look_at_enu_m": json.loads(
                            os.environ.get("UAV_OVERVIEW_FOCUS_ENU_M", "[0,0,0]")
                        )
                    }
                    if preset == "overview"
                    else {}
                ),
            },
        )

    def toggle_follow(self, follow: bool) -> None:
        if self.client is None or self.pending:
            return
        self.pending = True
        self.set_camera_status("Updating follow without changing view", "warning")
        self.client.execute("viewer/follow", {"follow": follow})

    def orbit(self) -> None:
        if self.client is None or self.pending:
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("Orbit selected vehicle")
        layout = QVBoxLayout(dialog)
        hint = QLabel("Adjust the camera freely. Follow keeps this offset as the drone moves.")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        fields = QFormLayout()
        x, y, z = self.offset_enu_m
        radius = math.sqrt(x * x + y * y + z * z)
        azimuth = QDoubleSpinBox()
        azimuth.setRange(-180, 180)
        azimuth.setSuffix("°")
        azimuth.setValue(math.degrees(math.atan2(y, x)))
        elevation = QDoubleSpinBox()
        elevation.setRange(5, 89)
        elevation.setSuffix("°")
        elevation.setValue(math.degrees(math.atan2(z, math.hypot(x, y))))
        distance = QDoubleSpinBox()
        distance.setRange(0.8, 30)
        distance.setSuffix(" m")
        distance.setValue(radius)
        for title, widget in (
            ("Around vehicle", azimuth),
            ("Elevation", elevation),
            ("Distance", distance),
        ):
            fields.addRow(title, widget)
        layout.addLayout(fields)
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Apply | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.button(QDialogButtonBox.StandardButton.Apply).clicked.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            yaw = math.radians(azimuth.value())
            pitch = math.radians(elevation.value())
            radius = distance.value()
            offset = (
                radius * math.cos(pitch) * math.cos(yaw),
                radius * math.cos(pitch) * math.sin(yaw),
                radius * math.sin(pitch),
            )
            self.pending = True
            self.preset = "custom"
            self.set_camera_status("Applying custom orbit", "warning")
            self.client.execute(
                "viewer/camera", {"offset_enu_m": offset, "follow": self.follow.isChecked()}
            )

    def resource(self, name: str, payload: dict) -> None:
        if name != "workspace" or self.pending:
            return
        self.vehicle_id = payload["vehicle"]["vehicle_id"]
        state = payload.get("viewer", {})
        following = state.get("following", False) and state.get("target") == payload["vehicle"].get(
            "model_name"
        )
        self.follow.setChecked(following)
        camera = state.get("camera_position_enu_m")
        vehicle = state.get("vehicle_position_enu_m")
        if camera is not None and vehicle is not None:
            self.offset_enu_m = tuple(value - origin for value, origin in zip(camera, vehicle))
        if state.get("available"):
            self.set_camera_status(
                "Following selected vehicle" if following else "Manual camera", "success"
            )
        else:
            self.set_camera_status("Viewer offline", "warning")

    def camera_applied(self, operation: str, payload: dict) -> None:
        if operation in {"viewer/camera", "viewer/follow"}:
            self.pending = False
            self.client.fetch("workspace")

    def camera_failed(self, operation: str, message: str) -> None:
        if operation in {"viewer/camera", "viewer/follow"}:
            self.pending = False
            self.set_camera_status("Camera unavailable", "warning")
            self.camera_status.setToolTip(message)

    def fit_viewer(self) -> None:
        if (
            not self.resizer
            or not self.container
            or self.window_id is None
            or self.resizer.state() != QProcess.ProcessState.NotRunning
        ):
            return
        ratio = self.devicePixelRatioF()
        width = max(1, round(self.container.width() * ratio))
        height = max(1, round(self.container.height() * ratio))
        size = (self.window_id, self.window_revision, width, height)
        if size == self.last_native_size:
            return
        self.last_native_size = size
        # The Gazebo GUI loads its own saved size after reparenting. Reassert host bounds.
        self.resizer.start("xdotool", ["windowsize", str(self.window_id), str(width), str(height)])
