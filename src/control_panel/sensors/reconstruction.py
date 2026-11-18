"""Embedded observational reconstruction, with no vehicle-command controls."""

import time
from typing import TYPE_CHECKING

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCloseEvent, QHideEvent, QShowEvent
from PySide6.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QSlider, QVBoxLayout, QWidget

from common.logging.mapping import RecordedMap
from control_panel.sensors.mapping import MapCanvas

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class LiveReconstructionView(QWidget):
    """Present only the focused vehicle's current, timestamped estimation session."""

    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.client = client
        self.snapshot: dict = {}
        self.last_valid: dict = {}
        self.last_received = 0.0
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.status = QLabel("Waiting for live reconstruction")
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(2 * self.status.fontMetrics().lineSpacing())
        layout.addWidget(self.status)
        layers = QHBoxLayout()
        self.landmarks = QCheckBox("Landmarks")
        self.surfaces = QCheckBox("Surfaces")
        self.trail = QCheckBox("Trail")
        for checkbox in (self.landmarks, self.surfaces, self.trail):
            checkbox.setChecked(True)
            checkbox.toggled.connect(self.present)
            layers.addWidget(checkbox)
        layout.addLayout(layers)
        self.canvas = MapCanvas()
        self.canvas.setMinimumSize(300, 160)
        self.canvas.trajectory_label = "estimated camera trail"
        layout.addWidget(self.canvas, 1)
        angles = QHBoxLayout()
        for title, bounds, value, callback in (
            ("Rotate", (-180, 180), 35, self.canvas.set_azimuth),
            ("Tilt", (-89, 89), 25, self.canvas.set_elevation),
        ):
            angles.addWidget(QLabel(title))
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(*bounds)
            slider.setValue(value)
            slider.valueChanged.connect(callback)
            angles.addWidget(slider)
        layout.addLayout(angles)
        self.legend = QLabel(
            "Local optical frame · metres · amber: measured surfaces · unknown ≠ clear"
        )
        self.legend.setWordWrap(True)
        self.legend.setMinimumHeight(2 * self.legend.fontMetrics().lineSpacing())
        layout.addWidget(self.legend)
        self.credit = QLabel("Terrain includes data provided by Service Nova Scotia")
        self.credit.setWordWrap(True)
        self.credit.setToolTip(
            "This product has been produced for the UAV Simulation Platform project and includes "
            "data provided by the Department of Service Nova Scotia. The incorporation of that "
            "data does not constitute endorsement by the Department. Terrain surface texture "
            "is synthetic. See the data license and source notice packaged with terrain assets."
        )
        self.credit.hide()
        layout.addWidget(self.credit)
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.refresh)
        client.resource_received.connect(self.resource)
        client.selection_changed.connect(self.clear)
        client.workspace_reset.connect(self.clear)

    def refresh(self) -> None:
        if self.isVisible():
            if self.last_received and time.monotonic() - self.last_received > 5:
                self.snapshot = {"ready": False, "error": "Map connection is stale"}
                self.present()
            self.client.fetch("perception/map")

    def clear(self, vehicle_id: str = "") -> None:
        self.snapshot = {}
        self.last_valid = {}
        self.last_received = 0.0
        self.canvas.surfaces = ()
        self.canvas.mapping = None
        self.canvas.update()
        self.status.setText(f"Waiting for {vehicle_id or 'current session'} reconstruction")

    def resource(self, name: str, payload: dict) -> None:
        if name == "workspace":
            self.credit.setVisible(payload.get("world", {}).get("scene_name") == "waterfront")
            return
        if name != "perception/map":
            return
        self.snapshot = payload
        if payload.get("session_id") and self.last_valid.get("session_id") not in {
            None,
            payload["session_id"],
        }:
            self.last_valid = {}
        if payload.get("ready"):
            self.last_valid = payload
        elif payload.get("map_frozen"):
            self.last_valid = {
                **payload,
                "timestamp_seconds": payload["last_tracked_timestamp_seconds"],
            }
        self.last_received = time.monotonic()
        self.present()

    def hideEvent(self, event: QHideEvent) -> None:
        self.timer.stop()
        super().hideEvent(event)

    def showEvent(self, event: QShowEvent) -> None:
        self.timer.start()
        super().showEvent(event)

    def closeEvent(self, event: QCloseEvent) -> None:
        self.timer.stop()
        super().closeEvent(event)

    def present(self) -> None:
        snapshot = self.snapshot
        self.canvas.dimmed = not snapshot.get("ready", False)
        if not snapshot.get("ready"):
            state = snapshot.get("tracking_state")
            explanation = snapshot.get("error") or (
                "waiting for sufficient stereo features"
                if state in {0, 1}
                else "tracking lost or stale images; no current position estimate"
            )
            if self.last_valid:
                self.status.setText(
                    f"FROZEN — NOT LIVE · {explanation}. "
                    f"Last tracked map: sim {self.last_valid['timestamp_seconds']:.1f}s"
                )
                self.draw_snapshot(self.last_valid)
                return
            self.canvas.mapping, self.canvas.surfaces = None, ()
            self.canvas.update()
            self.status.setText("Reconstruction unavailable: " + explanation)
            return
        self.status.setText(
            f"{snapshot['vehicle_id']} · Tracking · sim {snapshot['timestamp_seconds']:.1f}s · "
            f"{snapshot['age_seconds']:.1f}s old · "
            f"depth {snapshot['valid_depth_fraction']:.0%} measured"
        )
        self.draw_snapshot(snapshot)

    def draw_snapshot(self, snapshot: dict) -> None:
        """Draw observed map data; a frozen view never implies a current vehicle pose."""
        self.canvas.surfaces = (
            tuple(map(tuple, snapshot.get("surfaces", []))) if (self.surfaces.isChecked()) else ()
        )
        self.canvas.set_map(
            RecordedMap(
                tuple(map(tuple, snapshot.get("trajectory", []))) if self.trail.isChecked() else (),
                tuple(map(tuple, snapshot.get("landmarks", [])))
                if self.landmarks.isChecked()
                else (),
                "Live observation only",
            )
        )
