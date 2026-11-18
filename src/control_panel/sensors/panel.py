"""Live sensor health and acknowledged GNSS input control."""

import time
from pathlib import Path
from typing import TYPE_CHECKING

from PySide6.QtGui import QColor
from PySide6.QtWidgets import QCheckBox, QFileDialog, QLabel, QTableWidget, QTableWidgetItem

from control_panel.form import PanelView
from control_panel.sensors.live import LiveStereoView
from control_panel.sensors.mapping import RecordedMapDialog
from control_panel.sensors.perception import PerceptionCapabilities
from control_panel.sensors.recording import StereoRecordingDialog

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class SensorsPanel(PanelView):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__(client)
        self._stereo_implemented = False
        self._mapping_implemented = False
        self._mapping_ready = False
        self._mapping_received_monotonic = 0.0
        self.enabled = QCheckBox("Enable simulated GNSS fix")
        self.enabled.setChecked(True)
        self.form.addWidget(self.enabled)
        self.applied = QLabel("Waiting for applied sensor configuration…")
        self.form.addWidget(self.applied)
        self.button(
            "Apply GNSS input",
            lambda: self.send("sensors/gnss", {"enabled": self.enabled.isChecked()}),
        )
        self.camera_views = LiveStereoView(client)
        self.form.addWidget(self.camera_views)
        self.table = QTableWidget(7, 3)
        self.table.setHorizontalHeaderLabels(["Sensor", "Health", "Capability"])
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setMaximumHeight(200)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.form.addWidget(self.table)
        self.details = QLabel()
        self.details.setWordWrap(True)
        self.form.addWidget(self.details)
        self.perception = PerceptionCapabilities(client, self.send)
        self.form.addWidget(self.perception)
        self.button("Open stereo recording", self.open_recording)
        self.button("Open mapping result", self.open_mapping)
        self.finish()
        client.telemetry_received.connect(self.update_health)
        client.resource_received.connect(self.resource)

    def open_recording(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Open stereo recording report", "outputs/reports", "Stereo report (*.json)"
        )
        if path:
            dialog = StereoRecordingDialog(Path(path), self)
            dialog.exec()

    def open_mapping(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Open recorded mapping report", "outputs/reports", "Mapping report (*.json)"
        )
        if path:
            RecordedMapDialog(Path(path), self).exec()

    def resource(self, name: str, payload: dict) -> None:
        if name == "perception/map":
            self._mapping_ready = payload.get("ready", False)
            self._mapping_received_monotonic = time.monotonic()
            return
        if name == "perception":
            self._stereo_implemented = any(
                item["key"] == "stereo_capture" and item["implemented"] for item in payload["items"]
            )
            self._mapping_implemented = any(
                item["key"] == "visual_slam" and item["implemented"] for item in payload["items"]
            )
            self.describe_limits()
            return
        if name != "workspace":
            return
        environment = payload["environment"]
        inspected = payload.get("vehicle", {}).get("label", "Inspected vehicle")
        self.applied.setText(
            f"{inspected} · GNSS: {'enabled' if environment['gnss_enabled'] else 'disabled'}; "
            f"configured satellites: {environment['satellites']}"
        )
        self.describe_limits()

    def describe_limits(self) -> None:
        camera = (
            "Stereo cameras are configured; stream health is shown below. "
            if self._stereo_implemented
            else "No stereo camera model is configured. "
        )
        self.details.setText(
            camera
            + (
                "ORB-SLAM3 supplies observation-only camera motion and mapping. "
                if self._mapping_implemented
                else "No live visual estimator is configured. "
            )
            + "LiDAR health requires a configured measured-range source. "
            "IMU, magnetometer and barometer report health; live switching is unavailable."
        )

    def reset_panel(self) -> None:
        super().reset_panel()
        self._stereo_implemented = False
        self._mapping_implemented = False
        self._mapping_ready = False
        self._mapping_received_monotonic = 0.0
        self.enabled.setChecked(True)

    def update_health(self, snapshot: dict) -> None:
        generation = snapshot.get("run_id")
        identity = snapshot.get("vehicle_id")
        observed = {
            item["system"]: item
            for item in snapshot.get("health", [])
            if item.get("vehicle_id") == identity
            and item.get("controller_generation") == generation
        }
        for row, (title, system, capability) in enumerate(
            (
                ("GNSS", "GPS fix", "Live fix control"),
                ("IMU / gyro", "IMU", "Health monitoring"),
                ("Magnetometer", "Magnetometer", "Health monitoring"),
                ("Barometer", "Barometer", "Health monitoring"),
                (
                    "Camera",
                    "Camera",
                    "Rendered stereo" if self._stereo_implemented else "No capture pipeline",
                ),
                ("LiDAR", "LiDAR", "Configured measured-range source"),
                (
                    "Visual odometry",
                    None,
                    "ORB-SLAM3 observer" if self._mapping_implemented else "No estimation pipeline",
                ),
            )
        ):
            report = observed.get(system, {})
            state = report.get("state", "stale_or_unknown")
            reason = report.get("reason", "Current-generation health evidence unavailable")
            if system is None:
                state = (
                    "unsupported"
                    if not self._mapping_implemented
                    else "stale_or_unknown"
                    if time.monotonic() - self._mapping_received_monotonic > 1.5
                    else "healthy"
                    if self._mapping_ready
                    else "degraded"
                )
                reason = (
                    "ORB-SLAM3 observation only; native vision fusion is a separate fleet badge"
                )
            text = {
                "healthy": "Healthy",
                "degraded": "Degraded",
                "unavailable": "Unavailable",
                "stale_or_unknown": "Stale / unknown",
                "unsupported": "Unsupported",
            }[state]
            if system == "GPS fix" and state == "unavailable":
                text = "No fix"
            colour = {
                "healthy": "#77d7b4",
                "degraded": "#efc47d",
                "unavailable": "#ff7b85",
                "stale_or_unknown": "#829bb2",
                "unsupported": "#829bb2",
            }[state]
            for column, value in enumerate((title, text, capability)):
                item = QTableWidgetItem(value)
                item.setToolTip(
                    f"{identity} · {generation}\n{reason}\n"
                    f"Evidence age: {report.get('evidence_age_seconds')} s"
                )
                if column == 1:
                    item.setForeground(QColor(colour))
                self.table.setItem(row, column, item)
        self.table.resizeColumnsToContents()
