"""Presentation of normalized vehicle state from the application client."""

import json
import math
from pathlib import Path
from typing import TYPE_CHECKING

from PySide6.QtWidgets import (
    QFileDialog,
    QGridLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from control_panel.presentation import MetricCard, StatusBadge

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class TelemetryPanel(QWidget):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.snapshot = {}
        self.values = QLabel("Waiting for simulated vehicle telemetry…")
        self.values.setWordWrap(True)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        cards = QGridLayout()
        cards.setSpacing(8)
        self.altitude = MetricCard("RELATIVE ALTITUDE")
        self.speed = MetricCard("GROUND SPEED")
        self.mode = MetricCard("FLIGHT MODE")
        self.mode.value.setStyleSheet("font-size: 16px; font-weight: 600;")
        self.battery = MetricCard("BATTERY")
        for index, card in enumerate((self.altitude, self.speed, self.mode, self.battery)):
            cards.addWidget(card, index // 2, index % 2)
        layout.addLayout(cards)
        self.gps_badge = StatusBadge("Waiting for GNSS")
        layout.addWidget(self.gps_badge)
        layout.addWidget(self.values)
        export = QPushButton("Export telemetry snapshot")
        export.clicked.connect(self.export)
        layout.addWidget(export)
        heading = QLabel("Fleet overview · cards show the viewed vehicle")
        heading.setWordWrap(True)
        heading.setObjectName("sectionSubtitle")
        layout.addWidget(heading)
        self.fleet = QTableWidget(0, 4)
        self.fleet.setHorizontalHeaderLabels(["Vehicle", "Mode", "Alt (m)", "Link"])
        self.fleet.verticalHeader().hide()
        self.fleet.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.fleet.setAlternatingRowColors(True)
        self.fleet.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.fleet.setMaximumHeight(150)
        layout.addWidget(self.fleet)
        layout.addStretch()
        client.telemetry_received.connect(self.update_telemetry)
        client.resource_received.connect(self.resource)

    def resource(self, resource: str, payload: dict) -> None:
        if resource != "vehicles":
            return
        self.fleet.setRowCount(len(payload["items"]))
        for row, vehicle in enumerate(payload["items"]):
            values = (
                vehicle["label"],
                vehicle["flight_mode"],
                f"{vehicle['relative_altitude_m']:.1f}",
                "Live" if vehicle["connected"] else "Lost",
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setToolTip(value)
                self.fleet.setItem(row, column, item)

    def update_telemetry(self, snapshot: dict) -> None:
        self.snapshot = snapshot
        if not snapshot.get("connected"):
            self.values.setText("Vehicle disconnected — telemetry is unavailable")
            for card in (self.altitude, self.speed, self.mode, self.battery):
                card.value.setText("—")
            self.gps_badge.set_state("LINK OFFLINE", "warning")
            return
        messages = snapshot.get("messages", {})
        gps = messages.get("GPS_RAW_INT", {})
        battery = messages.get("SYS_STATUS", {})
        attitude = messages.get("ATTITUDE", {})
        position = messages.get("GLOBAL_POSITION_INT", {})
        speed = math.hypot(position.get("vx", 0), position.get("vy", 0)) / 100
        self.altitude.value.setText(f"{snapshot['relative_altitude_m']:.1f} m")
        self.speed.value.setText(f"{speed:.1f} m/s")
        self.mode.value.setText(snapshot.get("flight_mode", "Unknown"))
        percent = battery.get("battery_remaining", -1)
        self.battery.value.setText(f"{percent}%" if percent >= 0 else "—")
        fixed = gps.get("fix_type", 0) >= 3
        self.gps_badge.set_state(
            f"{'GNSS 3D FIX' if fixed else 'GNSS DEGRADED'}  ·  "
            f"{gps.get('satellites_visible', '—')} satellites",
            "success" if fixed else "warning",
        )
        voltage = battery.get("voltage_battery", 65535)
        current = battery.get("current_battery", -1)
        voltage_text = f"{voltage / 1000:.2f} V" if voltage != 65535 else "Unknown"
        current_text = f"{current / 100:.2f} A" if current >= 0 else "Unknown"
        age = snapshot.get("heartbeat_age_seconds")
        age_text = f"{age:.1f} s" if age is not None else "Unknown"
        self.values.setText(
            f"Vehicle: {snapshot['vehicle_id']}  ·  Armed: {snapshot['armed']}\n"
            f"Latitude: {snapshot['latitude_deg']:.7f}°\n"
            f"Longitude: {snapshot['longitude_deg']:.7f}°\n"
            f"R {math.degrees(attitude.get('roll', 0)):.1f}°  ·  "
            f"P {math.degrees(attitude.get('pitch', 0)):.1f}°  ·  "
            f"Y {math.degrees(attitude.get('yaw', 0)):.1f}°\n"
            f"Vertical {position.get('vz', 0) / 100:.1f} m/s down  ·  "
            f"MSL {position.get('alt', 0) / 1000:.1f} m\n"
            f"Power: {voltage_text}  /  {current_text}\n"
            f"Heartbeat age: {age_text}"
        )

    def export(self) -> None:
        filename, _ = QFileDialog.getSaveFileName(
            self, "Export telemetry snapshot", "telemetry.json", "JSON (*.json)"
        )
        if filename:
            try:
                Path(filename).write_text(
                    json.dumps(self.snapshot, indent=2) + "\n", encoding="utf-8"
                )
            except OSError as error:
                self.values.setText(str(error))
