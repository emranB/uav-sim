"""Searchable bounded console logs and persistent recording inventory."""

from pathlib import Path
from typing import TYPE_CHECKING
from urllib.parse import quote

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class LogsPanel(QWidget):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.client = client
        layout = QVBoxLayout(self)
        self.source = QComboBox()
        self.source.addItems(
            [
                "px4",
                "gazebo",
                "gazebo-viewer",
                "qgroundcontrol",
                "application",
                "desktop",
                "environment",
                "camera-streams",
            ]
        )
        self.query = QLineEdit()
        self.query.setPlaceholderText("Search recent log messages")
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMaximumBlockCount(client.preferences["log_rows"])
        self.inventory = QLabel()
        self.inventory.setWordWrap(True)
        for widget in (self.source, self.query, self.output, self.inventory):
            layout.addWidget(widget)
        export = QPushButton("Export visible log text")
        export.clicked.connect(self.export)
        layout.addWidget(export)
        recordings = QPushButton("Refresh flight recordings")
        recordings.clicked.connect(lambda: client.fetch("recordings"))
        layout.addWidget(recordings)
        self.last_status = ""
        client.error_received.connect(self.appendPlainText)
        client.workspace_reset.connect(self.clear)
        client.telemetry_received.connect(self.update_status)
        client.resource_received.connect(self.resource)
        client.preferences_changed.connect(
            lambda settings: self.output.setMaximumBlockCount(settings["log_rows"])
        )
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(2000)
        self.query.returnPressed.connect(self.refresh)
        self.source.currentTextChanged.connect(self.refresh)

    def refresh(self) -> None:
        self.client.fetch(
            f"logs?source={self.source.currentText()}&query={quote(self.query.text())}"
        )

    def resource(self, name: str, payload: dict) -> None:
        if name.startswith("logs?"):
            self.output.setPlainText(
                "\n".join(
                    f"{record.get('timestamp_utc', '')} {record.get('message', str(record))}"
                    for record in payload["records"]
                )
            )
        elif name == "recordings":
            self.inventory.setText(
                f"Telemetry files: {len(payload['telemetry'])}\n"
                f"PX4 ULogs: {len(payload['flight_logs'])}\n"
                "Search shows a recent tail; full recordings remain in outputs."
            )

    def appendPlainText(self, text: str) -> None:
        self.output.appendPlainText(text)

    def toPlainText(self) -> str:
        return self.output.toPlainText()

    def clear(self) -> None:
        self.output.clear()
        self.last_status = ""

    def export(self) -> None:
        filename, _ = QFileDialog.getSaveFileName(
            self, "Export logs", "simulation.log", "Log (*.log)"
        )
        if filename:
            try:
                Path(filename).write_text(self.output.toPlainText() + "\n", encoding="utf-8")
            except OSError as error:
                self.appendPlainText(str(error))

    def update_status(self, snapshot: dict) -> None:
        status = snapshot.get("messages", {}).get("STATUSTEXT", {}).get("text", "")
        if status and status != self.last_status:
            self.appendPlainText(f"{snapshot.get('vehicle_id', 'Unknown vehicle')} / PX4: {status}")
            self.last_status = status
