"""Compact acknowledged GNSS input control with an explicit independent vehicle selector."""

import uuid
from typing import TYPE_CHECKING

from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class GnssInputControl(QWidget):
    """Keep fault input visible without coupling its target to mission selection."""

    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.client = client
        self.pending = ""
        self.targets: tuple[str, ...] = ()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        self.status = QLabel("Choose a GNSS input vehicle explicitly")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        row = QHBoxLayout()
        self.vehicle = QComboBox()
        self.vehicle.setObjectName("gnssTargetSelection")
        self.vehicle.addItem("Choose GNSS vehicle", None)
        self.vehicle.currentIndexChanged.connect(self.selection_changed)
        self.enabled = QCheckBox("Enable GNSS fix")
        self.enabled.setObjectName("gnssEnableInput")
        self.enabled.setChecked(True)
        self.apply = QPushButton("Apply GNSS input")
        self.apply.setObjectName("gnssApplyInput")
        self.apply.setEnabled(False)
        row.addWidget(self.vehicle)
        row.addWidget(self.enabled)
        row.addWidget(self.apply)
        layout.addLayout(row)
        self.apply.clicked.connect(self.send)
        client.resource_received.connect(self.resource)
        client.gnss_operation_result.connect(self.finished)

    def resource(self, name: str, payload: dict) -> None:
        if name not in {"navigation/fleet", "vehicles"}:
            return
        for item in payload.get("items", []):
            if self.vehicle.findData(item["vehicle_id"]) < 0:
                self.vehicle.addItem(item["label"], item["vehicle_id"])

    def selection_changed(self, index: int) -> None:
        identity = self.vehicle.currentData()
        self.status.setText(
            "GNSS input target: " + self.vehicle.currentText()
            if identity
            else "Choose a GNSS input vehicle explicitly"
        )
        self.apply.setEnabled(bool(identity) and not self.pending)

    def send(self) -> None:
        identity = self.vehicle.currentData()
        if self.pending or not identity:
            return
        self.pending = uuid.uuid4().hex
        self.targets = (identity,)
        self.pending_name = self.vehicle.currentText()
        self.enabled.setEnabled(False)
        self.vehicle.setEnabled(False)
        self.apply.setEnabled(False)
        self.status.setText(
            "Applying GNSS input to " + self.pending_name + "; awaiting acknowledgement"
        )
        self.client.execute_gnss_targets(self.targets, self.enabled.isChecked(), self.pending)

    def finished(self, request_id: str, payload: dict) -> None:
        if request_id != self.pending:
            return
        self.pending = ""
        self.enabled.setEnabled(True)
        self.vehicle.setEnabled(True)
        if payload.get("accepted") is True:
            self.status.setText(
                "GNSS input acknowledged: " + self.pending_name + "; check observed badges"
            )
        else:
            self.status.setText("GNSS input failed: " + str(payload.get("error", payload)))
        self.apply.setEnabled(bool(self.vehicle.currentData()))
