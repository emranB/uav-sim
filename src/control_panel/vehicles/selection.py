"""Vehicle inspection focus; command scope belongs to the mission-map selector."""

from typing import TYPE_CHECKING

from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QWidget

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class VehicleSelection(QWidget):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.client = client
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        row = QHBoxLayout()
        row.addWidget(QLabel("Inspect"))
        self.focus = QComboBox()
        self.focus.setMinimumWidth(170)
        self.focus.setAccessibleName("Viewed vehicle")
        self.focus.setToolTip("Select the vehicle shown in telemetry, camera and saved files.")
        row.addWidget(self.focus)
        layout.addLayout(row)
        self.focus.currentIndexChanged.connect(self.changed)
        client.resource_received.connect(self.resource)
        client.selection_changed.connect(self.focus_changed)

    def focus_changed(self, vehicle_id: str) -> None:
        self.focus.blockSignals(True)
        self.focus.setCurrentIndex(self.focus.findData(vehicle_id))
        self.focus.blockSignals(False)

    def changed(self) -> None:
        if self.focus.currentData() is None:
            return
        self.client.select_vehicle(self.focus.currentData())

    def resource(self, resource: str, payload: dict) -> None:
        if resource != "vehicles":
            return
        selected = self.focus.currentData()
        self.focus.blockSignals(True)
        self.focus.clear()
        for vehicle in payload["items"]:
            state = "Flying" if vehicle["armed"] else "Ready" if vehicle["ready"] else "Waiting"
            self.focus.addItem(f"{vehicle['label']} · {state}", vehicle["vehicle_id"])
        self.focus.setCurrentIndex(max(0, self.focus.findData(selected)))
        self.focus.blockSignals(False)
