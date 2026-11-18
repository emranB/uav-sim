"""Read-only capability inventory with an explicit executable availability check."""

from collections.abc import Callable
from typing import TYPE_CHECKING

from PySide6.QtWidgets import QCheckBox, QComboBox, QGroupBox, QLabel, QPushButton, QVBoxLayout

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class PerceptionCapabilities(QGroupBox):
    """Never present an unimplemented sensor pipeline as an enabled vehicle control."""

    def __init__(
        self,
        client: "SimulationClient",
        send: Callable[[str, dict | None], None],
    ) -> None:
        super().__init__("Perception capabilities")
        self.client = client
        self.form = QVBoxLayout(self)
        self.summary = QLabel("Waiting for the viewed vehicle's capability inventory.")
        self.summary.setWordWrap(True)
        self.form.addWidget(self.summary)
        self.features = QComboBox()
        self.features.setAccessibleName("Perception capability to inspect")
        self.form.addWidget(self.features)
        self.available = QCheckBox("Implemented for this vehicle")
        self.available.setEnabled(False)
        self.form.addWidget(self.available)
        self.reason = QLabel()
        self.reason.setWordWrap(True)
        self.form.addWidget(self.reason)
        self.check = QPushButton("Check selected capability")
        self.check.setEnabled(False)
        self.check.setToolTip(
            "Check availability without enabling sensors or changing flight state."
        )
        self.check.clicked.connect(
            lambda: send("perception/check", {"capability_id": self.features.currentData()})
        )
        self.form.addWidget(self.check)
        self._items: dict[str, dict] = {}
        self.features.currentIndexChanged.connect(self.describe)
        client.resource_received.connect(self.resource)
        client.selection_changed.connect(self.clear)
        client.fetch("perception")

    def clear(self, vehicle_id: str) -> None:
        self.features.clear()
        self._items.clear()
        self.available.setChecked(False)
        self.check.setEnabled(False)
        self.reason.clear()
        self.summary.setText(f"Waiting for {vehicle_id}'s capability inventory.")
        self.client.fetch("perception")

    def resource(self, name: str, payload: dict) -> None:
        if name != "perception":
            return
        selected = self.features.currentData()
        self.features.blockSignals(True)
        self.features.clear()
        self._items = {item["key"]: item for item in payload["items"]}
        for key, item in self._items.items():
            state = "Implemented" if item["implemented"] else "Not implemented"
            self.features.addItem(f"{item['title']} - {state}", key)
        self.features.setCurrentIndex(max(0, self.features.findData(selected)))
        self.features.blockSignals(False)
        self.check.setEnabled(bool(self._items))
        self.summary.setText(
            f"Vehicle: {payload['vehicle_id']}. Availability does not imply runtime readiness."
        )
        self.describe()

    def describe(self) -> None:
        item = self._items.get(self.features.currentData())
        self.available.setChecked(bool(item and item["implemented"]))
        self.reason.setText(item["reason"] if item else "No capability selected.")
