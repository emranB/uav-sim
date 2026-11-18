"""Common presentation of asynchronous panel operations and feedback."""

from collections.abc import Callable
from typing import TYPE_CHECKING

from PySide6.QtWidgets import QLabel, QPushButton, QVBoxLayout, QWidget

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class PanelView(QWidget):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.client = client
        self.form = QVBoxLayout(self)
        self.form.setContentsMargins(12, 12, 12, 12)
        self.form.setSpacing(8)
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setObjectName("operationFeedback")
        self.status.setAccessibleName("Operation feedback")
        self.status.hide()
        self._busy_buttons: list[QPushButton] = []
        self.pending: str | None = None
        client.operation_result.connect(self.acknowledged)
        client.operation_failed.connect(self.failed)
        client.workspace_reset.connect(self.reset_panel)
        client.selection_changed.connect(lambda vehicle_id: self.reset_panel())

    def button(
        self, title: str, callback: Callable, layout: QVBoxLayout | None = None
    ) -> QPushButton:
        button = QPushButton(title)
        if title.startswith(("Apply", "Save", "Execute", "Restore")):
            button.setProperty("role", "primary")
        button.clicked.connect(callback)
        (layout if layout is not None else self.form).addWidget(button)
        return button

    def finish(self) -> None:
        self.button("Reset panel edits", self.reset_panel)
        self.form.addStretch()
        self.form.insertWidget(0, self.status)

    def reset_panel(self) -> None:
        self.pending = None
        self._restore_buttons()
        self.status.clear()
        self.status.hide()
        self.client.fetch("workspace")

    def send(self, operation: str, body: dict | None = None) -> None:
        if self.pending:
            self._feedback("Wait for the current operation to finish", "pending")
            return
        self.pending = operation
        self._busy_buttons = [
            button for button in self.findChildren(QPushButton) if button.isEnabled()
        ]
        for button in self._busy_buttons:
            button.setEnabled(False)
        self._feedback("Applying changes... Waiting for acknowledgement.", "pending")
        self.client.execute(operation, body)

    def acknowledged(self, operation: str, result: dict) -> None:
        if operation == self.pending:
            self.pending = None
            targets = result.get("results")
            if operation == "perception/check":
                message = (
                    "Capability implemented; fresh observations are ready."
                    if result.get("ready")
                    else "Capability implemented; waiting for usable observations."
                )
            elif result.get("state") == "starting":
                message = "Vehicle created — waiting for telemetry readiness"
            elif targets:
                message = f"Acknowledged by {len(targets)} vehicles"
            else:
                message = "Applied and acknowledged"
            self._restore_buttons()
            self._feedback(message, "success")

    def failed(self, operation: str, message: str) -> None:
        if self.pending == operation:
            self.pending = None
            self._restore_buttons()
            self._feedback(message, "error")

    def _restore_buttons(self) -> None:
        for button in self._busy_buttons:
            button.setEnabled(True)
        self._busy_buttons.clear()

    def _feedback(self, message: str, state: str) -> None:
        self.status.setText(message)
        self.status.setProperty("state", state)
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)
        self.status.show()
