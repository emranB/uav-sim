"""Visible whole-UAV restore scope and server-owned recovery progress."""

import time
import uuid
from typing import TYPE_CHECKING

from PySide6.QtCore import QTimer
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class RestoreControl(QFrame):
    """Request safe recovery; never imply success from a button press or process liveness."""

    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.client = client
        self.pending_id = ""
        self.last_completed_id = None
        self.last_received = 0.0
        self.availability = {"available": False, "reason": "Waiting for service owners"}
        self.status = {"state": "idle", "active": False}
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.button = QPushButton("Master Reset UAV")
        self.button.setObjectName("masterResetUav")
        self.feedback = QLabel("Restore unavailable")
        self.cancel_button = QPushButton("Cancel restore")
        self.cancel_button.setToolTip(
            "Finish acknowledged safe landing or restart before cancelling"
        )
        self.feedback.setMaximumWidth(180)
        self.feedback.setMinimumWidth(160)
        self.feedback.setWordWrap(True)
        layout.addWidget(self.button)
        layout.addWidget(self.cancel_button)
        layout.addWidget(self.feedback)
        self.button.clicked.connect(self.dispatch)
        self.cancel_button.clicked.connect(lambda: self.dispatch(cancel=True))
        client.resource_received.connect(self.receive)
        client.restore_operation_result.connect(self.finished)
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.poll)
        self.timer.start()
        self.refresh()

    def poll(self) -> None:
        self.client.fetch("restore")
        self.refresh()

    def receive(self, resource: str, payload: dict) -> None:
        if resource != "restore":
            return
        self.status = payload["restore"]
        self.availability = payload["availability"]
        self.last_received = time.monotonic()
        identifier = self.status.get("restore_id")
        if self.status["state"] == "completed" and identifier != self.last_completed_id:
            self.last_completed_id = identifier
            self.client.restore_completed.emit(self.status)
        self.refresh()

    def refresh(self) -> None:
        stale = time.monotonic() - self.last_received > 3
        active = self.status.get("active", False)
        self.button.setEnabled(
            not stale
            and not active
            and not self.pending_id
            and self.availability.get("available", False)
        )
        self.cancel_button.setVisible(active)
        self.cancel_button.setEnabled(active and not self.pending_id)
        fleet = ", ".join(
            self.status.get("vehicle_ids") or self.availability.get("vehicle_ids", [])
        )
        reason = (
            self.status.get("error")
            or self.availability.get("reason")
            or (
                "Safely land the affected fleet, finalize capture, restart owned UAV services, "
                "restore runtime defaults and empty targets. Saved records are retained."
            )
        )
        self.button.setToolTip(
            f"Affected fleet: {fleet or 'unknown'}\n{reason}\nUAV only; no host/Docker/other-app reset."
        )
        self.feedback.setText(
            "Restore status unknown"
            if stale
            else f"UAV restore: {self.status['state'].replace('_', ' ')}"
            if active
            else self.status.get("error")
            or (
                "Runtime defaults restored; records retained"
                if self.status["state"] == "completed"
                else "UAV services only; all fleet vehicles"
            )
        )
        self.feedback.setVisible(active or stale or bool(self.status.get("error")))
        self.feedback.setToolTip(reason)

    def dispatch(self, *, cancel: bool = False) -> None:
        if not (self.cancel_button.isEnabled() if cancel else self.button.isEnabled()):
            return
        self.pending_id = uuid.uuid4().hex
        self.feedback.setText("Waiting for restore admission…")
        self.refresh()
        if cancel:
            self.client.execute_restore(self.pending_id, cancel=True)
        else:
            self.client.execute_restore(self.pending_id)

    def finished(self, request_id: str, payload: dict) -> None:
        if request_id != self.pending_id:
            return
        self.pending_id = ""
        if payload.get("accepted"):
            self.status = payload
            self.last_received = time.monotonic()
        else:
            self.status = {"state": "error", "active": False, "error": payload.get("error")}
        self.refresh()

    def closeEvent(self, event: QCloseEvent) -> None:
        self.timer.stop()
        super().closeEvent(event)
