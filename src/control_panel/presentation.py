"""Shared visual components for operational state and telemetry values."""

from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout


class StatusBadge(QLabel):
    def __init__(self, text: str = "Connecting") -> None:
        super().__init__(text)
        self.setObjectName("statusBadge")

    def set_state(self, text: str, state: str) -> None:
        if self.text() == text and self.property("state") == state:
            return
        self.setText(text)
        self.setProperty("state", state)
        self.style().unpolish(self)
        self.style().polish(self)


class MetricCard(QFrame):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.setObjectName("metricCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        caption = QLabel(title)
        caption.setObjectName("metricCaption")
        self.value = QLabel("—")
        self.value.setObjectName("metricValue")
        layout.addWidget(caption)
        layout.addWidget(self.value)
