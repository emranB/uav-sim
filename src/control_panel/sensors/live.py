"""Focused-vehicle camera previews with explicit stale and unavailable states."""

from typing import TYPE_CHECKING

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class LiveStereoView(QWidget):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.client = client
        self._ready = False
        self._vehicle_id = "viewed vehicle"
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.status = QLabel("Stereo camera streams are unavailable.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        row = QHBoxLayout()
        self.views: dict[str, QLabel] = {}
        for side in ("left", "right"):
            column = QVBoxLayout()
            column.addWidget(QLabel(f"{side.title()} camera"))
            view = QLabel(f"{side.title()} camera")
            view.setAlignment(Qt.AlignmentFlag.AlignCenter)
            view.setMinimumSize(120, 140)
            view.setMaximumHeight(180)
            view.setToolTip(
                "Latest-frame preview. Recorded pairs retain exact shared simulation timestamps."
            )
            column.addWidget(view, 1)
            row.addLayout(column, 1)
            self.views[side] = view
        layout.addLayout(row)
        client.resource_received.connect(self.resource)
        client.selection_changed.connect(self.clear)
        client.workspace_reset.connect(lambda: self.clear(self._vehicle_id))
        client.camera_image_received.connect(self.image)
        client.camera_image_failed.connect(self.failed)
        self.timer = QTimer(self)
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    def clear(self, vehicle_id: str) -> None:
        self._vehicle_id = vehicle_id
        self._ready = False
        self.status.setText(f"Waiting for {vehicle_id}'s camera stream.")
        for side, view in self.views.items():
            view.setText(f"{side.title()} camera unavailable")

    def resource(self, name: str, payload: dict) -> None:
        if name != "perception":
            return
        camera = payload.get("camera", {})
        self._vehicle_id = payload["vehicle_id"]
        self._ready = camera.get("ready", False)
        if self._ready:
            self.status.setText(
                f"LIVE | {payload['vehicle_id']} | "
                f"Metadata sim time {camera['simulation_timestamp_ns'] / 1e9:.2f} s | "
                f"Recorded pairs {camera['recorded_pairs']}"
                + (" | Recording limit reached" if camera.get("recording_full") else "")
            )
        else:
            self.clear(payload["vehicle_id"])
            self.status.setText("Camera unavailable or stale. Enable the inspection deployment.")

    def refresh(self) -> None:
        if self._ready and self.isVisible():
            for side in self.views:
                self.client.fetch_camera_image(side)

    def image(self, side: str, contents: bytes) -> None:
        if not self._ready or side not in self.views:
            return
        image = QImage.fromData(contents)
        if image.isNull():
            self.failed(side, "Camera image could not be decoded")
            return
        view = self.views[side]
        view.setPixmap(
            QPixmap.fromImage(image).scaled(
                view.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def failed(self, side: str, message: str) -> None:
        if side in self.views:
            self.views[side].setText(f"{side.title()} camera unavailable")
            self.status.setText(message)
