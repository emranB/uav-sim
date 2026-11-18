"""Interactive, read-only presentation of estimated camera trajectories and landmarks."""

import math
from pathlib import Path

from PySide6.QtCore import QObject, QPointF, QRunnable, Qt, QThreadPool, Signal
from PySide6.QtGui import QColor, QPainter, QPaintEvent, QPen, QPolygonF
from PySide6.QtWidgets import QDialog, QLabel, QSlider, QVBoxLayout, QWidget

from common.logging.mapping import RecordedMap


class MapCanvas(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(400, 280)
        self.mapping: RecordedMap | None = None
        self.surfaces: tuple = ()
        self.trajectory_label = "recorded camera trajectory"
        self.dimmed = False
        self.azimuth_deg = 35
        self.elevation_deg = 25

    def set_map(self, mapping: RecordedMap) -> None:
        self.mapping = mapping
        self.update()

    def set_azimuth(self, degrees: int) -> None:
        self.azimuth_deg = degrees
        self.update()

    def set_elevation(self, degrees: int) -> None:
        self.elevation_deg = degrees
        self.update()

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#0b1522"))
        if self.dimmed:
            painter.setOpacity(0.45)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        if self.mapping is None:
            painter.setPen(QColor("#b8ccdd"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Loading estimated map...")
            return
        # Recent stereo surfaces set the live viewport; distant sparse outliers must
        # not shrink the useful reconstruction to a few pixels.
        points = (self.surfaces or self.mapping.landmarks) + self.mapping.trajectory + ((0, 0, 0),)
        bounds = [
            (min(point[axis] for point in points), max(point[axis] for point in points))
            for axis in range(3)
        ]
        center = [(low + high) / 2 for low, high in bounds]
        span = max(0.1, *(high - low for low, high in bounds))
        scale = min(self.width(), self.height()) * 0.7 / span
        yaw, pitch = math.radians(self.azimuth_deg), math.radians(self.elevation_deg)

        def project(point: tuple) -> QPointF:
            x, y, z = (point[index] - center[index] for index in range(3))
            horizontal = math.cos(yaw) * x - math.sin(yaw) * z
            depth = math.sin(yaw) * x + math.cos(yaw) * z
            vertical = math.cos(pitch) * y - math.sin(pitch) * depth
            return QPointF(
                self.width() / 2 + horizontal * scale, self.height() / 2 + vertical * scale
            )

        painter.setPen(QPen(QColor("#d6aa69"), 2))
        for point in self.surfaces:
            painter.drawPoint(project(point))
        painter.setPen(QPen(QColor("#6fb9dc"), 2))
        for point in self.mapping.landmarks:
            painter.drawPoint(project(point))
        painter.setPen(QPen(QColor("#7ee2bd"), 2))
        if self.mapping.trajectory:
            painter.drawPolyline(QPolygonF([project(point) for point in self.mapping.trajectory]))
        for axis, color, title in ((0, "#ef8c94", "X"), (1, "#88d5a4", "Y"), (2, "#86bfff", "Z")):
            endpoint = [0.0, 0.0, 0.0]
            endpoint[axis] = min(1.0, span / 3)
            painter.setPen(QPen(QColor(color), 2))
            painter.drawLine(project((0, 0, 0)), project(tuple(endpoint)))
            painter.drawText(project(tuple(endpoint)), title)
        painter.setPen(QColor("#b8ccdd"))
        painter.drawText(
            12, self.height() - 14, f"Blue: landmarks | Green: {self.trajectory_label}"
        )


class _MapSignals(QObject):
    loaded = Signal(object)
    failed = Signal(str)


class _ReadMap(QRunnable):
    def __init__(self, path: Path) -> None:
        super().__init__()
        self.path = path
        self.signals = _MapSignals()

    def run(self) -> None:
        try:
            self.signals.loaded.emit(RecordedMap.open(self.path))
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.signals.failed.emit(str(error))


class RecordedMapDialog(QDialog):
    map_presented = Signal()
    load_failed = Signal(str)

    def __init__(self, path: Path, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Recorded visual mapping - read-only")
        self.resize(850, 620)
        self._closed = False
        layout = QVBoxLayout(self)
        self.status = QLabel("Loading estimated map...")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addWidget(QLabel("Local camera-map coordinates in metres; no global positioning."))
        self.canvas = MapCanvas()
        layout.addWidget(self.canvas, 1)
        for title, limits, value, callback in (
            ("Rotate map", (-180, 180), 35, self.canvas.set_azimuth),
            ("Tilt map", (-89, 89), 25, self.canvas.set_elevation),
        ):
            layout.addWidget(QLabel(title))
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(*limits)
            slider.setValue(value)
            slider.valueChanged.connect(callback)
            layout.addWidget(slider)
        task = _ReadMap(path)
        task.signals.loaded.connect(self.loaded)
        task.signals.failed.connect(self.failed)
        QThreadPool.globalInstance().start(task)

    def loaded(self, mapping: RecordedMap) -> None:
        if not self._closed:
            self.canvas.set_map(mapping)
            self.status.setText(mapping.summary)
            self.map_presented.emit()

    def failed(self, message: str) -> None:
        if not self._closed:
            self.status.setText(f"Cannot read recorded map: {message}")
            self.load_failed.emit(message)

    def done(self, result: int) -> None:
        self._closed = True
        super().done(result)
