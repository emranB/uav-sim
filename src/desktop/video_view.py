"""Display decoded replay frames with portable Qt raster painting."""

from PySide6.QtCore import QPoint, QRect, Qt, Slot
from PySide6.QtGui import QColor, QImage, QPainter, QPaintEvent
from PySide6.QtMultimedia import QVideoFrame
from PySide6.QtWidgets import QWidget


class RecordedVideoView(QWidget):
    """Paint actual decoded images without requiring an OpenGL video surface."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._image = QImage()

    @Slot(QVideoFrame)
    def present_frame(self, frame: QVideoFrame) -> None:
        """Receive decoded media on the GUI thread; preserve the last valid frame at EOF."""
        image = frame.toImage()
        if not image.isNull():
            self._image = image
            self.update()

    def clear_frame(self) -> None:
        """Remove the previous run's image before a new source starts decoding."""
        self._image = QImage()
        self.update()

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#0b1522"))
        if not self._image.isNull():
            size = self._image.size().scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio)
            origin = QPoint(
                (self.width() - size.width()) // 2, (self.height() - size.height()) // 2
            )
            painter.drawImage(QRect(origin, size), self._image)
