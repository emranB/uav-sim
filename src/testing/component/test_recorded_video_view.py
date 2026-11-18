"""Decoded replay frames remain visible without a graphics rendering context."""

from PySide6.QtGui import QColor, QImage
from PySide6.QtMultimedia import QVideoFrame
from PySide6.QtWidgets import QApplication

from desktop.video_view import RecordedVideoView


def test_raster_video_preserves_aspect_and_clears_the_previous_run() -> None:
    app = QApplication.instance() or QApplication([])
    view = RecordedVideoView()
    try:
        view.resize(320, 320)
        view.show()
        image = QImage(640, 320, QImage.Format.Format_RGB32)
        image.fill(QColor("red"))
        view.present_frame(QVideoFrame(image))
        app.processEvents()
        painted = view.grab().toImage()
        assert painted.pixelColor(160, 160) == QColor("red")
        assert painted.pixelColor(160, 20) == QColor("#0b1522")
        view.clear_frame()
        app.processEvents()
        assert view.grab().toImage().pixelColor(160, 160) == QColor("#0b1522")
    finally:
        view.close()
