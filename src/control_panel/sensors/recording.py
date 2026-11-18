"""Read-only stereo playback; file work stays outside the desktop event loop."""

from pathlib import Path

from PySide6.QtCore import QObject, QRunnable, Qt, QThreadPool, QTimer, Signal
from PySide6.QtGui import QCloseEvent, QImage, QPixmap, QResizeEvent
from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from common.logging.stereo import StereoRecording


class _FrameSignals(QObject):
    loaded = Signal(int, object, object, object)
    failed = Signal(int, str)


class _ReadFrame(QRunnable):
    def __init__(
        self,
        path: Path,
        index: int,
        generation: int,
        recording: StereoRecording | None = None,
    ) -> None:
        super().__init__()
        self.path, self.index, self.generation = path, index, generation
        self.recording = recording
        self.signals = _FrameSignals()

    def run(self) -> None:
        try:
            recording = self.recording or StereoRecording.open(self.path)
            left, right = recording.images(self.index)
            images = [QImage.fromData(contents) for contents in (left, right)]
            frame = recording.frames[self.index]
            if any(
                image.isNull() or image.width() != frame.width or image.height() != frame.height
                for image in images
            ):
                raise ValueError("Recorded image does not match its declared dimensions")
            self.signals.loaded.emit(self.generation, recording, images[0], images[1])
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.signals.failed.emit(self.generation, str(error))


class StereoRecordingDialog(QDialog):
    """Replay real captures without issuing vehicle commands or implying live imagery."""

    frame_presented = Signal(int)
    load_failed = Signal(str)

    def __init__(self, path: Path, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Stereo recording - read-only playback")
        self.resize(850, 480)
        self.path = path
        self._generation = 0
        self._loading = False
        self._closed = False
        self._playing = False
        self._recording: StereoRecording | None = None
        self._images: tuple[QImage, QImage] | None = None
        layout = QVBoxLayout(self)
        self.status = QLabel("Loading recorded stereo images. This is not a live camera feed.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        views = QHBoxLayout()
        self.views = []
        self.view_labels = []
        for side in ("Left camera", "Right camera"):
            column = QVBoxLayout()
            label = QLabel(side)
            column.addWidget(label)
            self.view_labels.append(label)
            image = QLabel()
            image.setAlignment(Qt.AlignmentFlag.AlignCenter)
            image.setMinimumSize(160, 120)
            column.addWidget(image, 1)
            views.addLayout(column)
            self.views.append(image)
        layout.addLayout(views, 1)
        self.position = QSlider(Qt.Orientation.Horizontal)
        self.position.setAccessibleName("Recorded stereo frame")
        self.position.setEnabled(False)
        self.position.valueChanged.connect(self.load)
        layout.addWidget(self.position)
        row = QHBoxLayout()
        self.play = QPushButton("Play")
        self.play.setEnabled(False)
        self.play.clicked.connect(self.toggle_play)
        row.addWidget(self.play)
        close = QPushButton("Close")
        close.clicked.connect(self.close)
        row.addWidget(close)
        layout.addLayout(row)
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.advance)
        self.load(0)

    def load(self, index: int) -> None:
        self._generation += 1
        self._loading = True
        self.position.setEnabled(False)
        self.play.setEnabled(False)
        task = _ReadFrame(self.path, index, self._generation, self._recording)
        task.signals.loaded.connect(self.loaded)
        task.signals.failed.connect(self.failed)
        QThreadPool.globalInstance().start(task)

    def loaded(
        self,
        generation: int,
        recording: StereoRecording,
        left: QImage,
        right: QImage,
    ) -> None:
        if self._closed or generation != self._generation:
            return
        self._loading = False
        self._recording = recording
        self._images = left, right
        for label, title in zip(self.view_labels, recording.view_labels):
            label.setText(title)
        self.position.setMaximum(len(recording.frames) - 1)
        self.position.setEnabled(True)
        self.play.setEnabled(len(recording.frames) > 1)
        frame = recording.frames[self.position.value()]
        self.status.setText(
            f"RECORDED | Frame {self.position.value() + 1}/{len(recording.frames)} | "
            f"Simulation time {frame.simulation_timestamp_ns / 1e9:.3f} s | "
            f"{recording.vehicle_id or 'camera rig'} | {recording.source}; read-only playback."
        )
        self.render()
        self.schedule()
        self.frame_presented.emit(self.position.value())

    def failed(self, generation: int, message: str) -> None:
        if self._closed or generation != self._generation:
            return
        self._loading = False
        self._playing = False
        self.timer.stop()
        self.play.setText("Play")
        self._images = None
        for view in self.views:
            view.clear()
        self.status.setText(f"Cannot read stereo recording: {message}")
        self.load_failed.emit(message)

    def render(self) -> None:
        if self._images:
            for view, image in zip(self.views, self._images):
                view.setPixmap(
                    QPixmap.fromImage(image).scaled(
                        view.size(),
                        Qt.AspectRatioMode.KeepAspectRatio,
                        Qt.TransformationMode.SmoothTransformation,
                    )
                )

    def resizeEvent(self, event: QResizeEvent) -> None:
        super().resizeEvent(event)
        self.render()

    def toggle_play(self) -> None:
        self._playing = not self._playing
        self.play.setText("Pause" if self._playing else "Play")
        if self._playing and self.position.value() == self.position.maximum():
            self.position.setValue(0)
        else:
            self.schedule()

    def schedule(self) -> None:
        self.timer.stop()
        if not self._playing or self._loading or self._recording is None:
            return
        index = self.position.value()
        if index == self.position.maximum():
            self._playing = False
            self.play.setText("Play")
            return
        frames = self._recording.frames
        delay_ms = (
            frames[index + 1].simulation_timestamp_ns - frames[index].simulation_timestamp_ns
        ) // 1_000_000
        self.timer.start(max(1, min(5000, delay_ms)))

    def advance(self) -> None:
        self.position.setValue(min(self.position.value() + 1, self.position.maximum()))

    def closeEvent(self, event: QCloseEvent) -> None:
        self._closed = True
        self.timer.stop()
        super().closeEvent(event)

    def done(self, result: int) -> None:
        """Escape and dialog rejection must stop playback just like the Close button."""
        self._closed = True
        self.timer.stop()
        super().done(result)
