"""Recorded-run selection and local playback, separate from live simulator views."""

from pathlib import Path

from PySide6.QtCore import Qt, QUrl
from PySide6.QtGui import QCloseEvent
from PySide6.QtMultimedia import QAudioOutput, QMediaPlayer, QVideoSink
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from desktop.client import SimulationClient
from desktop.video_view import RecordedVideoView


class ReplayPlayer(QWidget):
    """Play an acknowledged completed render; never imply that Blender is live."""

    def __init__(self, client: SimulationClient) -> None:
        super().__init__()
        self.setObjectName("recordedReplayPlayer")
        self.completed: dict[str, dict] = {}
        layout = QVBoxLayout(self)
        heading = QLabel("RECORDED MISSION REPLAY")
        heading.setObjectName("sectionTitle")
        layout.addWidget(heading)
        explanation = QLabel(
            "Blender views are generated after capture. Live controls remain on the left."
        )
        explanation.setWordWrap(True)
        layout.addWidget(explanation)
        self.runs = QComboBox()
        self.runs.setObjectName("recordedRunSelection")
        self.runs.setPlaceholderText("Saved missions appear here")
        self.runs.currentIndexChanged.connect(self.selection_changed)
        layout.addWidget(self.runs)
        actions = QHBoxLayout()
        self.play = QPushButton("Play replay")
        self.play.setObjectName("playRecordedReplay")
        self.play.setEnabled(False)
        self.play.clicked.connect(self.start_playback)
        stop = QPushButton("Stop replay")
        stop.clicked.connect(self.stop_playback)
        actions.addWidget(self.play)
        actions.addWidget(stop)
        layout.addLayout(actions)
        self.status = QLabel(
            "No replay video yet. Capture a mission, save it, then wait for rendering."
        )
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.pages = QStackedWidget()
        self.placeholder = QWidget()
        placeholders = QVBoxLayout(self.placeholder)
        placeholders.setContentsMargins(0, 0, 0, 0)
        for name, colour in (("ATLAS", "#39d9b1"), ("BOREAL", "#62a5ff")):
            frame = QFrame()
            frame.setStyleSheet(
                "background: #0b1522; border: 1px solid #2b4058; border-radius: 6px;"
            )
            card = QVBoxLayout(frame)
            title = QLabel(name + " · RECORDED VIEW")
            title.setStyleSheet(f"color: {colour}; border: none; padding: 12px; font-weight: bold;")
            card.addWidget(title)
            message = QLabel(
                "No replay video yet\n\nAvailable after saving and rendering this mission"
            )
            message.setAlignment(Qt.AlignmentFlag.AlignCenter)
            message.setWordWrap(True)
            message.setStyleSheet("color: #b8ccdd; border: none; padding: 16px;")
            card.addWidget(message, 1)
            placeholders.addWidget(frame, 1)
        self.video = RecordedVideoView()
        self.pages.addWidget(self.placeholder)
        self.pages.addWidget(self.video)
        layout.addWidget(self.pages, 1)
        credit = QLabel(
            "Terrain: Service Nova Scotia · Buildings: Halifax Regional Municipality\n"
            "Simplified building geometry and synthetic materials"
        )
        credit.setWordWrap(True)
        layout.addWidget(credit)
        self.player = QMediaPlayer(self)
        self.audio = QAudioOutput(self)
        self.audio.setMuted(True)
        self.player.setAudioOutput(self.audio)
        self.video_sink = QVideoSink(self.video)
        self.video_sink.videoFrameChanged.connect(self.video.present_frame)
        self.player.setVideoSink(self.video_sink)
        self.player.errorOccurred.connect(
            lambda error, message: self.status.setText("Replay playback failed: " + message)
        )
        client.resource_received.connect(self.resource)

    def resource(self, name: str, payload: dict) -> None:
        if name != "replay":
            return
        for capture in payload.get("items", []):
            if capture.get("retained") is not True or not capture.get("vehicle_names"):
                continue
            identifier = capture["capture_id"]
            if self.runs.findData(identifier) < 0:
                names = ", ".join(capture.get("vehicle_names", {}).values())
                self.runs.addItem(f"{names} · {identifier[-8:]}", identifier)
        if self.runs.currentIndex() < 0 and self.runs.count():
            self.runs.setCurrentIndex(0)
        job = payload.get("offline_job", {})
        if job.get("state") == "completed" and job.get("video_path"):
            self.completed[job["capture_id"]] = job
        if job.get("capture_id") == self.runs.currentData():
            state = job.get("state")
            if state in {"queued", "running", "rendering"}:
                self.status.setText(f"Rendering recorded run · {job.get('progress', 0):.0%}")
            elif state in {"error", "cancelled"}:
                self.status.setText("Replay " + state + ": " + str(job.get("error") or ""))
            elif state == "completed":
                self.status.setText("Replay ready · recorded poses and sensor evidence")
        self.play.setEnabled(self.runs.currentData() in self.completed)

    def selection_changed(self, index: int) -> None:
        self.stop_playback()
        available = self.runs.currentData() in self.completed
        self.play.setEnabled(available)
        self.status.setText(
            "Replay ready" if available else "No rendered video for this saved run yet"
        )

    def start_playback(self) -> None:
        job = self.completed.get(self.runs.currentData())
        if job is None:
            return
        path = Path(job["video_path"])
        if not path.is_file():
            self.status.setText("Rendered video file is unavailable: " + str(path))
            return
        self.video.clear_frame()
        self.player.setSource(QUrl.fromLocalFile(str(path)))
        self.pages.setCurrentWidget(self.video)
        self.player.play()

    def stop_playback(self) -> None:
        if hasattr(self, "player"):
            self.player.stop()
        if hasattr(self, "pages"):
            self.pages.setCurrentWidget(self.placeholder)

    def closeEvent(self, event: QCloseEvent) -> None:
        self.player.stop()
        super().closeEvent(event)
