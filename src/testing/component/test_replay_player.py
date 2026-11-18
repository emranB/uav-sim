"""A live capture never becomes a playable Blender video before rendering succeeds."""

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication

from desktop.replay_player import ReplayPlayer


class PlaybackClient(QObject):
    resource_received = Signal(str, dict)


def test_player_requires_matching_completed_capture() -> None:
    _app = QApplication.instance() or QApplication([])
    client = PlaybackClient()
    player = ReplayPlayer(client)
    try:
        captures = [
            {"capture_id": "capture-a", "retained": True, "vehicle_names": {"vehicle_1": "Atlas"}},
            {"capture_id": "capture-b", "retained": True, "vehicle_names": {"vehicle_2": "Boreal"}},
            {"capture_id": "capture-unsaved", "retained": False},
            {"capture_id": "capture-interrupted", "retained": True, "vehicle_names": None},
        ]
        client.resource_received.emit(
            "replay",
            {
                "items": captures,
                "offline_job": {"capture_id": "capture-a", "state": "rendering", "progress": 0.5},
            },
        )
        assert not player.play.isEnabled() and "50%" in player.status.text()
        assert player.runs.count() == 2
        client.resource_received.emit(
            "replay",
            {
                "items": captures,
                "offline_job": {
                    "capture_id": "capture-a",
                    "state": "completed",
                    "video_path": "/missing/test.mp4",
                },
            },
        )
        assert player.play.isEnabled()
        player.runs.setCurrentIndex(1)
        assert not player.play.isEnabled()
        player.runs.setCurrentIndex(0)
        player.play.click()
        assert "unavailable" in player.status.text()
        assert player.pages.currentWidget() == player.placeholder
    finally:
        player.close()
