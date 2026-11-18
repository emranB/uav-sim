"""Compose the desktop workspace and own its presentation resources."""

import os
import signal
import sys
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QApplication, QCheckBox, QSplitter, QTabWidget, QVBoxLayout, QWidget

from common.configuration.runtime import RuntimeSettings
from desktop.flight_actions import FlightActionRow
from desktop.native_recording import NativeCaptureRecording
from desktop.replay_capture import ReplayCaptureRow
from desktop.theme import apply_theme
from desktop.window import MainWindow
from integrations.recording.native_screen import NativeScreenRecording


def arrange_native_capture_views(window: MainWindow) -> None:
    """Keep QGC, Gazebo and the live reconstruction visible in the recording layout."""
    tabs = window.findChild(QTabWidget, "rightWorkspaceTabs")
    world, reconstruction = tabs.widget(0), tabs.widget(3)
    tabs.removeTab(3)
    tabs.removeTab(0)
    combined = QSplitter(Qt.Orientation.Vertical)
    combined.addWidget(world)
    combined.addWidget(reconstruction)
    combined.setSizes([420, 360])
    combined.setChildrenCollapsible(False)
    content = combined
    if os.environ.get("UAV_WATERFRONT_DEMO") == "true":
        from control_panel.sensors.gnss_control import GnssInputControl

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(combined, 1)
        layout.addWidget(GnssInputControl(window.client))
    tabs.insertTab(0, content, "World + ORB map")
    tabs.setCurrentIndex(0)
    world.show()
    reconstruction.show()


def main() -> int:
    environment = dict(os.environ)
    environment.setdefault("UAV_HEADLESS", "false")
    if RuntimeSettings.from_environment(environment).headless:
        return 0
    app = QApplication(sys.argv)
    apply_theme(app)
    window = MainWindow()
    native_recording = None
    if environment.get("UAV_NATIVE_SCREEN_RECORDING") == "true":
        arrange_native_capture_views(window)
        waterfront_demo = environment.get("UAV_WATERFRONT_DEMO") == "true"
        if waterfront_demo:
            # Qt Multimedia is optional and installed only in the demo desktop image.
            from desktop.replay_player import ReplayPlayer

            live_workspace = window.takeCentralWidget()
            workspace = QSplitter(Qt.Orientation.Horizontal)
            workspace.addWidget(live_workspace)
            live_workspace.setSizes([640, 640])
            workspace.addWidget(ReplayPlayer(window.client))
            workspace.setSizes([1280, 640])
            workspace.setChildrenCollapsible(False)
            window.setCentralWidget(workspace)
        if waterfront_demo:
            window.setWindowFlag(Qt.WindowType.FramelessWindowHint, True)
            window.move(0, 0)
            window.setFixedSize(1920, 1200)
        else:
            window.setFixedSize(1600, 1000)
        row = window.findChild(ReplayCaptureRow)
        flight_actions = window.findChild(FlightActionRow)
        reference = QCheckBox("Use stereo map reference for selected mission targets")
        reference.setObjectName("stereoReferenceChoice")
        reference.setToolTip("Snapshot this horizontal reference when Start mission is pressed")
        reference.toggled.connect(
            lambda enabled: setattr(flight_actions, "stereo_horizontal_reference", enabled)
        )
        flight_actions.parentWidget().layout().insertWidget(1, reference)
        native_recording = NativeCaptureRecording(
            window.client,
            Path(environment.get("OUTPUT_DIR", "outputs")),
            lambda: int(window.winId()),
            lambda: (window.width(), window.height()),
            window,
            recorder_factory=lambda outputs, identifier: NativeScreenRecording(
                outputs,
                identifier,
                dimensions=(window.width(), window.height()),
                title="Halifax waterfront | GPS-loss mission walkthrough",
            ),
        )
        row.set_capture_dispatcher(native_recording.execute)
        native_recording.busy_changed.connect(row.set_native_recording_busy)
        native_recording.failure.connect(row.feedback.setText)
        native_recording.failure.connect(window.statusBar().showMessage)
        native_recording.sealed.connect(
            lambda _: window.statusBar().showMessage("Native screen recording sealed", 5000)
        )
    signal.signal(signal.SIGTERM, lambda signum, frame: window.close())
    signal.signal(signal.SIGINT, lambda signum, frame: window.close())
    if environment.get("UAV_WATERFRONT_DEMO") == "true":
        window.showFullScreen()
    else:
        window.show()
    window_file = Path("/runtime/Desktop.window")
    window_file.write_text(str(int(window.winId())), encoding="utf-8")
    if environment.get("UAV_NATIVE_CONTROL_STATE") == "true":
        from testing.simulation.native_control_state import NativeControlState

        _native_control_state = NativeControlState(window)
    if native_recording is None:
        QTimer.singleShot(350, window.showMaximized)
    try:
        return app.exec()
    finally:
        if native_recording is not None:
            native_recording.close()
        window_file.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
