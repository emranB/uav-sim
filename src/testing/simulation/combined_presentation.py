"""Native qualification fixture: real application widgets and compact sensor intent."""

import argparse
import json
import signal
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QTimer, Slot
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTabWidget,
)

from desktop.main import arrange_native_capture_views
from desktop.replay_capture import ReplayCaptureRow
from desktop.theme import apply_theme
from desktop.viewer import ViewerPanel
from desktop.window import MainWindow
from testing.simulation.visual_presentation import PresentationHarness


@dataclass(frozen=True)
class GnssIntent:
    request_id: str
    target_ids: tuple[str, ...]
    enabled: bool


class CombinedPresentationHarness(PresentationHarness):
    """Drive the visible widgets; the normal asynchronous client owns every command."""

    def __init__(self, window: MainWindow, root: Path) -> None:
        self.replay_pending = {}
        self.gnss_request = None
        self.gnss_pending: GnssIntent | None = None
        super().__init__(window, root)
        self.capture_row = window.findChild(ReplayCaptureRow)
        self.gnss_frame = QFrame()
        layout = QHBoxLayout(self.gnss_frame)
        layout.setContentsMargins(10, 4, 10, 4)
        self.gnss_enabled = QCheckBox("Request GNSS fix")
        self.gnss_enabled.setChecked(True)
        self.gnss_apply = QPushButton("Apply GNSS to targets")
        self.stereo_reference = QCheckBox("Stereo map reference")
        self.stereo_reference.toggled.connect(
            lambda checked: setattr(self.row, "stereo_horizontal_reference", checked)
        )
        self.gnss_feedback = QLabel("Input request and measured fusion remain separate")
        self.gnss_feedback.setWordWrap(True)
        layout.addWidget(self.gnss_enabled)
        layout.addWidget(self.gnss_apply)
        layout.addWidget(self.stereo_reference)
        layout.addWidget(self.gnss_feedback, 1)
        self.row.parentWidget().layout().insertWidget(1, self.gnss_frame)
        self.gnss_apply.clicked.connect(self.apply_gnss)
        window.client.gnss_operation_result.connect(self.gnss_acknowledged)
        window.client.replay_operation_result.connect(self.replay_acknowledged)

    def apply_gnss(self) -> None:
        """Present selected intent; native parameter acknowledgement is shown independently."""
        if self.gnss_pending is not None:
            return
        intent = GnssIntent(
            self.gnss_request or uuid.uuid4().hex,
            tuple(sorted(self.row.selected_ids)),
            self.gnss_enabled.isChecked(),
        )
        self.gnss_request = None
        self.gnss_pending = intent
        self.gnss_apply.setEnabled(False)
        self.gnss_enabled.setEnabled(False)
        self.gnss_feedback.setText("Applying GNSS input; waiting for acknowledgement")
        try:
            self.window.client.execute_gnss_targets(
                intent.target_ids, intent.enabled, intent.request_id
            )
        except ValueError as error:
            self.gnss_acknowledged(intent.request_id, {"accepted": False, "error": str(error)})

    def gnss_acknowledged(self, request_id: str, result: dict) -> None:
        intent = self.gnss_pending
        if intent is None or intent.request_id != request_id:
            return
        accepted = result.get("accepted") is True and not result.get("error")
        message = (
            "GNSS input acknowledged; inspect measured GPS / fusion below"
            if accepted
            else (
                "GNSS request failed: "
                + str(result.get("error", "Native acknowledgement rejected"))
            )
        )
        self.gnss_feedback.setText(message)
        self.save_ack(
            intent.request_id,
            {
                "accepted": accepted,
                "wall_time": time.time(),
                "requested_enabled": intent.enabled,
                "target_ids": list(intent.target_ids),
                "result": result,
            },
        )
        self.gnss_pending = None
        self.gnss_apply.setEnabled(True)
        self.gnss_enabled.setEnabled(True)

    def replay_acknowledged(self, request_id: str, result: dict) -> None:
        identifier = self.replay_pending.pop(request_id, None)
        if identifier:
            self.save_ack(identifier, {**result, "wall_time": time.time()})

    @Slot(dict)
    def apply_request(self, request: dict) -> None:
        if request.get("kind") in {"replay", "gnss"}:
            if request.get("kind") == "gnss" and self.gnss_pending is not None:
                self.save_ack(
                    request["request_id"], {"accepted": False, "error": "A GNSS request is pending"}
                )
                return
            if request.get("target_ids") is not None:
                self.row.clear_selection()
                for identity in request["target_ids"]:
                    self.row.checkboxes[identity].setChecked(True)
            if request["kind"] == "gnss":
                self.gnss_request = request["request_id"]
                self.gnss_enabled.setChecked(request["enabled"])
                self.gnss_apply.click()
            else:
                button = {
                    "start": self.capture_row.start_button,
                    "stop": self.capture_row.stop_button,
                    "keep": self.capture_row.keep_button,
                }[request["action"]]
                if not button.isEnabled():
                    self.save_ack(
                        request["request_id"], {"accepted": False, "error": button.toolTip()}
                    )
                    return

                def click_visible_button() -> None:
                    button.click()
                    self.replay_pending[self.capture_row.pending_id] = request["request_id"]

                if request["action"] == "keep":
                    tabs = self.window.findChild(QTabWidget, "rightWorkspaceTabs")
                    tabs.setCurrentIndex(2)
                    QTimer.singleShot(250, click_visible_button)
                else:
                    click_visible_button()
            return
        if request.get("action") == "start":
            self.stereo_reference.setChecked(request.get("stereo_horizontal_reference", False))
        super().apply_request(request)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    args.evidence.mkdir(parents=True, exist_ok=True)
    app = QApplication([])
    apply_theme(app)
    window = MainWindow()
    arrange_native_capture_views(window)
    harness = CombinedPresentationHarness(window, args.evidence)
    window.setWindowTitle("Drone Simulation — isolated recorded demonstration")
    window.resize(1600, 1000)
    window.showNormal()
    window.move(100, 70)
    Path("/runtime/Desktop.window").write_text(str(int(window.winId())))

    def snapshot() -> None:
        labels = [{"text": item.text()} for item in window.findChildren(QLabel) if item.isVisible()]
        record = {
            "wall_time": time.time(),
            "window_id": int(window.winId()),
            "geometry": [window.width(), window.height()],
            "native_viewers": {
                p.window_file.stem: p.foreign_window is not None
                for p in window.findChildren(ViewerPanel)
            },
            "visible_labels": labels,
        }
        (args.evidence / "native-widget-state.json").write_text(json.dumps(record, indent=2) + "\n")

    timer = QTimer(window)
    timer.timeout.connect(snapshot)
    timer.start(1000)
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    QTimer.singleShot(7_200_000, window.close)
    try:
        return app.exec()
    finally:
        harness.close()


if __name__ == "__main__":
    raise SystemExit(main())
