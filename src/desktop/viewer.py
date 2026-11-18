"""Host an independently owned X11 viewer without controlling simulation lifetime."""

import json
import logging
import sys
import time
from pathlib import Path

from PySide6.QtCore import QProcess, Qt, QTimer
from PySide6.QtGui import QCloseEvent, QWindow
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget


class ViewerPanel(QWidget):
    """Attach a published native window; its service remains the process owner."""

    def __init__(self, title: str, window_file: Path) -> None:
        super().__init__()
        self.window_file = window_file
        self.window_id: int | None = None
        self.window_revision: int | None = None
        self.foreign_window: QWindow | None = None
        self.container: QWidget | None = None
        self.parent_probe = QProcess(self)
        self.parent_probe.finished.connect(self.verify_parent)
        self.parent_repair = QProcess(self)
        self.parent_repair.finished.connect(self.finish_parent_repair)
        self.parent_repair.errorOccurred.connect(self.parent_repair_error)
        self.repair_cancelled = False
        self.repair_deadline = QTimer(self)
        self.repair_deadline.setSingleShot(True)
        self.repair_deadline.setInterval(3000)
        self.repair_deadline.timeout.connect(self.parent_repair.kill)
        self.probed_window: tuple | None = None
        self.last_parent_probe = 0.0
        self.panel_layout = QVBoxLayout(self)
        self.panel_layout.setContentsMargins(0, 0, 0, 0)
        self.message = QLabel(f"Waiting for {title}…")
        self.panel_layout.addWidget(self.message)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)

    def refresh(self) -> None:
        try:
            window_id = int(self.window_file.read_text(encoding="utf-8"))
            revision = self.window_file.stat().st_mtime_ns
        except (OSError, ValueError):
            window_id = None
            revision = None
        # X11 may reuse an ID before this poll observes the old file disappearing.
        if (window_id, revision) == (self.window_id, self.window_revision):
            self.fit_viewer()
            self.probe_parent()
            return
        self.detach()
        if window_id is None:
            return
        foreign_window = QWindow.fromWinId(window_id)
        if foreign_window is None:
            self.message.setText("This display backend cannot embed the viewer window.")
            return
        self.window_id = window_id
        self.window_revision = revision
        self.foreign_window = foreign_window
        self.container = QWidget.createWindowContainer(foreign_window, self)
        self.container.setAttribute(Qt.WidgetAttribute.WA_NativeWindow)
        self.container.setMinimumSize(200, 150)
        self.panel_layout.addWidget(self.container)
        self.panel_layout.setStretchFactor(self.container, 1)
        self.message.hide()
        self.fit_viewer()

    def probe_parent(self) -> None:
        if (
            self.foreign_window is None
            or self.window_id is None
            or self.parent_probe.state() != QProcess.ProcessState.NotRunning
            or self.parent_repair.state() != QProcess.ProcessState.NotRunning
            or time.monotonic() - self.last_parent_probe < 2
        ):
            return
        self.last_parent_probe = time.monotonic()
        self.probed_window = (self.window_id, self.window_revision)
        self.parent_probe.start(
            sys.executable,
            [
                "-m",
                "desktop.native_window",
                str(self.window_id),
                str(int(self.window().winId())),
            ],
        )

    def verify_parent(self, exit_code: int, exit_status: QProcess.ExitStatus) -> None:
        if exit_code or self.probed_window != (self.window_id, self.window_revision):
            return
        try:
            result = json.loads(bytes(self.parent_probe.readAllStandardOutput()))
        except ValueError:
            return
        if result["attached"] or self.foreign_window is None:
            return
        if self.container is None:
            return
        parent_id = int(self.container.winId())
        # Release window-manager ownership before repairing Qt's cached parent.
        logging.warning("Recovering externally reparented viewer %s", self.window_file.stem)
        self.repair_cancelled = False
        self.parent_repair.start(
            sys.executable,
            [
                "-m",
                "desktop.native_window",
                str(self.window_id),
                str(parent_id),
                "--attach",
            ],
        )
        self.repair_deadline.start()

    def finish_parent_repair(self, exit_code: int, exit_status: QProcess.ExitStatus) -> None:
        """Expose attachment failures while allowing a later observed repair to recover."""
        self.repair_deadline.stop()
        if self.repair_cancelled:
            return
        if exit_code or exit_status == QProcess.ExitStatus.CrashExit:
            details = bytes(self.parent_repair.readAllStandardError()).decode(errors="replace")
            reason = (
                details.strip().splitlines()[-1]
                if details.strip()
                else "Native repair failed or exceeded its deadline"
            )
            message = f"Could not attach {self.window_file.stem}: {reason[:240]}"
            self.message.setText(message)
            self.message.show()
            logging.error(message)
            return
        self.message.hide()
        self.fit_viewer()

    def parent_repair_error(self, error: QProcess.ProcessError) -> None:
        """A process that failed to start does not emit the normal finished signal."""
        if error == QProcess.ProcessError.FailedToStart:
            self.finish_parent_repair(-1, QProcess.ExitStatus.CrashExit)

    def fit_viewer(self) -> None:
        """Optional native integration hook for viewers that override embedded geometry."""

    def detach(self) -> None:
        """Release the hosted window without terminating its owning process."""
        self.repair_cancelled = True
        self.repair_deadline.stop()
        if self.parent_probe.state() != QProcess.ProcessState.NotRunning:
            self.parent_probe.kill()
            self.parent_probe.waitForFinished(200)
        if self.parent_repair.state() != QProcess.ProcessState.NotRunning:
            self.parent_repair.kill()
            self.parent_repair.waitForFinished(200)
        self.probed_window = None
        if self.foreign_window:
            self.foreign_window.hide()
            native_detach = QProcess(self)
            native_detach.start(
                sys.executable,
                [
                    "-m",
                    "desktop.native_window",
                    str(self.window_id),
                    "--detach",
                ],
            )
            if not native_detach.waitForFinished(1000):
                native_detach.kill()
                native_detach.waitForFinished(200)
                logging.warning("Native viewer detachment exceeded its shutdown deadline")
            native_detach.deleteLater()
            self.foreign_window.setParent(None)
        if self.container:
            self.panel_layout.removeWidget(self.container)
            self.container.deleteLater()
        self.container = None
        self.foreign_window = None
        self.window_id = None
        self.window_revision = None
        self.message.show()

    def closeEvent(self, event: QCloseEvent) -> None:
        self.timer.stop()
        self.detach()
        super().closeEvent(event)
