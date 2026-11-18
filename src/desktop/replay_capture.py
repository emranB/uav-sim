"""Explicit replay capture controls; vehicle selection is supplied by composition."""

import time
import uuid
from typing import TYPE_CHECKING, Callable

from PySide6.QtCore import QTimer, QUrl
from PySide6.QtGui import QCloseEvent, QDesktopServices
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QToolButton,
    QVBoxLayout,
)

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class ReplayCaptureRow(QFrame):
    """Show server-acknowledged writers independently of flight and diagnostic logging."""

    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.setObjectName("replayCaptureControls")
        self.client = client
        self.capture_dispatcher: Callable[[str, dict, str], None] = client.execute_replay
        self.native_recording_busy = False
        self.targets: tuple[str, ...] = ()
        self.pending_id = ""
        self.last_received = 0.0
        self.capture: dict = {"state": "unavailable"}
        self.items: list[dict] = []
        self.offline_job: dict = {"state": "idle"}
        self.automatic_job: dict = {"state": "idle"}
        self.renderer: dict = {"available": False}
        self.interval_capture_id = None
        self.observed_active_capture_id: str | None = None
        self.follow_capture_id: str | None = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 7, 12, 7)
        self.indicator = QLabel("○ Replay capture · Waiting for recorder status")
        self.indicator.setWordWrap(True)
        actions = QHBoxLayout()
        actions.addWidget(self.indicator, 1)
        self.start_button = QPushButton("Start Capture")
        self.start_button.setObjectName("replayStart")
        self.stop_button = QPushButton("Stop Capture")
        self.stop_button.setObjectName("replayStop")
        self.keep_button = QPushButton("Save run / Keep selected capture")
        self.keep_button.setObjectName("replayKeep")
        for button in (self.start_button, self.stop_button):
            actions.addWidget(button)
        self.offline_toggle = QToolButton()
        self.offline_toggle.setText("Replay tools ↗")
        self.offline_toggle.setCheckable(False)
        actions.addWidget(self.offline_toggle)
        layout.addLayout(actions)
        self.offline_page = QFrame()
        advanced = QVBoxLayout(self.offline_page)
        advanced.setContentsMargins(12, 12, 12, 12)
        advanced.addWidget(QLabel("Saved captures"))
        advanced.addWidget(self.keep_button)
        self.offline_frame = QFrame()
        offline = QVBoxLayout(self.offline_frame)
        offline.setContentsMargins(0, 0, 0, 0)
        self.saved = QComboBox()
        self.saved.setAccessibleName("Finalized replay captures")
        self.saved.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        self.saved.setMinimumContentsLength(12)
        offline.addWidget(self.saved)
        interval = QFormLayout()
        self.full_interval = QCheckBox("Full captured interval")
        self.full_interval.setChecked(True)
        interval.addRow(self.full_interval)
        self.from_seconds, self.to_seconds = QDoubleSpinBox(), QDoubleSpinBox()
        for name, spinner in (
            ("Start offset in wall seconds", self.from_seconds),
            ("End offset in wall seconds", self.to_seconds),
        ):
            spinner.setDecimals(3)
            spinner.setRange(0, 1800)
            spinner.setSuffix(" s")
            spinner.setAccessibleName(name)
            interval.addRow("From" if spinner is self.from_seconds else "To", spinner)
        offline.addLayout(interval)
        jobs = QGridLayout()
        self.export_button = QPushButton("Export interval")
        self.preview_button = QPushButton("Preview first 2 s")
        self.render_button = QPushButton("Render interval")
        self.cancel_button = QPushButton("Cancel offline job")
        for index, button in enumerate(
            (
                self.export_button,
                self.preview_button,
                self.render_button,
                self.cancel_button,
            )
        ):
            jobs.addWidget(button, index // 2, index % 2)
        offline.addLayout(jobs)
        advanced.addWidget(self.offline_frame)
        self.offline_frame.setVisible(True)
        self.full_interval.toggled.connect(self.refresh)
        self.export_button.clicked.connect(lambda: self.dispatch("replay/export"))
        self.render_button.clicked.connect(lambda: self.dispatch("replay/render"))
        self.preview_button.clicked.connect(lambda: self.dispatch("replay/render", preview=True))
        self.cancel_button.clicked.connect(lambda: self.dispatch("replay/cancel"))
        self.job_status = QLabel()
        self.job_status.setWordWrap(True)
        advanced.addWidget(self.job_status)
        self.play_button = QPushButton("Open completed 3D replay MP4")
        self.play_button.clicked.connect(self.open_video)
        advanced.addWidget(self.play_button)
        self.details = QLabel(
            "Diagnostic telemetry recording remains independent of replay capture."
        )
        self.details.setWordWrap(True)
        advanced.addWidget(self.details)
        self.feedback = QLabel()
        self.feedback.setWordWrap(True)
        advanced.addWidget(self.feedback)
        advanced.addStretch()
        self.start_button.clicked.connect(lambda: self.dispatch("replay/start"))
        self.stop_button.clicked.connect(lambda: self.dispatch("replay/stop"))
        self.keep_button.clicked.connect(lambda: self.dispatch("replay/keep"))
        self.saved.currentIndexChanged.connect(self.refresh)
        self.saved.activated.connect(self.preserve_saved_selection)
        client.resource_received.connect(self.receive)
        client.replay_operation_result.connect(self.finished)
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.poll)
        self.timer.start()
        self.refresh()

    def set_targets(self, targets: tuple[str, ...]) -> None:
        """Selection only affects the next Start; an active capture keeps its own IDs."""
        self.targets = tuple(targets)
        self.refresh()

    def poll(self) -> None:
        self.client.fetch("replay")
        self.refresh()

    def receive(self, resource: str, payload: dict) -> None:
        if resource != "replay":
            return
        self.last_received = time.monotonic()
        self.capture = payload.get("capture", {"state": "unavailable"})
        self.follow_active_capture()
        self.items = payload.get("items", [])
        self.offline_job = payload.get("offline_job", {"state": "unavailable"})
        self.automatic_job = payload.get("automatic_job", {"state": "idle"})
        if self.automatic_job.get("state") not in {"idle", "unavailable"}:
            if self.offline_job.get("state") in {
                "idle",
                "unavailable",
                "error",
                "completed",
                "cancelled",
            } or self.automatic_job.get("state") in {"queued", "rendering"}:
                self.offline_job = self.automatic_job
        self.renderer = payload.get("renderer", {"available": False})
        selected = self.saved.currentData()
        self.saved.blockSignals(True)
        self.saved.clear()
        for item in self.items:
            identifier = item["capture_id"]
            names = ", ".join((item.get("vehicle_names") or {}).values())
            self.saved.addItem(
                f"{identifier[-8:]} · {names or 'unknown vehicles'} · {item.get('state')}"
                + (" · retained" if item.get("retained") else ""),
                identifier,
            )
        index = self.saved.findData(self.follow_capture_id)
        if self.follow_capture_id is not None and index >= 0:
            self.follow_capture_id = None
        else:
            index = self.saved.findData(selected)
        self.saved.setCurrentIndex(max(0, index))
        self.saved.blockSignals(False)
        self.refresh()

    def follow_active_capture(self) -> None:
        """Select a newly finalized writer unless the user chooses another saved interval."""
        identifier = self.capture.get("capture_id")
        if (
            self.capture.get("state") in {"starting", "capturing", "finalizing"}
            and identifier
            and identifier != self.observed_active_capture_id
        ):
            self.observed_active_capture_id = identifier
            self.follow_capture_id = identifier

    def preserve_saved_selection(self, index: int) -> None:
        """An explicit saved-interval choice takes precedence over automatic following."""
        self.follow_capture_id = None

    def refresh(self) -> None:
        stale = time.monotonic() - self.last_received > 3
        state = self.capture.get("state", "unavailable")
        active = state in {"capturing", "starting", "finalizing"}
        names = self.capture.get("vehicle_names") or {}
        identities = self.capture.get("vehicle_ids", [])
        scope = ", ".join(names.get(key, key) for key in identities)
        labels = {
            "idle": "○ Idle",
            "starting": "◌ Starting writers",
            "capturing": "● Capturing",
            "finalizing": "◌ Finalizing",
            "stopped": "■ Stopped",
            "interrupted": "■ Interrupted",
            "error": "! Error",
        }
        title = "○ Status unknown" if stale else labels.get(state, "○ Unavailable")
        seconds = int(self.capture.get("duration_wall_seconds", 0))
        self.indicator.setText(
            f"Capture · {title}"
            + (f" · {scope} · {seconds // 60:02}:{seconds % 60:02}" if scope else "")
        )
        self.indicator.setToolTip(
            "Capture status is acknowledged by the recorder; flight is independent"
        )
        self.start_button.setEnabled(
            bool(self.targets)
            and not active
            and not stale
            and not self.pending_id
            and not self.native_recording_busy
        )
        self.start_button.setToolTip(
            "Capture selected IDs: " + ", ".join(self.targets)
            if self.targets
            else "Choose vehicles using the quick-action checkboxes"
        )
        # Stop remains available with stale polling: it only finalizes replay evidence.
        self.stop_button.setEnabled(state == "capturing" and not self.pending_id)
        self.stop_button.setToolTip("Finalize replay now; flight and diagnostic logging continue")
        self.keep_button.setEnabled(self.saved.currentData() is not None and not self.pending_id)
        capture_id = self.saved.currentData()
        item = next((row for row in self.items if row["capture_id"] == capture_id), {})
        if capture_id != self.interval_capture_id:
            self.interval_capture_id = capture_id
            self.from_seconds.setValue(0)
            self.to_seconds.setValue(item.get("duration_wall_seconds") or 0)
        self.from_seconds.setEnabled(not self.full_interval.isChecked())
        self.to_seconds.setEnabled(not self.full_interval.isChecked())
        job_active = self.offline_job.get("state") in {"queued", "running", "rendering"}
        available = capture_id is not None and not job_active and not self.pending_id and not stale
        self.export_button.setEnabled(available)
        self.render_button.setEnabled(available and self.renderer.get("available", False))
        self.preview_button.setEnabled(self.render_button.isEnabled())
        self.cancel_button.setEnabled(job_active and not self.pending_id)
        reason = self.renderer.get("reason") or (
            "Animated 3D reconstruction from recorded poses and retained scene assets. "
            "Preview renders at most the first 2 seconds of the selected captured interval."
        )
        self.render_button.setToolTip(reason)
        self.preview_button.setToolTip(reason)
        job = self.offline_job
        job_capture_id = job.get("capture_id", "")
        job_capture = next(
            (entry for entry in self.items if entry["capture_id"] == job_capture_id), {}
        )
        job_names = ", ".join((job_capture.get("vehicle_names") or {}).values())
        self.job_status.setText(
            f"3D replay job: {job.get('state', 'unknown')} · {100 * job.get('progress', 0):.0f}%"
            + (f" · {job['queued_count']} queued" if job.get("queued_count") else "")
            + (
                f"\n{job.get('title', 'Recorded replay')} · capture {job_capture_id[-8:]}"
                if job_capture_id
                else ""
            )
            + (f" · {job_names}" if job_names else "")
            + (f"\n{job['error']}" if job.get("error") else "")
            + (f"\n{job['path']}" if job.get("path") else "")
        )
        self.job_status.setToolTip("Rendering capture: " + job_capture_id)
        self.job_status.setVisible(job.get("state") not in {"idle", "unavailable"})
        self.play_button.setEnabled(job.get("state") == "completed" and bool(job.get("video_path")))
        rows = []
        for identity, row in self.capture.get("vehicles", {}).items():
            diagnostic = row.get("diagnostic_recorder", {})
            rows.append(
                f"{names.get(identity, identity)}: {row.get('samples', 0)} replay poses "
                f"({row.get('bytes_written', 0) // 1024} KiB); diagnostic "
                + ("available" if diagnostic.get("available") else "unknown/unavailable")
                + f" ({diagnostic.get('bytes_written', 0)} bytes); "
                + (row.get("error") or row.get("observation_error") or "")
            )
        details = (
            " · ".join(rows) or "Replay is an observed trajectory/event interval, not camera video."
        )
        if self.capture.get("automatic_render_error"):
            retention = (
                "Capture retained"
                if self.capture.get("automatic_capture_retained")
                else "Capture retention could not be confirmed"
            )
            details += f"\nAutomatic 3D replay not queued: {self.capture['automatic_render_error']}. {retention}."
        self.details.setText(details)
        self.details.setToolTip(
            "Telemetry write loss, link packet loss and camera frame loss are separate counters. "
            "Unavailable counters remain unknown; missing poses are not inferred packet loss.\n"
            + "\n".join(
                f"{identity}: {row.get('diagnostic_recorder', {}).get('path')}\n"
                f"Telemetry write loss: {row.get('diagnostic_recorder', {}).get('telemetry_write_losses')} · "
                f"API proxy packet drops: {row.get('diagnostic_recorder', {}).get('link_packet_losses')} · "
                f"Camera frame loss: {row.get('diagnostic_recorder', {}).get('camera_frame_losses')} · "
                f"Camera pair drops: {row.get('diagnostic_recorder', {}).get('camera_pair_drops')}"
                for identity, row in self.capture.get("vehicles", {}).items()
            )
        )
        self.feedback.setVisible(bool(self.feedback.text()))

    def open_video(self) -> None:
        """Open the actual encoded output using the desktop's configured media player."""
        path = self.offline_job.get("video_path")
        if path and not QDesktopServices.openUrl(QUrl.fromLocalFile(path)):
            self.feedback.setText(
                "No desktop media handler accepted the MP4. Open this file: " + path
            )
            self.feedback.setVisible(True)

    def queue_current_saved_run(self, vehicle_id: str) -> bool:
        """Request the current finalized capture after saving its vehicle's mission plan.

        Historical selection is preserved. An active, stale or unrelated capture is
        never stopped, retargeted or substituted for a saved mission's missing run.
        """
        identifier = self.capture.get("capture_id")
        if (
            self.capture.get("state") != "stopped"
            or not identifier
            or vehicle_id not in self.capture.get("vehicle_ids", [])
            or time.monotonic() - self.last_received > 3
            or self.pending_id
        ):
            return False
        self.dispatch("replay/keep", capture_id=identifier)
        return True

    def dispatch(
        self, operation: str, *, preview: bool = False, capture_id: str | None = None
    ) -> None:
        if self.pending_id:
            return
        body = (
            {"target_ids": list(self.targets)}
            if operation == "replay/start"
            else (
                {"capture_id": capture_id or self.saved.currentData()}
                if operation in {"replay/keep", "replay/export", "replay/render"}
                else {}
            )
        )
        if operation in {"replay/export", "replay/render"}:
            item = next(row for row in self.items if row["capture_id"] == body["capture_id"])
            if not self.full_interval.isChecked():
                body.update(
                    start_offset_seconds=self.from_seconds.value(),
                    end_offset_seconds=self.to_seconds.value(),
                )
            if preview:
                start = body.get("start_offset_seconds", 0)
                end = body.get("end_offset_seconds", item["duration_wall_seconds"])
                body.update(start_offset_seconds=start, end_offset_seconds=min(end, start + 2))
        self.pending_id = uuid.uuid4().hex
        self.feedback.setText("Waiting for replay acknowledgement…")
        self.refresh()
        self.capture_dispatcher(operation, body, self.pending_id)

    def set_native_recording_busy(self, busy: bool) -> None:
        """Wait for the owned screen recording to seal before starting another capture."""
        self.native_recording_busy = busy
        self.refresh()

    def set_capture_dispatcher(self, dispatch: Callable[[str, dict, str], None]) -> None:
        """Wire an optional asynchronous capture preparer at application startup."""
        self.capture_dispatcher = dispatch

    def finished(self, request_id: str, payload: dict) -> None:
        if request_id != self.pending_id:
            return
        self.pending_id = ""
        self.feedback.setText(
            payload.get("error")
            or (
                "Capture retained; 3D replay was not queued: " + payload["automatic_render_error"]
                if payload.get("automatic_render_error")
                else ""
            )
            or (
                "Retained saved capture; automatic 3D replay requested"
                if payload.get("retained")
                else ""
            )
        )
        if payload.get("state") and not payload.get("job_id"):
            self.capture = payload
            self.follow_active_capture()
            self.last_received = time.monotonic()
        elif payload.get("job_id"):
            self.offline_job = payload
        self.refresh()

    def closeEvent(self, event: QCloseEvent) -> None:
        self.timer.stop()
        super().closeEvent(event)
