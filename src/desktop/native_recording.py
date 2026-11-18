"""Connect native replay controls to an owned, asynchronous window recording."""

import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Callable

from PySide6.QtCore import QObject, QTimer, Signal

from desktop.client import SimulationClient
from integrations.recording.native_screen import NativeScreenRecording


class NativeCaptureRecording(QObject):
    """Prepare before Start, bind its acknowledgement, and seal after Stop/Keep.

    Window pixels continue for 2.5 seconds after the last Stop/Keep acknowledgement.
    Encoder shutdown, probing and hashing run outside the GUI thread. Flight commands
    and the backend's existing automatic render queue remain independent.
    """

    busy_changed = Signal(bool)
    failure = Signal(str)
    sealed = Signal(dict)
    task_finished = Signal(str, object, str)

    def __init__(
        self,
        client: SimulationClient,
        outputs: Path,
        window_id: Callable[[], int],
        window_size: Callable[[], tuple[int, int]],
        parent: QObject,
        recorder_factory: Callable[[Path, int], NativeScreenRecording] = NativeScreenRecording,
    ) -> None:
        super().__init__(parent)
        self.client = client
        self.outputs = outputs
        self.window_id = window_id
        self.window_size = window_size
        self.recorder_factory = recorder_factory
        self.recorder: NativeScreenRecording | None = None
        self.requests: dict[str, str] = {}
        self.start_request: tuple[str, dict] | None = None
        self.finishing = False
        self.closed = False
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="native-recording")
        self.tail_timer = QTimer(self)
        self.tail_timer.setSingleShot(True)
        self.tail_timer.setInterval(2500)
        self.tail_timer.timeout.connect(self.finish)
        self.task_finished.connect(self.completed)
        client.replay_operation_result.connect(self.acknowledged)
        client.resource_received.connect(self.capture_status)

    def capture_status(self, name: str, payload: dict) -> None:
        """Seal our screen recording when the bounded backend capture stops itself."""
        capture = payload.get("capture", {})
        if (
            name == "replay"
            and self.recorder is not None
            and not self.finishing
            and self.recorder.capture_id is not None
            and self.recorder.capture_id == capture.get("capture_id")
            and capture.get("state") in {"stopped", "interrupted", "error"}
            and not self.tail_timer.isActive()
        ):
            self.tail_timer.start()

    def execute(self, operation: str, body: dict, request_id: str) -> None:
        """Dispatch one immutable native intent after any required recorder preparation."""
        if operation == "replay/start":
            if self.closed or self.start_request or self.recorder or self.finishing:
                self.reject(request_id, "The previous native recording is still finalizing")
                return
            if self.window_size() not in {(1600, 1000), (1920, 1200)}:
                self.reject(
                    request_id, "Native capture requires 1600x1000 or 1920x1200 window geometry"
                )
                return
            self.start_request = (request_id, {"target_ids": list(body["target_ids"])})
            self.busy_changed.emit(True)
            identifier = self.window_id()

            def prepare() -> None:
                self.recorder = self.recorder_factory(self.outputs, identifier)
                self.recorder.start()

            self.submit("prepare", prepare)
            return
        self.requests[request_id] = operation
        self.client.execute_replay(operation, body, request_id)

    def reject(self, request_id: str, message: str) -> None:
        """Report preparation errors through the same correlated acknowledgement path."""
        self.failure.emit(message)
        self.client.replay_operation_result.emit(request_id, {"accepted": False, "error": message})

    def submit(self, task: str, operation: Callable[[], object]) -> None:
        def run() -> None:
            try:
                result = operation()
            except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
                self.task_finished.emit(task, None, f"Native recording {task} failed: {error}")
            else:
                self.task_finished.emit(task, result, "")

        self.executor.submit(run)

    def completed(self, task: str, result: object, error: str) -> None:
        """Apply worker completion on the Qt thread, preserving the original request ID."""
        if self.closed:
            return
        if task == "prepare":
            pending, self.start_request = self.start_request, None
            if pending is None:
                return
            request_id, body = pending
            if error:
                self.reject(request_id, error)
                self.finish()
                return
            self.requests[request_id] = "replay/start"
            self.client.execute_replay("replay/start", body, request_id)
        elif task == "finish":
            self.recorder = None
            self.finishing = False
            self.busy_changed.emit(False)
            if error:
                self.failure.emit(error)
            else:
                self.sealed.emit(result)
        elif error:
            self.failure.emit(error)
            self.finish()

    def acknowledged(self, request_id: str, payload: dict) -> None:
        """Use acknowledged capture identity, never the currently inspected vehicle."""
        operation = self.requests.pop(request_id, None)
        if operation is None or self.closed:
            return
        if payload.get("error") or payload.get("accepted") is False:
            if operation == "replay/start":
                self.finish()
            return
        if operation == "replay/start" and self.recorder is not None:
            capture_id = payload.get("capture_id", "")
            recorder = self.recorder
            self.submit("bind", lambda: recorder.bind(capture_id))
        elif operation in {"replay/stop", "replay/keep"} and self.recorder is not None:
            self.tail_timer.start()

    def finish(self) -> None:
        """Seal only the screen process owned by this desktop capture."""
        if self.finishing:
            return
        self.tail_timer.stop()
        self.finishing = True

        def finalize() -> dict:
            return self.recorder.finish() if self.recorder is not None else {}

        self.submit("finish", finalize)

    def close(self) -> None:
        """Drain bounded recorder shutdown after the application's Qt loop exits."""
        self.closed = True
        self.tail_timer.stop()
        if not self.finishing:
            self.finish()
        self.executor.shutdown(wait=True)
