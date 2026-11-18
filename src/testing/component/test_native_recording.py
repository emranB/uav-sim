"""Native capture preparation, acknowledgement identity and owned shutdown."""

import time
from pathlib import Path
from typing import Callable

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication

from desktop.native_recording import NativeCaptureRecording


class CaptureClient(QObject):
    replay_operation_result = Signal(str, dict)
    resource_received = Signal(str, dict)

    def __init__(self) -> None:
        super().__init__()
        self.calls = []

    def execute_replay(self, operation: str, body: dict, request_id: str) -> None:
        self.calls.append((operation, body, request_id))


class ScreenRecorder:
    def __init__(self, outputs: Path, identifier: int) -> None:
        self.started = False
        self.bound = []
        self.finished = False
        self.capture_id = None

    def start(self) -> None:
        self.started = True

    def bind(self, capture_id: str) -> None:
        self.bound.append(capture_id)
        self.capture_id = capture_id

    def finish(self) -> dict:
        self.finished = True
        return {"state": "completed"}


def test_backend_duration_stop_seals_only_the_matching_native_capture() -> None:
    app, client, controller = make_recording()
    controller.tail_timer.setInterval(1)
    try:
        controller.execute("replay/start", {"target_ids": ["vehicle_1"]}, "start")
        pump_until(app, lambda: bool(client.calls))
        client.replay_operation_result.emit(
            "start", {"accepted": True, "capture_id": "capture-owned"}
        )
        pump_until(app, lambda: controller.recorder.capture_id == "capture-owned")
        recorder = controller.recorder
        client.resource_received.emit(
            "replay", {"capture": {"capture_id": "capture-other", "state": "stopped"}}
        )
        assert not recorder.finished
        client.resource_received.emit(
            "replay", {"capture": {"capture_id": "capture-owned", "state": "stopped"}}
        )
        pump_until(app, lambda: recorder.finished)
    finally:
        controller.close()


def pump_until(app: QApplication, condition: Callable[[], bool]) -> None:
    deadline = time.monotonic() + 3
    while not condition():
        assert time.monotonic() < deadline
        app.processEvents()
        time.sleep(0.005)


def make_recording(
    factory=ScreenRecorder, size: tuple[int, int] = (1600, 1000)
) -> tuple[QApplication, CaptureClient, NativeCaptureRecording]:
    app = QApplication.instance() or QApplication([])
    client = CaptureClient()
    controller = NativeCaptureRecording(
        client, Path("unused-test-output"), lambda: 42, lambda: size, client, factory
    )
    return app, client, controller


def test_recording_prepares_before_capture_and_binds_only_matching_ack() -> None:
    app, client, controller = make_recording()
    body = {"target_ids": ["vehicle_1"]}
    try:
        controller.execute("replay/start", body, "start-atlas")
        body["target_ids"].append("vehicle_2")
        pump_until(app, lambda: bool(client.calls))
        recorder = controller.recorder
        assert recorder.started
        assert client.calls == [("replay/start", {"target_ids": ["vehicle_1"]}, "start-atlas")]
        client.replay_operation_result.emit("other-request", {"capture_id": "wrong"})
        assert recorder.bound == []
        client.replay_operation_result.emit("start-atlas", {"capture_id": "capture-atlas"})
        pump_until(app, lambda: bool(recorder.bound))
        client.replay_operation_result.emit("start-atlas", {"capture_id": "duplicate"})
        assert recorder.bound == ["capture-atlas"]
        controller.execute("replay/stop", {}, "stop-atlas")
        client.replay_operation_result.emit("stop-atlas", {"state": "stopped"})
        assert controller.tail_timer.isActive()
        controller.finish()
        pump_until(app, lambda: controller.recorder is None)
        assert recorder.finished
    finally:
        controller.close()


def test_failed_screen_preparation_never_starts_backend_capture() -> None:
    class BrokenRecorder(ScreenRecorder):
        def start(self) -> None:
            raise OSError("Encoder unavailable")

    app, client, controller = make_recording(BrokenRecorder)
    results = []
    client.replay_operation_result.connect(
        lambda identity, result: results.append((identity, result))
    )
    try:
        controller.execute("replay/start", {"target_ids": ["vehicle_1"]}, "start-failed")
        pump_until(app, lambda: bool(results))
        assert client.calls == []
        assert results[0][0] == "start-failed"
        assert "Encoder unavailable" in results[0][1]["error"]
        pump_until(app, lambda: controller.recorder is None)
    finally:
        controller.close()


def test_wrong_window_geometry_rejects_capture_before_preparation() -> None:
    app, client, controller = make_recording(size=(1200, 800))
    results = []
    client.replay_operation_result.connect(lambda identity, result: results.append(result))
    try:
        controller.execute("replay/start", {"target_ids": ["vehicle_1"]}, "wrong-size")
        assert client.calls == []
        assert "1600" in results[0]["error"]
    finally:
        controller.close()


def test_failed_stop_keeps_owned_recorder_until_retry_or_shutdown() -> None:
    app, client, controller = make_recording()
    controller.execute("replay/start", {"target_ids": ["vehicle_1"]}, "start")
    pump_until(app, lambda: bool(client.calls))
    recorder = controller.recorder
    client.replay_operation_result.emit("start", {"capture_id": "capture-owned"})
    pump_until(app, lambda: bool(recorder.bound))
    controller.execute("replay/stop", {}, "stop")
    client.replay_operation_result.emit("stop", {"error": "Backend unavailable"})
    assert not controller.tail_timer.isActive()
    assert not recorder.finished
    controller.close()
    assert recorder.finished
