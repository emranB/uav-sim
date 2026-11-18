"""Exercise bounded pipe failures and optional recording backpressure."""

import errno
import threading
import time
from pathlib import Path

import pytest

from common.logging.async_recording import AsyncRecording
from integrations.vision.native_pipe import NativePosePipe

FAKE_NATIVE = """#!/usr/bin/env python3
import json, os, struct, sys, time
request = struct.Struct("<QII")
while header := sys.stdin.buffer.read(request.size):
    timestamp, left, right = request.unpack(header)
    sys.stdin.buffer.read(left + right)
    if MODE == "timeout":
        time.sleep(3)
    if MODE == "truncated":
        sys.stdout.buffer.write(struct.pack("<I", 10) + b"x")
        sys.stdout.buffer.flush()
        break
    if MODE == "eof":
        break
    if MODE == "diagnostics":
        os.write(2, b"x" * 150000)
    response = json.dumps({"timestamp_seconds": timestamp / 1e9}).encode()
    sys.stdout.buffer.write(struct.pack("<I", len(response)) + response)
    sys.stdout.buffer.flush()
"""


def child(directory: Path, mode: str) -> str:
    executable = directory / f"native-{mode}"
    executable.write_text(FAKE_NATIVE.replace("MODE", repr(mode)))
    executable.chmod(0o755)
    return str(executable)


@pytest.mark.parametrize(
    ("mode", "error", "reason"),
    [
        ("timeout", TimeoutError, "frame deadline"),
        ("truncated", RuntimeError, "truncated pose response"),
        ("eof", RuntimeError, "EOF"),
    ],
)
def test_pipe_rejects_failed_exchange(
    tmp_path: Path,
    mode: str,
    error: type[Exception],
    reason: str,
) -> None:
    with NativePosePipe(
        tmp_path / "camera.yaml", tmp_path / "native.log", child(tmp_path, mode)
    ) as pipe:
        with pytest.raises(error, match=reason):
            pipe.estimate(
                [b"left", b"right"], 1_000_000_000, threading.Event(), timeout_seconds=0.2
            )
        with pytest.raises(RuntimeError, match="must restart"):
            pipe.estimate([b"left", b"right"], 2_000_000_000, threading.Event())
    assert pipe.process.poll() is not None


@pytest.mark.parametrize("fail", [False, True])
def test_pipe_delivers_poses_while_diagnostics_sink_is_slow_or_failing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    fail: bool,
) -> None:
    entered = threading.Event()
    release = threading.Event()
    executable = child(tmp_path, "diagnostics")

    class Sink:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            return None

        def write(self, payload):
            entered.set()
            if fail:
                raise OSError(errno.ENOSPC, "simulated disk full")
            release.wait(5)
            return len(payload)

    monkeypatch.setattr(Path, "open", lambda *_args, **_kwargs: Sink())
    pipe = NativePosePipe(tmp_path / "camera.yaml", tmp_path / "native.log", executable)
    try:
        stop = threading.Event()
        for frame in range(8):
            response = pipe.estimate(
                [b"left", b"right"], frame * 1_000_000_000, stop, timeout_seconds=0.8
            )
            assert response["timestamp_seconds"] == frame
        assert entered.is_set()
        assert pipe.diagnostics.pending.qsize() <= 64
        assert pipe.diagnostics.dropped > 0
        if fail:
            assert "disk full" in pipe.diagnostics.error
    finally:
        release.set()
        pipe.close()
    assert pipe.process.returncode == 0


def test_recording_close_is_bounded_if_sink_remains_blocked(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    entered = threading.Event()
    release = threading.Event()

    class Sink:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            return None

        def write(self, payload):
            entered.set()
            release.wait(3)
            return len(payload)

    monkeypatch.setattr(Path, "open", lambda *_args, **_kwargs: Sink())
    recorder = AsyncRecording(tmp_path / "poses.jsonl", pending_limit=2)
    try:
        assert recorder.offer({"position": 1})
        assert entered.wait(1)
        assert recorder.offer({"position": 2})
        assert recorder.offer({"position": 3})
        assert not recorder.offer({"position": 4})
        started = time.monotonic()
        status = recorder.close(timeout_seconds=0.05)
        assert time.monotonic() - started < 0.3
        assert status["pending"] == 2 and status["dropped"] == 1
        assert not status["writer_finished"]
    finally:
        release.set()
        recorder.close()
