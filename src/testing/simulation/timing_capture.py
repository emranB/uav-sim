"""Bounded asynchronous event recording for opt-in runtime experiments."""

import json
import logging
import queue
import threading
import time
from pathlib import Path


class TimingRecorder:
    """Bound diagnostic work; failed diagnostics never masquerade as complete evidence."""

    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self._directory = directory
        self._closed = False
        self._queue = queue.Queue(maxsize=4096)
        self._dropped = 0
        self._error = None
        self._count = 0
        self._thread = threading.Thread(
            target=self._write, name="stereo-timing-writer", daemon=True
        )
        self._thread.start()

    def emit(self, event: dict) -> None:
        if self._closed:
            return
        event["logged_utc_seconds"] = time.time()
        if self._count >= 100_000:
            self._dropped += 1
            return
        self._count += 1
        try:
            self._queue.put_nowait(event)
        except queue.Full:
            self._dropped += 1

    def _write(self) -> None:
        try:
            with (self._directory / "events.jsonl").open("x", buffering=1) as stream:
                while True:
                    event = self._queue.get()
                    if event is None:
                        return
                    stream.write(json.dumps(event, allow_nan=False) + "\n")
        except (OSError, ValueError) as error:
            self._error = str(error)
            logging.exception("Stereo timing capture failed")

    def close(self) -> None:
        self._closed = True
        self._queue.put(None, timeout=3)
        self._thread.join(timeout=5)
        report = {
            "schema_version": 1,
            "complete": not self._thread.is_alive() and self._error is None and self._dropped == 0,
            "events_offered": self._count,
            "dropped_events": self._dropped,
            "error": self._error,
            "clock_sources": "Gazebo sensor header simulation time; actual callback/worker/publication monotonic times; UTC logging clock retained separately.",
        }
        (self._directory / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
