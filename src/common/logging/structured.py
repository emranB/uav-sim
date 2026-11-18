"""Shared file format for application and desktop diagnostics."""

import json
import logging
from datetime import datetime, timezone
from pathlib import Path

from common.logging.storage import BoundedRecording


class StructuredFormatter(logging.Formatter):
    def __init__(self, source: str) -> None:
        super().__init__()
        self.source = source

    def format(self, record: logging.LogRecord) -> str:
        return json.dumps(
            {
                "timestamp_utc": datetime.fromtimestamp(record.created, timezone.utc).isoformat(),
                "source": self.source,
                "vehicle_id": getattr(record, "vehicle_id", None),
                "level": record.levelname,
                "message": record.getMessage(),
            }
        )


class RecordingHandler(logging.Handler):
    """Keep diagnostics bounded without removing old records or failing caller threads."""

    def __init__(self, path: Path) -> None:
        super().__init__()
        self.recording = BoundedRecording(path)

    def emit(self, record: logging.LogRecord) -> None:
        self.recording.write(self.format(record) + "\n")

    def close(self) -> None:
        self.recording.close()
        super().close()


def recording_handler(directory: Path, source: str) -> RecordingHandler:
    handler = RecordingHandler(directory / f"{source}.jsonl")
    handler.setFormatter(StructuredFormatter(source))
    return handler
