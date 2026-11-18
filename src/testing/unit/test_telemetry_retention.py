"""Idle rotation keeps flight evidence and never converts an I/O failure to health."""

import errno
import json
from pathlib import Path
from unittest.mock import patch

from common.logging.storage import BoundedRecording
from common.logging.telemetry import TelemetryRecording


def test_long_idle_recording_keeps_only_two_bounded_recent_segments(tmp_path: Path) -> None:
    recording = TelemetryRecording(tmp_path, "vehicle_1", "run-1", idle_segment_bytes=8)
    for _ in range(50):
        assert recording.write("xx\n")
    recording.close()
    files = list(tmp_path.glob("*.jsonl"))
    assert len(files) == 2
    assert sum(path.stat().st_size for path in files) <= 16
    assert recording.snapshot()["available"]
    manifest = json.loads(recording.manifest_path.read_text())
    assert manifest["closed"] and manifest["idle_bytes_removed"] > 0


def test_idle_rotation_preserves_flight_and_pre_arm_context(tmp_path: Path) -> None:
    recording = TelemetryRecording(tmp_path, "vehicle_2", "run-2", idle_segment_bytes=8)
    for _ in range(8):
        recording.write("x\n")
    recording.observe_armed(True)
    assert recording.write("flight record\n")
    recording.observe_armed(False)
    protected = [tmp_path / row["path"] for row in recording.segments if row["kind"] != "idle"]
    for _ in range(50):
        assert recording.write("x\n")
    recording.close()
    assert protected and all(path.exists() for path in protected)
    assert any("flight record" in path.read_text() for path in protected)
    assert len([row for row in recording.segments if row["kind"] == "idle"]) == 2


def test_storage_failure_remains_latched_across_arm_disarm_and_rotation(tmp_path: Path) -> None:
    class FaultWriter(BoundedRecording):
        failing = False

        def write(self, text: str) -> bool:
            if self.failing:
                self.error = str(OSError(errno.ENOSPC, "Injected full disk"))
                return False
            return super().write(text)

    recording = TelemetryRecording(
        tmp_path, "vehicle_1", "run-3", idle_segment_bytes=8, writer_factory=FaultWriter
    )
    assert recording.write("first\n")
    FaultWriter.failing = True
    assert not recording.write("x\n")
    recording.observe_armed(True)
    recording.observe_armed(False)
    assert not recording.write("x\n")
    assert not recording.snapshot()["available"]
    assert "Injected full disk" in recording.error
    recording.close()


def test_idle_expiry_failure_is_visible_and_retains_the_file(tmp_path: Path) -> None:
    recording = TelemetryRecording(
        tmp_path, "vehicle_1", "run-4", idle_segment_bytes=8, idle_segments=1
    )
    recording.write("first\n")
    path = Path(recording.snapshot()["path"])
    with patch("pathlib.Path.unlink", side_effect=PermissionError("retention denied")):
        assert not recording.write("next\n")
    assert path.exists()
    assert "retention denied" in recording.error
    recording.close()
