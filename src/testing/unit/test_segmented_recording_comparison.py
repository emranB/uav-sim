"""Recordings emitted by the idle/flight writer remain comparable through the catalog."""

import json
from pathlib import Path

import pytest

from common.logging.records import RecordingCatalog
from common.logging.telemetry import TelemetryRecording


def test_writer_segments_are_listed_and_compared_without_renaming(tmp_path: Path) -> None:
    writer = TelemetryRecording(tmp_path / "telemetry", "vehicle_1", "controller")
    names = []
    for armed, altitude in ((False, 0), (True, 4000), (False, 0)):
        writer.observe_armed(armed)
        writer.write(
            json.dumps(
                {
                    "schema_version": 2,
                    "timestamp_utc": "2026-11-11T12:00:00+00:00",
                    "message": {
                        "mavpackettype": "GLOBAL_POSITION_INT",
                        "lat": 446488000,
                        "lon": -635752000,
                        "relative_alt": altitude,
                        "vx": 120,
                        "vy": 0,
                    },
                }
            )
            + "\n"
        )
        names.append(Path(writer.snapshot()["path"]).name)
    writer.close()
    catalog = RecordingCatalog(tmp_path)
    assert set(names) <= {row["name"] for row in catalog.list()["telemetry"]}
    result = catalog.compare(names[0], names[1])["comparison"]
    assert result["left"]["position_samples"] == result["right"]["position_samples"] == 1
    assert result["delta"]["maximum_altitude_m"] == 4
    assert catalog.statistics(names[2])["maximum_altitude_m"] == 0


@pytest.mark.parametrize(
    "name",
    [
        "../telemetry-1-0001-idle.jsonl",
        "telemetry-1-0001-other.jsonl",
        "telemetry-1-1-flight.jsonl",
        "telemetry-1-0001-flight.jsonl/other",
        "telemetry-1.manifest.json",
    ],
)
def test_unrecognized_names_and_traversal_remain_rejected(tmp_path: Path, name: str) -> None:
    with pytest.raises(ValueError, match="inventory"):
        RecordingCatalog(tmp_path).statistics(name)
