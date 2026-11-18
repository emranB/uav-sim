"""Exact estimator input archives have explicit finite storage and replay identities."""

from pathlib import Path

import pytest

from common.logging.stereo import StereoFrame, StereoRecording
from common.logging.stereo_inputs import StereoInputArchive


def test_archive_preserves_exact_pairs_and_stops_at_its_limit(tmp_path: Path) -> None:
    recording = StereoRecording(
        tmp_path,
        (StereoFrame(0, "left.ppm", "right.ppm", 320, 240),),
        {"baseline_m": 0.4, "rate_hz": 10},
        vehicle_id="vehicle_1",
    )
    archive = StereoInputArchive(tmp_path / "inputs", recording, 2, 10, "run-one")
    assert archive.append(1, [b"left-one", b"right-one"])
    with pytest.raises(ValueError, match="increase"):
        archive.append(1, [b"left-one", b"right-one"])
    assert archive.append(2, [b"left-two", b"right-two"])
    assert not archive.append(3, [b"left-three", b"right-three"])
    replay = StereoRecording.open(tmp_path / "inputs/report.json")
    assert replay.vehicle_id == "vehicle_1"
    assert len(replay.frames) == 2
    assert replay.images(1) == (b"left-two", b"right-two")


def test_failed_pair_publication_does_not_claim_an_archived_frame(tmp_path: Path) -> None:
    import errno
    import json
    from unittest.mock import patch

    recording = StereoRecording(
        tmp_path,
        (StereoFrame(0, "left.ppm", "right.ppm", 320, 240),),
        {"baseline_m": 0.4, "rate_hz": 10},
        vehicle_id="vehicle_1",
    )
    archive = StereoInputArchive(tmp_path / "inputs", recording, 3, 10, "run-one")
    assert archive.append(1, [b"left-one", b"right-one"])
    original = (tmp_path / "inputs/report.json").read_bytes()
    with patch(
        "common.logging.stereo_inputs.write_atomic", side_effect=OSError(errno.ENOSPC, "full")
    ):
        with pytest.raises(OSError):
            archive.append(2, [b"left-two", b"right-two"])
    assert (tmp_path / "inputs/report.json").read_bytes() == original
    assert len(json.loads(original)["pairs"]) == 1
    assert archive.append(2, [b"left-two", b"right-two"])
    assert len(StereoRecording.open(tmp_path / "inputs/report.json").frames) == 2
