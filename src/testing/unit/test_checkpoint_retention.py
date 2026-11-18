"""Reviewed raw retention rejects changed, active, redirected and protected evidence."""

import hashlib
import json
import os
import time
from pathlib import Path

import pytest

from common.logging.checkpoint_retention import apply_checkpoint_retention, raw_recording_path


def plan(tmp_path: Path) -> tuple[Path, Path]:
    file = tmp_path / "perception/run-old/0001-left.ppm"
    file.parent.mkdir(parents=True)
    file.write_bytes(b"old image")
    os.utime(file, (time.time() - 600, time.time() - 600))
    document = {
        "schema_version": 1,
        "checkpoint_id": "checkpoint-1",
        "protected_paths": [],
        "candidate_files": [
            {
                "relative_path": str(file.relative_to(tmp_path)),
                "sha256": hashlib.sha256(file.read_bytes()).hexdigest(),
                "bytes": file.stat().st_size,
                "mtime_ns": file.stat().st_mtime_ns,
                "reason": "Superseded closed preliminary raw input",
            }
        ],
    }
    path = tmp_path / "plan.json"
    path.write_text(json.dumps(document))
    return path, file


def test_reviewed_raw_file_removal_preserves_metadata_and_audits_checksum(tmp_path: Path) -> None:
    path, file = plan(tmp_path)
    metadata = file.parent / "report.json"
    metadata.write_text('{"passed":true}')
    result = apply_checkpoint_retention(
        tmp_path, path, tmp_path / "audit.json", inode_provider=lambda: set()
    )
    assert result["state"] == "complete" and result["files_removed"] == 1
    assert not file.exists() and metadata.exists()
    assert Path(result["journal"]).exists()


def test_protected_overlap_rejected_before_any_deletion(tmp_path: Path) -> None:
    path, file = plan(tmp_path)
    document = json.loads(path.read_text())
    document["protected_paths"] = ["perception/run-old"]
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="protected"):
        apply_checkpoint_retention(
            tmp_path, path, tmp_path / "audit.json", inode_provider=lambda: set()
        )
    assert file.exists()


def test_open_file_is_kept_even_if_the_plan_missed_it(tmp_path: Path) -> None:
    path, file = plan(tmp_path)
    info = file.stat()
    result = apply_checkpoint_retention(
        tmp_path,
        path,
        tmp_path / "audit.json",
        inode_provider=lambda: {(info.st_dev, info.st_ino)},
    )
    assert file.exists() and result["files_removed"] == 0


def test_changed_checksum_stops_cleanup_and_retains_file(tmp_path: Path) -> None:
    path, file = plan(tmp_path)
    before = file.stat()
    file.write_bytes(b"new image")
    os.utime(file, ns=(before.st_atime_ns, before.st_mtime_ns))
    with pytest.raises(RuntimeError, match="checksum"):
        apply_checkpoint_retention(
            tmp_path, path, tmp_path / "audit.json", inode_provider=lambda: set()
        )
    assert file.exists()


@pytest.mark.parametrize(
    "path",
    [
        "../source.py",
        "reports/failure.json",
        "px4/run/log.txt",
        "perception/config.json",
        "/tmp/source.ppm",
    ],
)
def test_retention_cannot_reach_source_or_compact_evidence(tmp_path: Path, path: str) -> None:
    with pytest.raises(ValueError):
        raw_recording_path(tmp_path, path)
