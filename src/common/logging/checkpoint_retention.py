"""Apply an explicit dependency-based raw-recording plan at a stopped checkpoint."""

import gzip
import hashlib
import json
import os
import stat
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from common.logging.storage import write_json_atomic


@dataclass(frozen=True)
class RetentionCandidate:
    relative_path: str
    sha256: str
    bytes: int
    mtime_ns: int
    reason: str


def open_file_inodes() -> set[tuple[int, int]]:
    """Inspect this PID namespace; administrative cleanup requires host PID visibility."""
    result = set()
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            for descriptor in (process / "fd").iterdir():
                try:
                    info = descriptor.stat()
                    if stat.S_ISREG(info.st_mode):
                        result.add((info.st_dev, info.st_ino))
                except (FileNotFoundError, ProcessLookupError):
                    continue
        except (FileNotFoundError, ProcessLookupError):
            continue
    return result


def raw_recording_path(root: Path, relative_path: str) -> Path:
    """Limit deletion to original raw camera images, telemetry and native flight logs."""
    relative = Path(relative_path)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("Retention paths must remain relative to the output root")
    parts = relative.parts
    offset = 2 if len(parts) > 2 and parts[0] == "vehicles" else 0
    if len(parts) <= offset + 1:
        raise ValueError("Retention requires a complete raw-recording filename")
    category = parts[offset]
    allowed = (
        category == "perception"
        and relative.suffix == ".ppm"
        or category == "telemetry"
        and relative.name.startswith("telemetry-")
        and relative.suffix == ".jsonl"
        or category == "px4"
        and relative.suffix == ".ulg"
        and "log" in parts
    )
    path = root / relative
    if not allowed or path.resolve() != path.absolute():
        raise ValueError(f"Retention refuses non-raw or redirected path: {relative_path}")
    return path


def apply_checkpoint_retention(
    root: Path,
    plan_path: Path,
    report_path: Path,
    *,
    inode_provider: Callable[[], set[tuple[int, int]]] = open_file_inodes,
) -> dict:
    """Remove only reviewed, unchanged closed raw files; preserve an audit for every byte set."""
    root = root.resolve()
    plan = json.loads(plan_path.read_text())
    if plan.get("schema_version") != 1 or not isinstance(plan.get("checkpoint_id"), str):
        raise ValueError("Retention requires an identified version-one checkpoint plan")
    candidates = tuple(RetentionCandidate(**row) for row in plan["candidate_files"])
    protected = tuple(plan["protected_paths"])
    minimum_age = plan.get("minimum_age_seconds", 300)
    if type(minimum_age) is not int or minimum_age < 60:
        raise ValueError("Checkpoint cleanup requires at least 60 seconds of file quiescence")
    paths = [raw_recording_path(root, row.relative_path) for row in candidates]
    for candidate in candidates:
        if any(
            candidate.relative_path == prefix or candidate.relative_path.startswith(prefix + "/")
            for prefix in protected
        ):
            raise ValueError(f"Candidate overlaps protected evidence: {candidate.relative_path}")
    if report_path.exists():
        raise FileExistsError("Use a new cleanup report; never overwrite a previous audit")
    audit_path = report_path.with_suffix(".files.jsonl.gz")
    if audit_path.exists():
        raise FileExistsError("Cleanup journal already exists")
    report = {
        "schema_version": 1,
        "checkpoint_id": plan["checkpoint_id"],
        "state": "running",
        "plan": str(plan_path),
        "started_wall_time": time.time(),
        "available_before": os.statvfs(root).f_bavail * os.statvfs(root).f_frsize,
        "files_removed": 0,
        "logical_bytes_removed": 0,
        "allocated_bytes_released": 0,
        "skipped": [],
        "journal": str(audit_path),
    }
    opened = inode_provider()
    checked_at = time.monotonic()
    try:
        with gzip.open(audit_path, "xt", encoding="utf-8") as journal:
            for candidate, path in zip(candidates, paths):
                if time.monotonic() - checked_at > 10:
                    opened = inode_provider()
                    checked_at = time.monotonic()
                before = path.stat()
                if not stat.S_ISREG(before.st_mode):
                    raise RuntimeError(
                        f"Recording is no longer a regular file: {candidate.relative_path}"
                    )
                if (
                    before.st_dev,
                    before.st_ino,
                ) in opened or time.time() - before.st_mtime < minimum_age:
                    report["skipped"].append(
                        {"path": candidate.relative_path, "reason": "Open or recently modified"}
                    )
                    continue
                if before.st_size != candidate.bytes or before.st_mtime_ns != candidate.mtime_ns:
                    raise RuntimeError(f"Recording changed since review: {candidate.relative_path}")
                with path.open("rb") as contents:
                    checksum = hashlib.sha256()
                    for block in iter(lambda: contents.read(4 * 1024**2), b""):
                        checksum.update(block)
                    digest = checksum.hexdigest()
                if digest != candidate.sha256:
                    raise RuntimeError(f"Recording checksum changed: {candidate.relative_path}")
                after = path.stat()
                if (before.st_ino, before.st_size, before.st_mtime_ns) != (
                    after.st_ino,
                    after.st_size,
                    after.st_mtime_ns,
                ):
                    raise RuntimeError(
                        f"Recording changed during verification: {candidate.relative_path}"
                    )
                journal.write(
                    json.dumps(
                        {
                            "path": candidate.relative_path,
                            "bytes": candidate.bytes,
                            "sha256": digest,
                            "reason": candidate.reason,
                            "inode": before.st_ino,
                            "links_before": before.st_nlink,
                            "mtime_ns": before.st_mtime_ns,
                        }
                    )
                    + "\n"
                )
                journal.flush()
                path.unlink()
                report["files_removed"] += 1
                report["logical_bytes_removed"] += candidate.bytes
                if before.st_nlink == 1:
                    report["allocated_bytes_released"] += before.st_blocks * 512
        report["state"] = "complete"
    except (OSError, ValueError, RuntimeError) as error:
        report.update(state="failed", error=str(error))
        raise
    finally:
        report.update(
            finished_wall_time=time.time(),
            available_after=os.statvfs(root).f_bavail * os.statvfs(root).f_frsize,
        )
        write_json_atomic(report_path, report)
    return report
