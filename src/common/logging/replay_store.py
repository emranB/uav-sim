"""Sealed replay evidence and explicit retention, separate from diagnostic recordings."""

import fcntl
import hashlib
import json
import re
import shutil
import threading
import time
import uuid
from pathlib import Path
from typing import TextIO

from common.interfaces.replay import ReplayCaptureLimits
from common.logging.storage import ensure_capacity, write_json_atomic


class ReplayCaptureStore:
    """Own only replay packages; never remove source telemetry, missions or experiments."""

    def __init__(self, directory: Path, limits: ReplayCaptureLimits) -> None:
        self.directory = directory
        self.limits = limits
        self._lock = threading.RLock()
        self._protected: set[str] = set()
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "retained").mkdir(exist_ok=True)

    def path(self, capture_id: str) -> Path:
        """Resolve an opaque capture identifier without accepting filesystem paths."""
        if not isinstance(capture_id, str) or not re.fullmatch(r"capture-[a-f0-9]{32}", capture_id):
            raise ValueError("Choose a replay capture from the inventory")
        path = self.directory / capture_id
        if path.is_symlink() or path.resolve().parent != self.directory.resolve():
            raise ValueError("Replay capture is outside its store")
        return path

    def allocate(self) -> tuple[str, Path, TextIO]:
        """Admit a finite package and lock its draft against abrupt-shutdown recovery."""
        with self._lock:
            self.prune()
            used = sum(
                item.stat().st_size
                for item in self.directory.rglob("*")
                if item.is_file() and not item.is_symlink()
            )
            if used + self.limits.maximum_bytes > self.limits.maximum_store_bytes:
                raise OSError("Replay store budget reached; retained evidence was preserved")
            ensure_capacity(self.directory, self.limits.maximum_bytes)
            capture_id = f"capture-{uuid.uuid4().hex}"
            path = self.path(capture_id)
            path.mkdir()
            lease = (path / "owner.lock").open("a+", encoding="utf-8")
            fcntl.flock(lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                write_json_atomic(
                    path / "draft.json",
                    {
                        "schema_version": 1,
                        "capture_id": capture_id,
                        "state": "starting",
                        "created_wall_time": time.time(),
                    },
                )
            except OSError:
                lease.close()
                shutil.rmtree(path)
                raise
            return capture_id, path, lease

    def seal(self, capture_id: str, manifest: dict) -> dict:
        """Publish a final manifest once; subsequent Save/Keep changes only retention."""
        with self._lock:
            path = self.path(capture_id)
            if (path / "manifest.json").exists():
                raise RuntimeError("Replay capture is already finalized and immutable")
            files = {}
            for candidate in sorted(path.glob("*.jsonl")):
                if candidate.is_symlink():
                    raise ValueError("Replay evidence cannot be a symbolic link")
                files[candidate.name] = {
                    "bytes": candidate.stat().st_size,
                    "sha256": self._digest(candidate),
                }
            context = path / "context.json"
            if context.exists():
                if context.is_symlink():
                    raise ValueError("Replay context cannot be a symbolic link")
                files[context.name] = {
                    "bytes": context.stat().st_size,
                    "sha256": self._digest(context),
                }
            payload = {
                **manifest,
                "schema_version": 1,
                "capture_id": capture_id,
                "data_kind": "observed trajectories and events",
                "camera_video_recorded": False,
                "files": files,
                "finalized_wall_time": time.time(),
            }
            write_json_atomic(path / "manifest.json", payload)
            digest = self._digest(path / "manifest.json")
            write_json_atomic(path / "sealed.json", {"manifest_sha256": digest})
            (path / "draft.json").unlink(missing_ok=True)
            for name in (*files, "manifest.json", "sealed.json"):
                (path / name).chmod(0o444)
            return payload

    def manifest(self, capture_id: str, *, verify_data: bool = False) -> dict:
        """Read sealed evidence and optionally verify every retained render input."""
        path = self.path(capture_id)
        payload = self._read(path / "manifest.json")
        expected = self._read(path / "sealed.json")["manifest_sha256"]
        if self._digest(path / "manifest.json") != expected:
            raise ValueError("Saved replay manifest changed after finalization")
        if verify_data:
            for name, recorded in payload["files"].items():
                if Path(name).name != name or name not in {
                    p.name for p in path.iterdir() if p.is_file() and not p.is_symlink()
                }:
                    raise ValueError("Saved replay input is missing or outside the package")
                candidate = path / name
                if (
                    candidate.stat().st_size != recorded["bytes"]
                    or self._digest(candidate) != recorded["sha256"]
                ):
                    raise ValueError(f"Saved replay input changed: {name}")
        return payload

    def keep(self, capture_id: str) -> dict:
        """Pin existing evidence; this operation never starts capture or changes its interval."""
        with self._lock:
            self.manifest(capture_id)
            write_json_atomic(
                self.directory / "retained" / f"{capture_id}.json",
                {
                    "capture_id": capture_id,
                    "retained_wall_time": time.time(),
                    "manifest_sha256": self._digest(self.path(capture_id) / "manifest.json"),
                },
            )
            return {"accepted": True, "capture_id": capture_id, "retained": True}

    def inventory(self, *, limit: int | None = 100) -> list[dict]:
        """Return compact package metadata without reading trajectory arrays."""
        rows = []
        for path in sorted(self.directory.glob("capture-*/manifest.json"), reverse=True):
            try:
                manifest = self.manifest(path.parent.name)
                rows.append(
                    {
                        key: manifest.get(key)
                        for key in (
                            "capture_id",
                            "state",
                            "vehicle_ids",
                            "vehicle_names",
                            "duration_wall_seconds",
                            "partial_mission",
                            "renderable",
                            "error",
                            "finalized_wall_time",
                        )
                    }
                    | {
                        "retained": (
                            self.directory / "retained" / f"{path.parent.name}.json"
                        ).exists(),
                        "path": str(path.parent),
                    }
                )
            except (OSError, ValueError, KeyError) as error:
                rows.append({"capture_id": path.parent.name, "state": "error", "error": str(error)})
        ordered = sorted(rows, key=lambda row: row.get("finalized_wall_time", 0), reverse=True)
        return ordered if limit is None else ordered[:limit]

    def protect(self, capture_id: str) -> None:
        """Prevent expiry while an offline job reads a finalized package."""
        with self._lock:
            self.manifest(capture_id)
            self._protected.add(capture_id)

    def release(self, capture_id: str) -> None:
        with self._lock:
            self._protected.discard(capture_id)

    def prune(self) -> None:
        """Expire only excess finalized, unretained packages owned by this store."""
        with self._lock:
            candidates = [
                row
                for row in self.inventory(limit=None)
                if not row.get("retained")
                and row["capture_id"] not in self._protected
                and row.get("state") not in {"interrupted", "error"}
            ]
            for row in candidates[self.limits.unretained_capture_count :]:
                path = self.path(row["capture_id"])
                if path.exists() and not path.is_symlink():
                    shutil.rmtree(path)

    def recover_interrupted(self) -> list[str]:
        """Retain orphaned drafts without claiming a landed or completed mission."""
        recovered = []
        for draft in self.directory.glob("capture-*/draft.json"):
            path = self.path(draft.parent.name)
            if (path / "manifest.json").exists():
                continue
            with (path / "owner.lock").open("a+", encoding="utf-8") as lease:
                try:
                    fcntl.flock(lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    continue
                previous = self._read(draft)
                self.seal(
                    path.name,
                    {
                        **previous,
                        "state": "interrupted",
                        "partial_mission": True,
                        "renderable": False,
                        "error": (
                            "Capture process ended before interval finalization; flight state unknown"
                        ),
                    },
                )
                self.keep(path.name)
                recovered.append(path.name)
        return recovered

    @staticmethod
    def _read(path: Path) -> dict:
        if path.is_symlink():
            raise ValueError("Replay metadata cannot be a symbolic link")
        if path.stat().st_size > 1024**2:
            raise ValueError("Replay metadata exceeds its 1 MiB limit")
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("Replay metadata must be an object")
        return payload

    @staticmethod
    def _digest(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024**2), b""):
                digest.update(chunk)
        return digest.hexdigest()
