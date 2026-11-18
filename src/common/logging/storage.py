"""Storage admission, atomic publication and bounded append-only recording."""

import errno
import json
import os
import sys
import uuid
from pathlib import Path
from typing import TextIO

DEFAULT_RESERVE_BYTES = 2 * 1024**3
DEFAULT_RECORDING_LIMIT_BYTES = 256 * 1024**2


class StorageCapacityError(OSError):
    """Admission failed before consuming the space reserved for safe shutdown."""


def configured_reserve_bytes() -> int:
    """Return the deployment's free-space reserve, in bytes (64 MiB..16 GiB)."""
    value = int(os.environ.get("UAV_STORAGE_RESERVE_BYTES", str(DEFAULT_RESERVE_BYTES)))
    if not 64 * 1024**2 <= value <= 16 * 1024**3:
        raise ValueError("UAV_STORAGE_RESERVE_BYTES must be in 64 MiB..16 GiB")
    return value


def ensure_capacity(
    directory: Path, required_bytes: int = 0, reserve_bytes: int | None = None
) -> dict:
    """Check writable-user free bytes and inodes; do not count privileged reserved blocks.

    This is admission, not a reservation against other applications. Every writer must
    recheck; failures must remain observable while transports continue safe shutdown.
    """
    if type(required_bytes) is not int or required_bytes < 0:
        raise ValueError("Required storage must be nonnegative integer bytes")
    reserve = configured_reserve_bytes() if reserve_bytes is None else reserve_bytes
    if type(reserve) is not int or reserve < 0:
        raise ValueError("Storage reserve must be nonnegative integer bytes")
    existing = directory
    while not existing.exists() and existing != existing.parent:
        existing = existing.parent
    state = os.statvfs(existing)
    available = state.f_bavail * state.f_frsize
    if available < required_bytes + reserve:
        raise StorageCapacityError(
            errno.ENOSPC,
            f"{directory}: insufficient storage: required {required_bytes + reserve} bytes "
            f"including {reserve} reserve, available {available} bytes",
        )
    if state.f_favail < 1024:
        raise StorageCapacityError(
            errno.ENOSPC, f"{directory}: insufficient free inodes: {state.f_favail}; required 1024"
        )
    return {
        "available_bytes": available,
        "reserve_bytes": reserve,
        "required_bytes": required_bytes,
        "available_inodes": state.f_favail,
    }


def write_atomic(path: Path, contents: bytes, reserve_bytes: int | None = None) -> None:
    """Replace only after a complete write; preserve the last good file on failure.

    Rename is atomic on the same filesystem. This does not promise power-loss durability.
    """
    ensure_capacity(path.parent, len(contents), reserve_bytes)
    temporary = path.with_name(f".{path.name}-{uuid.uuid4().hex}.tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_bytes(contents)
        temporary.replace(path)
    except OSError as error:
        raise OSError(error.errno, f"Cannot publish {path}: {error}") from error
    finally:
        temporary.unlink(missing_ok=True)


def write_json_atomic(path: Path, payload: dict) -> None:
    """Publish a critical run ledger using the shutdown reserve, preserving prior evidence."""
    write_atomic(
        path, (json.dumps(payload, indent=2, allow_nan=False) + "\n").encode(), reserve_bytes=0
    )


def ensure_run_capacity(directory: Path) -> dict:
    """Admit execution with a declared 2 GiB run allowance plus the free-space reserve."""
    required = int(os.environ.get("UAV_RUN_RECORDING_BUDGET_BYTES", str(2 * 1024**3)))
    if not 32 * 1024**2 <= required <= 32 * 1024**3:
        raise ValueError("UAV_RUN_RECORDING_BUDGET_BYTES must be in 32 MiB..32 GiB")
    return ensure_capacity(directory, required)


class BoundedRecording:
    """Stop recording visibly on capacity/write failure while callers keep draining input.

    Existing lines are retained, never rotated or deleted. A failed recorder stays failed
    until its owner explicitly creates a new recording after restoring capacity.
    """

    def __init__(self, path: Path, maximum_bytes: int = DEFAULT_RECORDING_LIMIT_BYTES) -> None:
        if type(maximum_bytes) is not int or maximum_bytes <= 0:
            raise ValueError("Recording limit must be positive integer bytes")
        self.path = path
        self.maximum_bytes = maximum_bytes
        self.bytes_written = 0
        self.error: str | None = None
        self._stream: TextIO | None = None

    def __enter__(self) -> "BoundedRecording":
        return self

    def __exit__(self, *exception: object) -> None:
        self.close()

    def _failed(self, error: OSError) -> None:
        if self.error is None:
            self.error = f"Recording unavailable: {self.path}: {error}"
            print(self.error, file=sys.stderr, flush=True)

    def write(self, text: str) -> bool:
        """Return false with a latched error instead of killing a transport receiver."""
        if self.error is not None:
            return False
        try:
            if self._stream is None:
                self.bytes_written = self.path.stat().st_size if self.path.exists() else 0
            size = len(text.encode("utf-8"))
            if self.bytes_written + size > self.maximum_bytes:
                raise OSError(errno.ENOSPC, f"Recording byte limit {self.maximum_bytes} reached")
            ensure_capacity(self.path.parent, size)
            if self._stream is None:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                self._stream = self.path.open("a", encoding="utf-8", buffering=1)
            self._stream.write(text)
            self.bytes_written += size
            return True
        except OSError as error:
            self._failed(error)
            return False

    def snapshot(self) -> dict:
        """Expose in-memory recording health without filesystem or transport access."""
        return {
            "available": self.error is None,
            "error": self.error,
            "bytes": self.bytes_written,
            "maximum_bytes": self.maximum_bytes,
            "path": str(self.path),
        }

    def close(self) -> None:
        if self._stream is not None:
            try:
                self._stream.close()
            except OSError as error:
                self._failed(error)
            finally:
                self._stream = None


def directory_bytes(directory: Path) -> int:
    """Measure regular output files while atomic status updates may rename temporary files.

    A disappeared pathname is re-counted at its destination on a subsequent check.
    Other stat errors remain failures. Symlinks are not followed into external stores.
    """
    total = 0
    for path in directory.rglob("*"):
        try:
            if path.is_file() and not path.is_symlink():
                total += path.stat().st_size
        except FileNotFoundError:
            # A bounded writer atomically renamed or retired this transient pathname.
            continue
    return total
