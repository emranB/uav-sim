"""Retire only descendants of one Linux UAV service owner, including orphaned children."""

import ctypes
import os
import signal
import time
from pathlib import Path


def enable_service_subreaper() -> None:
    """Adopt a nested launcher's orphaned children inside this dedicated supervisor."""
    library = ctypes.CDLL(None, use_errno=True)
    if library.prctl(36, 1, 0, 0, 0) != 0:  # Linux PR_SET_CHILD_SUBREAPER.
        error = ctypes.get_errno()
        raise OSError(error, "Cannot establish UAV service child ownership")


def descendant_process_ids(parent_pid: int) -> set[int]:
    """Read this owner's descendants; reject unexpectedly large service process trees."""
    found: set[int] = set()
    pending = [parent_pid]
    while pending:
        parent = pending.pop()
        try:
            children = Path(f"/proc/{parent}/task/{parent}/children").read_text().split()
        except FileNotFoundError:
            continue
        for encoded in children:
            child = int(encoded)
            if child not in found:
                found.add(child)
                pending.append(child)
        if len(found) > 256:
            raise RuntimeError("UAV service exceeded its 256-child ownership bound")
    return found


def process_start_ticks(pid: int) -> int:
    """Return Linux process birth ticks, distinguishing reused numerical process IDs."""
    fields = Path(f"/proc/{pid}/stat").read_text().rpartition(") ")[2].split()
    return int(fields[19])


def retire_service_children(grace_seconds: float = 2.0) -> None:
    """Reap adopted descendants before acknowledging restart; never inspect other owners."""
    owner = os.getpid()

    def remaining() -> dict[int, int]:
        live = {}
        for pid in descendant_process_ids(owner):
            try:
                reaped, _ = os.waitpid(pid, os.WNOHANG)
                if reaped:
                    continue
            except ChildProcessError:
                pass
            try:
                live[pid] = process_start_ticks(pid)
            except FileNotFoundError:
                pass
        return live

    def send(identities: dict[int, int], signum: int) -> None:
        for pid, birth in identities.items():
            try:
                if process_start_ticks(pid) == birth:
                    os.kill(pid, signum)
            except ProcessLookupError:
                pass
            except FileNotFoundError:
                pass

    children = remaining()
    send(children, signal.SIGTERM)
    deadline = time.monotonic() + grace_seconds
    while children and time.monotonic() < deadline:
        time.sleep(0.05)
        children = remaining()
    send(children, signal.SIGKILL)
    deadline = time.monotonic() + 2
    while children and time.monotonic() < deadline:
        time.sleep(0.05)
        children = remaining()
    if children:
        raise TimeoutError("Owned UAV service descendants did not terminate after bounded shutdown")
