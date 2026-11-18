"""Acknowledge only declared UAV process owners through their transient status files."""

import json
import math
import time
from collections.abc import Callable
from pathlib import Path

from common.lifecycle.supervisor import SERVICE_ROLES
from common.logging.storage import write_json_atomic


class RuntimeServiceOwners:
    """No Docker/host access: owners restart only their own child process groups."""

    def __init__(
        self,
        roles: tuple[str, ...],
        root: Path = Path("/runtime/services"),
        requests: Path = Path("/runtime/service-requests"),
    ) -> None:
        if len(set(roles)) != len(roles) or not set(roles).issubset(SERVICE_ROLES):
            raise ValueError("Restore roles must be unique declared UAV services")
        self.roles, self.root = roles, root
        self.requests = requests

    def state(self, role: str) -> dict:
        if role not in self.roles:
            raise ValueError("Service is outside the configured UAV restore scope")
        path = self.root / role / "status.json"
        if path.is_symlink() or path.stat().st_size > 4096:
            raise ValueError(f"{role}: invalid process owner status")
        status = json.loads(path.read_text())
        age = time.time() - status["received_wall_time"]
        if not math.isfinite(age) or not 0 <= age < 3 or status.get("running") is not True:
            raise RuntimeError(f"{role}: a fresh running process owner is unavailable")
        if not isinstance(status.get("generation"), str) or not status["generation"]:
            raise ValueError(f"{role}: process generation is unavailable")
        return status

    def restart(self, role: str, safe: Callable[[], None]) -> str:
        """Verify safety immediately before requesting restart, then require a new owner generation."""
        previous = self.state(role)["generation"]
        safe()
        # The API owns its request spool; each supervisor owns its status metadata.
        self.requests.mkdir(parents=True, exist_ok=True)
        write_json_atomic(
            self.requests / f"{role}.restart",
            {
                "previous_generation": previous,
                "requested_wall_time": time.time(),
            },
        )
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            try:
                current = self.state(role)
                if current["generation"] != previous:
                    return current["generation"]
            except (FileNotFoundError, RuntimeError):
                pass
            time.sleep(0.1)
        raise TimeoutError(f"{role}: owner did not acknowledge a new process generation")
