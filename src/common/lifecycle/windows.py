"""Verify native view publication against its current supervised process identity."""

import json
from pathlib import Path


def current_service_window(window: Path, service_owner: dict, restarted_wall_time: float) -> bool:
    """Reject stale, foreign or unverifiable windows even when a title/ID was republished."""
    try:
        metadata = window.with_suffix(".owner.json")
        if (
            window.is_symlink()
            or metadata.is_symlink()
            or window.stat().st_size > 64
            or metadata.stat().st_size > 4096
        ):
            return False
        view = json.loads(metadata.read_text())
        identity = service_owner.get("pid"), service_owner.get("process_start_ticks")
        return (
            all(type(value) is int and value > 0 for value in identity)
            and isinstance(service_owner.get("machine"), str)
            and bool(service_owner["machine"])
            and view.get("schema_version") == 1
            and window.stat().st_mtime >= restarted_wall_time
            and int(window.read_text()) == view["window_id"]
            and view["published_wall_time"] >= restarted_wall_time
            and view["owner_pid"] == identity[0]
            and view["owner_start_ticks"] == identity[1]
            and view["machine"] == service_owner["machine"]
            and type(view["native_pid"]) is int
            and view["native_pid"] > 0
            and type(view["native_start_ticks"]) is int
            and view["native_start_ticks"] > 0
        )
    except (OSError, ValueError, KeyError, TypeError):
        return False
