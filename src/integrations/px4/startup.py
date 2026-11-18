"""Apply explicit recording policy to a private copy of the pinned SITL startup."""

import hashlib
from pathlib import Path

from common.logging.storage import write_json_atomic


def prepare_startup_script(
    template: Path,
    run_directory: Path,
    log_mode: str,
    *,
    dds_time_source: str | None = None,
) -> Path:
    """Set native logging after its default is installed and before logger startup.

    PX4 discards an explicit parameter assignment equal to the current default.
    Its later boot-log default would therefore override an early environment
    assignment of zero. Keep the pinned template intact and record both hashes.
    An unexpected startup layout fails before launching the controller.
    """
    if log_mode not in {"armed", "boot"}:
        raise ValueError("PX4 log mode must be armed or boot")
    if dds_time_source not in {None, "simulation", "agent"}:
        raise ValueError("DDS time source must be simulation or agent")
    original = template.read_bytes()
    marker = b"param set-default SDLOG_MODE 1\n"
    if original.count(marker) != 1:
        raise RuntimeError("Pinned PX4 startup lacks its unique logging-default boundary")
    selected_mode = 0 if log_mode == "armed" else 1
    configured = original.replace(
        marker, marker + f"param set SDLOG_MODE {selected_mode}\n".encode()
    )
    if dds_time_source is not None:
        starts = [
            line
            for line in original.splitlines(keepends=True)
            if line.lstrip().startswith(b"uxrce_dds_client start ")
        ]
        if len(starts) != 1:
            raise RuntimeError("Pinned PX4 startup lacks its unique DDS-client start boundary")
        sync_enabled = int(dds_time_source == "agent")
        configured = configured.replace(
            starts[0],
            f"param set UXRCE_DDS_SYNCT {sync_enabled}\n".encode() + starts[0],
        )
    destination = run_directory / "rcS"
    destination.write_bytes(configured)
    write_json_atomic(
        run_directory / "startup-policy.json",
        {
            "schema_version": 1,
            "log_mode": log_mode,
            "sdlog_mode": selected_mode,
            "template": str(template),
            "template_sha256": hashlib.sha256(original).hexdigest(),
            "startup_script": str(destination),
            "startup_sha256": hashlib.sha256(configured).hexdigest(),
            "dds_time_source": dds_time_source,
            "uxrce_dds_synct": int(dds_time_source == "agent") if dds_time_source else None,
        },
    )
    return destination
