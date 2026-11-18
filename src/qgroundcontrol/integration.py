"""Application-level coordination with the independently owned QGC process."""

from pathlib import Path


def reset_view() -> None:
    """Restart the live QGC view without removing saved settings or mission files."""
    try:
        Path("/runtime/qgc.restart").touch()
    except OSError as error:
        raise RuntimeError("Could not request a QGroundControl view reset") from error
