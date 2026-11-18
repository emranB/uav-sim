"""Launch QGC separately from the desktop and simulator server."""

import logging
import os
import tempfile
import time
from pathlib import Path

from PySide6.QtCore import QSettings

from common.configuration.runtime import RuntimeSettings
from common.configuration.world import WorldLocation
from common.lifecycle.process import run_process


def remove_stale_instance_locks(directory: Path) -> None:
    """Remove abandoned QGC locks before a new child can reuse their recorded PID.

    Only the launcher's own regular lock files in its private temporary directory
    are considered. A live PID or a file owned by another user remains untouched.
    """
    for path in list(directory.glob("qgc-*.lock"))[:16]:
        if path.is_symlink() or path.stat().st_uid != os.getuid():
            continue
        with path.open("rb") as stream:
            contents = stream.read(1025)
        if len(contents) > 1024:
            raise OSError(f"QGroundControl instance lock exceeds its expected size: {path}")
        try:
            process_id = int(contents.splitlines()[0])
            if process_id <= 0:
                raise ValueError("Expected a positive process ID")
        except (ValueError, IndexError) as error:
            raise OSError(f"QGroundControl instance lock is malformed: {path}") from error
        try:
            os.kill(process_id, 0)
        except ProcessLookupError:
            path.unlink()
            logging.warning("Removed abandoned QGroundControl instance lock %s", path.name)
        except PermissionError:
            # An unverifiable process must not be treated as a stale instance.
            continue


def prepare_preferences(path: Path) -> None:
    """Preserve user settings while repairing unusable embedded-window geometry."""
    preferences = QSettings(str(path), QSettings.IniFormat)
    for key, value in {"firstRunPromptIdsShown": "3", "preferredFirmwareClass": 12}.items():
        if not preferences.contains(key):
            preferences.setValue(key, value)
    # Qt can persist a tiny native size while an embedded window is being detached.
    # Restore a usable startup surface so the host can discover and embed it again.
    for dimension, minimum in (("width", 800), ("height", 600)):
        key = f"MainWindowState/{dimension}"
        try:
            size = preferences.value(key, minimum, type=int)
        except (TypeError, ValueError):
            size = minimum
        preferences.setValue(key, max(size, minimum))
    preferences.sync()
    if preferences.status() != QSettings.Status.NoError:
        raise OSError(f"QGroundControl preferences are not writable: {path.parent}")


def main() -> int:
    settings = RuntimeSettings.from_environment(os.environ)
    if settings.headless:
        return 0
    os.environ.update(settings.rendering_environment(os.environ))
    failures = 0
    while failures < 5:
        remove_stale_instance_locks(Path(tempfile.gettempdir()))
        prepare_preferences(Path.home() / ".config/QGroundControl/QGroundControl.ini")
        location = WorldLocation.configured()
        preferences = QSettings(
            str(Path.home() / ".config/QGroundControl/QGroundControl.ini"), QSettings.IniFormat
        )
        preferences.setValue("FlightMapPosition/Latitude", location.latitude_deg)
        preferences.setValue("FlightMapPosition/Longitude", location.longitude_deg)
        preferences.sync()
        result = run_process(
            ["/opt/qgroundcontrol/AppRun"],
            window_name="QGroundControl",
            restart_file=Path("/runtime/qgc.restart"),
            log_source="qgroundcontrol",
        )
        if result == 75:
            continue
        if result == 0:
            return 0
        failures += 1
        logging.error("QGroundControl exited with %s; restart attempt %s", result, failures)
        time.sleep(2)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
