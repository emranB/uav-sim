"""Owned integration fixture: inject ENOSPC into vehicle1 telemetry, not the host disk.

Run only as the control-panel entry point for the declared storage-negative check.
The fault flag is test-owned; normal production startup never imports this module.
"""

import errno
from pathlib import Path
from unittest.mock import patch

from common.logging.storage import BoundedRecording
from common.simulation_manager.server import main

FAULT_FLAG = Path("/runtime/storage-recording-fault")


class FullRecordingStream:
    """A file-like test boundary that returns the real filesystem failure errno."""

    def write(self, text: str) -> int:
        raise OSError(errno.ENOSPC, "Injected telemetry recording disk full")

    def close(self) -> None:
        pass


class FaultRecording(BoundedRecording):
    """Keep the real receiver and acknowledgement path while failing one writer."""

    def write(self, text: str) -> bool:
        if (
            FAULT_FLAG.exists()
            and self.path.parent == Path("/workspace/outputs/telemetry")
            and self.error is None
        ):
            if self._stream is not None:
                self._stream.close()
            self._stream = FullRecordingStream()
        return super().write(text)


if __name__ == "__main__":
    with patch("integrations.mavlink.connection.BoundedRecording", FaultRecording):
        main()
