"""QGroundControl presentation boundary within the desktop workspace."""

from pathlib import Path

from desktop.viewer import ViewerPanel


class QGroundControlPanel(ViewerPanel):
    def __init__(self) -> None:
        super().__init__("QGroundControl", Path("/runtime/QGroundControl.window"))
