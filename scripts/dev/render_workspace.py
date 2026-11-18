"""Render presentation QA with labelled substitutes for external native windows."""

from pathlib import Path
from unittest.mock import patch

from PySide6.QtWidgets import QApplication, QLabel

from desktop.theme import apply_theme
from desktop.window import MainWindow
from testing.component.test_panels import RecordedClient


def main() -> None:
    application = QApplication([])
    apply_theme(application)
    client = RecordedClient()
    client.close = lambda: None
    with (
        patch("desktop.window.SimulationClient", return_value=client),
        patch(
            "desktop.layout.QGroundControlPanel",
            return_value=QLabel("QGroundControl view substitute"),
        ),
        patch("desktop.layout.GazeboPanel", return_value=QLabel("Gazebo view substitute")),
    ):
        window = MainWindow()
        window.show()
        application.processEvents()
        destination = Path("outputs/reports/ui-presentation.png")
        destination.parent.mkdir(parents=True, exist_ok=True)
        window.grab().save(str(destination))
        window.close()


if __name__ == "__main__":
    main()
