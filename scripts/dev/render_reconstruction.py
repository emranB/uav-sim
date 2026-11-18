"""Render the actual live reconstruction widget without opening a native window."""

import argparse
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from control_panel.sensors.reconstruction import LiveReconstructionView
from desktop.client import SimulationClient
from desktop.theme import apply_theme


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    arguments = parser.parse_args()
    application = QApplication([])
    apply_theme(application)
    client = SimulationClient()
    client.timer.stop()
    view = LiveReconstructionView(client)
    view.resize(700, 520)
    view.show()
    succeeded = False

    def capture() -> None:
        nonlocal succeeded
        succeeded = bool(view.snapshot.get("ready"))
        arguments.destination.parent.mkdir(parents=True, exist_ok=True)
        succeeded = view.grab().save(str(arguments.destination)) and succeeded
        view.close()
        client.close()
        application.quit()

    QTimer.singleShot(5000, capture)
    application.exec()
    return 0 if succeeded else 1


if __name__ == "__main__":
    raise SystemExit(main())
