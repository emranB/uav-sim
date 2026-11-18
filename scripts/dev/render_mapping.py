"""Render an estimated map offscreen from actual mapping output artifacts."""

import argparse
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from control_panel.sensors.mapping import RecordedMapDialog
from desktop.theme import apply_theme


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    arguments = parser.parse_args()
    application = QApplication([])
    apply_theme(application)
    dialog = RecordedMapDialog(arguments.source)

    def save() -> None:
        arguments.destination.parent.mkdir(parents=True, exist_ok=True)
        accepted = dialog.grab().save(str(arguments.destination))
        dialog.close()
        application.exit(0 if accepted else 1)

    dialog.map_presented.connect(lambda: QTimer.singleShot(50, save))
    dialog.load_failed.connect(lambda message: application.exit(1))
    QTimer.singleShot(10000, lambda: application.exit(1))
    dialog.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
