"""Render offscreen presentation evidence using actual recorded sensor images."""

import argparse
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from control_panel.sensors.recording import StereoRecordingDialog
from desktop.theme import apply_theme


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    application = QApplication([])
    apply_theme(application)
    dialog = StereoRecordingDialog(arguments.source)

    def save_frame(index: int) -> None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        saved = dialog.grab().save(str(arguments.output))
        dialog.close()
        application.exit(0 if saved else 1)

    dialog.frame_presented.connect(save_frame)
    dialog.load_failed.connect(lambda message: application.exit(1))
    QTimer.singleShot(10000, lambda: application.exit(1))
    dialog.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
