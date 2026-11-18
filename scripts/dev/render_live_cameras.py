"""Render the real Sensors panel offscreen after both live camera images arrive."""

import argparse
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from control_panel.sensors.panel import SensorsPanel
from desktop.client import SimulationClient
from desktop.theme import apply_theme


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    application = QApplication([])
    apply_theme(application)
    client = SimulationClient()
    panel = SensorsPanel(client)
    panel.resize(560, 1050)
    panel.show()
    sides = set()
    saved = False

    def save() -> None:
        nonlocal saved
        if saved:
            return
        saved = True
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        accepted = panel.grab().save(str(arguments.output))
        client.close()
        panel.close()
        application.exit(0 if accepted else 1)

    def received(side: str, contents: bytes) -> None:
        sides.add(side)
        if sides == {"left", "right"}:
            QTimer.singleShot(50, save)

    client.camera_image_received.connect(received)
    QTimer.singleShot(15000, lambda: application.exit(1))
    result = application.exec()
    client.close()
    panel.close()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
