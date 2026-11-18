"""Read-only native widget state for the opt-in full-window recording acceptance run."""

import time
from pathlib import Path

from PySide6.QtCore import QObject, QPoint, QTimer
from PySide6.QtWidgets import QAbstractButton, QComboBox, QMenu, QWidget

from common.logging.storage import write_json_atomic


class NativeControlState(QObject):
    """Publish actual widget geometry; pointer actions remain external native X11 clicks."""

    names = {
        "flightTargetSelector",
        "quick_start",
        "quick_land",
        "replayStart",
        "replayStop",
        "replayKeep",
        "recordedRunSelection",
        "playRecordedReplay",
        "gnssTargetSelection",
        "gnssEnableInput",
        "gnssApplyInput",
        "stereoReferenceChoice",
    }

    def __init__(self, window: QWidget) -> None:
        super().__init__(window)
        self.window = window
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish)
        self.timer.start(250)

    def publish(self) -> None:
        controls = {}
        menus = []
        for widget in self.window.findChildren(QWidget):
            if isinstance(widget, QMenu) and widget.isVisible():
                entries = []
                for action in widget.actions():
                    rect = widget.actionGeometry(action)
                    point = widget.mapToGlobal(rect.center())
                    entries.append(
                        {
                            "text": action.text(),
                            "value": action.data(),
                            "center": [point.x(), point.y()],
                            "enabled": action.isEnabled(),
                            "checked": action.isChecked(),
                        }
                    )
                menus.append({"owner": widget.parentWidget().objectName(), "entries": entries})
            name = widget.objectName()
            if name not in self.names:
                continue
            origin = widget.mapToGlobal(QPoint(0, 0))
            row = {
                "rect": [origin.x(), origin.y(), widget.width(), widget.height()],
                "visible": widget.isVisible(),
                "enabled": widget.isEnabled(),
            }
            if isinstance(widget, QAbstractButton):
                row.update(text=widget.text(), checked=widget.isChecked())
            if isinstance(widget, QComboBox):
                row.update(
                    selected=widget.currentData(),
                    text=widget.currentText(),
                    items=[
                        {"text": widget.itemText(index), "value": widget.itemData(index)}
                        for index in range(widget.count())
                    ],
                )
            controls[name] = row
        write_json_atomic(
            Path("/runtime/Desktop.controls.json"),
            {
                "wall_time": time.time(),
                "window_id": int(self.window.winId()),
                "inspect": self.window.client.vehicle_id,
                "mission_targets": list(self.window.client.target_ids),
                "controls": controls,
                "menus": menus,
            },
        )
