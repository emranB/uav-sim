"""Named missions, import/export and experiment configuration restoration."""

import json
from pathlib import Path
from typing import TYPE_CHECKING

from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QLabel,
    QLineEdit,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from control_panel.form import PanelView

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class MissionsPanel(PanelView):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__(client)
        self.name = QLineEdit()
        self.name.setPlaceholderText("Mission or experiment name")
        self.form.addWidget(self.name)
        sections = QTabWidget()
        self.form.addWidget(sections)

        def page(title: str) -> QVBoxLayout:
            widget = QWidget()
            sections.addTab(widget, title)
            return QVBoxLayout(widget)

        missions = page("Missions")
        self.missions = QComboBox()
        missions.addWidget(self.missions)
        self.button(
            "Save current vehicle mission",
            lambda: self.send("missions/save", {"name": self.name.text().strip()}),
            missions,
        )
        self.button(
            "Load selected mission",
            lambda: self.send("missions/load", {"name": self.missions.currentText()}),
            missions,
        )
        self.button("Execute uploaded mission", lambda: self.send("start"), missions)
        self.button("Import .plan file", self.import_plan, missions)
        self.button(
            "Export selected mission",
            lambda: self.send("missions/export", {"name": self.missions.currentText()}),
            missions,
        )
        self.button(
            "Archive selected mission",
            lambda: self.send("missions/archive", {"name": self.missions.currentText()}),
            missions,
        )
        missions.addStretch()

        experiments = page("Experiments")
        self.experiments = QComboBox()
        experiments.addWidget(self.experiments)
        self.button(
            "Save current configuration",
            lambda: self.send("experiments/save", {"name": self.name.text().strip()}),
            experiments,
        )
        self.button(
            "Restore selected experiment",
            lambda: self.send("experiments/load", {"name": self.experiments.currentText()}),
            experiments,
        )
        self.button(
            "Archive selected experiment",
            lambda: self.send("experiments/archive", {"name": self.experiments.currentText()}),
            experiments,
        )
        explanation = QLabel(
            "Restores the mission, sensor settings, faults and vehicle limits. "
            "Repeatability is not assumed; running process state is not restored."
        )
        explanation.setWordWrap(True)
        experiments.addWidget(explanation)
        experiments.addStretch()

        comparison = page("Compare")
        self.left_recording = QComboBox()
        self.right_recording = QComboBox()
        comparison.addWidget(self.left_recording)
        comparison.addWidget(self.right_recording)
        self.button(
            "Compare recording statistics",
            lambda: self.send(
                "recordings/compare",
                {
                    "left": self.left_recording.currentText(),
                    "right": self.right_recording.currentText(),
                },
            ),
            comparison,
        )
        self.comparison = QLabel("Whole-recording statistics include idle time and all missions.")
        self.comparison.setWordWrap(True)
        comparison.addWidget(self.comparison)
        comparison.addStretch()
        self.button("Refresh saved records", self.refresh)
        self.finish()
        client.resource_received.connect(self.resource)
        client.operation_result.connect(self.completed)
        self.refresh()

    def reset_panel(self) -> None:
        super().reset_panel()
        self.name.clear()
        self.comparison.setText("Whole-recording statistics include idle time and all missions.")
        self.refresh()

    def refresh(self) -> None:
        self.client.fetch("missions")
        self.client.fetch("experiments")
        self.client.fetch("recordings")

    def resource(self, name: str, payload: dict) -> None:
        if name == "recordings":
            filenames = [record["name"] for record in payload["telemetry"]]
            for widget in (self.left_recording, self.right_recording):
                widget.clear()
                widget.addItems(filenames)
            self.left_recording.setCurrentIndex(max(0, len(filenames) - 2))
            self.right_recording.setCurrentIndex(max(0, len(filenames) - 1))
            return
        widget = {"missions": self.missions, "experiments": self.experiments}.get(name)
        if widget is None:
            return
        selected = widget.currentText()
        widget.clear()
        widget.addItems([item["name"] for item in payload["items"]])
        if widget.findText(selected) >= 0:
            widget.setCurrentText(selected)

    def completed(self, operation: str, payload: dict) -> None:
        if operation == "recordings/compare":
            comparison = payload["comparison"]
            lines = []
            for side in ("left", "right"):
                stats = comparison[side]
                lines.append(
                    f"{side.title()}: {stats['position_samples']} positions; "
                    f"max altitude {stats['maximum_altitude_m']:.1f} m; "
                    f"max speed {stats['maximum_speed_m_s']:.1f} m/s"
                )
            lines.append(f"Altitude difference: {comparison['delta']['maximum_altitude_m']:.1f} m")
            self.comparison.setText("\n".join(lines))
        if operation.startswith(("missions/", "experiments/")):
            self.refresh()
        if operation == "missions/export":
            filename, _ = QFileDialog.getSaveFileName(
                self, "Export mission", "mission.plan", "Mission (*.plan)"
            )
            if filename:
                try:
                    Path(filename).write_text(
                        json.dumps(payload["mission"], indent=2) + "\n", encoding="utf-8"
                    )
                    self.status.setText("Mission exported")
                except OSError as error:
                    self.status.setText(str(error))

    def import_plan(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(self, "Import mission", "", "Mission (*.plan)")
        if filename:
            try:
                self.send("mission", json.loads(Path(filename).read_text(encoding="utf-8")))
            except (OSError, ValueError) as error:
                self.status.setText(str(error))
