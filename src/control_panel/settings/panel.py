"""Persistent desktop preferences and current deployment diagnostics."""

from typing import TYPE_CHECKING

from PySide6.QtWidgets import QFormLayout, QLabel, QSpinBox

from common.configuration.preferences import DEFAULT_PREFERENCES
from control_panel.form import PanelView

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class SettingsPanel(PanelView):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__(client)
        self.fields = {}
        fields = QFormLayout()
        for name, title, minimum, maximum in (
            ("poll_interval_ms", "Telemetry polling (ms)", 250, 2000),
            ("log_rows", "Visible log rows", 100, 5000),
            ("map_width_percent", "QGC width (%)", 60, 85),
        ):
            widget = QSpinBox()
            widget.setRange(minimum, maximum)
            widget.setValue(client.preferences[name])
            self.fields[name] = widget
            fields.addRow(title, widget)
        self.form.addLayout(fields)
        self.button("Apply and save preferences", self.apply)
        self.button("Restore default preferences", self.defaults)
        self.runtime = QLabel()
        self.runtime.setWordWrap(True)
        self.form.addWidget(self.runtime)
        self.button("Refresh runtime diagnostics", lambda: client.fetch("capabilities"))
        self.location = QLabel()
        self.location.setWordWrap(True)
        self.form.addWidget(self.location)
        self.finish()
        client.resource_received.connect(self.resource)
        client.fetch("capabilities")

    def apply(self) -> None:
        try:
            self.client.save_preferences(
                {name: widget.value() for name, widget in self.fields.items()}
            )
            self.status.setText("Preferences saved and applied")
        except (OSError, ValueError) as error:
            self.status.setText(str(error))

    def defaults(self) -> None:
        for name, value in DEFAULT_PREFERENCES.items():
            self.fields[name].setValue(value)
        self.apply()

    def resource(self, name: str, payload: dict) -> None:
        if name == "capabilities":
            runtime = payload["requested_runtime"]
            self.runtime.setText(
                f"Renderer: {runtime['render_mode']}\nHeadless: {runtime['headless']}\n"
                f"ROS domain: {runtime['ros_domain_id']}\nDDS connected: {payload['ros_connected']}\n"
                "GPU, display and middleware choices are deployment settings applied at startup."
            )
        if name == "workspace":
            world = payload["world"]
            self.location.setText(
                f"Start location: {world['name']}\n"
                f"Latitude / longitude: {world['latitude_deg']}, {world['longitude_deg']}\n"
                f"World reference elevation: {world['elevation_m']} m\n"
                "The physical world is flat; satellite imagery does not supply 3D terrain."
            )
