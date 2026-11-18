"""Explicit vehicle identity, transport assignments and live readiness."""

from typing import TYPE_CHECKING

from PySide6.QtWidgets import QComboBox, QDoubleSpinBox, QFormLayout, QLabel, QLineEdit

from control_panel.form import PanelView

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class VehiclesPanel(PanelView):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__(client)
        self.selector = QComboBox()
        self.selector.setToolTip("Changes the workspace-wide viewed vehicle")
        self.selector.activated.connect(
            lambda index: client.select_vehicle(
                self.selector.itemData(index),
                getattr(client, "apply_to_all", False),
            )
        )
        self.form.addWidget(self.selector)
        self.current_vehicle_id = getattr(client, "vehicle_id", "vehicle_1")
        self.name = QLineEdit()
        self.name.setPlaceholderText("Name for a new vehicle or rename")
        self.form.addWidget(self.name)
        self.model = QComboBox()
        self.model.addItem("X500 quadcopter", "x500")
        self.form.addWidget(self.model)
        self.button(
            "Add vehicle",
            lambda: self.send(
                "vehicles/add",
                {
                    "label": self.name.text(),
                    "model": self.model.currentData(),
                },
            ),
        )
        self.button(
            "Rename viewed vehicle",
            lambda: self.send(
                "vehicles/rename",
                {
                    "vehicle_id": self.current_vehicle_id,
                    "label": self.name.text(),
                },
            ),
        )
        self.button(
            "Remove viewed vehicle",
            lambda: self.send(
                "vehicles/remove",
                {
                    "vehicle_id": self.current_vehicle_id,
                },
            ),
        ).setToolTip("Requires a disarmed vehicle. Historical records are retained.")
        self.configuration = QLabel("Waiting for vehicle configuration…")
        self.configuration.setWordWrap(True)
        self.form.addWidget(self.configuration)
        self.state = QLabel()
        self.state.setWordWrap(True)
        self.form.addWidget(self.state)
        limits = QFormLayout()
        self.cruise = QDoubleSpinBox()
        self.cruise.setRange(0.5, 15)
        self.cruise.setValue(5)
        self.climb = QDoubleSpinBox()
        self.climb.setRange(0.5, 5)
        self.climb.setValue(2)
        limits.addRow("Cruise speed (m/s)", self.cruise)
        limits.addRow("Climb speed (m/s)", self.climb)
        self.form.addLayout(limits)
        self.parameters = QLabel("Read parameters to see current vehicle limits")
        self.parameters.setWordWrap(True)
        self.form.addWidget(self.parameters)
        self.button("Read vehicle parameters", lambda: self.send("vehicles/read"))
        self.button(
            "Apply speed limits",
            lambda: self.send(
                "vehicles/configure",
                {"cruise_speed_m_s": self.cruise.value(), "climb_speed_m_s": self.climb.value()},
            ),
        )
        self.button("Refresh configuration", lambda: client.fetch("workspace"))
        self.button("Stop / Land selected vehicle", lambda: self.send("stop"))
        self.finish()
        client.resource_received.connect(self.resource)
        client.telemetry_received.connect(self.telemetry)
        client.operation_result.connect(self.applied)

    def applied(self, operation: str, payload: dict) -> None:
        if operation not in {"vehicles/read", "vehicles/configure"}:
            return
        parameters = payload["parameters"]
        self.cruise.setValue(parameters["cruise_speed_m_s"])
        self.climb.setValue(parameters["climb_speed_m_s"])
        self.parameters.setText(
            f"Verified cruise: {parameters['cruise_speed_m_s']:.1f} m/s; "
            f"climb: {parameters['climb_speed_m_s']:.1f} m/s"
        )

    def reset_panel(self) -> None:
        super().reset_panel()
        self.parameters.setText("Read parameters to see current vehicle limits")

    def resource(self, name: str, payload: dict) -> None:
        if name == "vehicles":
            self.selector.blockSignals(True)
            self.selector.clear()
            for vehicle in payload["items"]:
                self.selector.addItem(vehicle["label"], vehicle["vehicle_id"])
            self.selector.setCurrentIndex(self.selector.findData(self.current_vehicle_id))
            self.selector.blockSignals(False)
        if name != "workspace":
            return
        vehicle = payload["vehicle"]
        identifier = vehicle["vehicle_id"]
        self.current_vehicle_id = identifier
        self.selector.setCurrentIndex(self.selector.findData(identifier))
        self.configuration.setText(
            f"Flight controller: {vehicle['flight_controller']}\n"
            f"Model: {vehicle['model']}\nMAVLink system: {vehicle['mavlink_system_id']}\n"
            f"ROS namespace: {vehicle['ros_namespace']}\n"
            f"World: {payload['world']['name']}\n"
            f"Mission: {payload.get('mission_name') or 'Uploaded vehicle mission / none'}\n\n"
            "Use the workspace selector to view one vehicle or target the collection. "
            "Each vehicle owns its mission, faults, limits and recordings."
        )

    def telemetry(self, snapshot: dict) -> None:
        self.state.setText(
            f"Connected: {snapshot.get('connected', False)}\n"
            f"Ready: {snapshot.get('ready', False)}\nArmed: {snapshot.get('armed', False)}\n"
            f"Mode: {snapshot.get('flight_mode', 'Unknown')}\n"
            f"Heartbeat age: {snapshot.get('heartbeat_age_seconds', 'Unknown')} s"
        )
