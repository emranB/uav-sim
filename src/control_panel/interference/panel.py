"""Parameters for real simulated GNSS degradation and API link impairment."""

from typing import TYPE_CHECKING

from PySide6.QtWidgets import QFormLayout, QGroupBox, QLabel, QSpinBox, QVBoxLayout

from control_panel.form import PanelView

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class InterferencePanel(PanelView):
    def __init__(self, client: "SimulationClient") -> None:
        super().__init__(client)
        explanation = QLabel(
            "Choose command targets below the mission map. Readings describe the inspected "
            "vehicle; fleet status lists each vehicle separately."
        )
        explanation.setWordWrap(True)
        self.form.addWidget(explanation)
        gnss_group = QGroupBox("GNSS reception")
        gnss_layout = QVBoxLayout(gnss_group)
        hint = QLabel("10 satellites is normal. 0–3 satellites removes the simulated fix.")
        hint.setWordWrap(True)
        gnss_layout.addWidget(hint)
        gnss_fields = QFormLayout()
        self.satellites = QSpinBox()
        self.satellites.setRange(0, 20)
        self.satellites.setValue(10)
        gnss_fields.addRow("Visible satellites", self.satellites)
        gnss_layout.addLayout(gnss_fields)
        self.form.addWidget(gnss_group)
        self.button(
            "Apply GNSS fault",
            lambda: self.send("interference/gnss", {"satellites": self.satellites.value()}),
            gnss_layout,
        )
        link_group = QGroupBox("Telemetry communication")
        link_layout = QVBoxLayout(link_group)
        hint = QLabel(
            "Delay and packet loss can run together. QGroundControl and ROS use separate links."
        )
        hint.setWordWrap(True)
        link_layout.addWidget(hint)
        fields = QFormLayout()
        self.latency = QSpinBox()
        self.latency.setRange(0, 2000)
        self.latency.setSuffix(" ms")
        self.loss = QSpinBox()
        self.loss.setRange(0, 100)
        self.loss.setSuffix(" %")
        self.seed = QSpinBox()
        self.seed.setRange(0, 2**31 - 1)
        for title, widget in (
            ("One-way latency", self.latency),
            ("Packet loss", self.loss),
            ("Random seed", self.seed),
        ):
            fields.addRow(title, widget)
        link_layout.addLayout(fields)
        self.form.addWidget(link_group)
        self.button(
            "Apply communication faults",
            lambda: self.send(
                "interference/link",
                {
                    "latency_ms": self.latency.value(),
                    "loss_percent": self.loss.value(),
                    "seed": self.seed.value(),
                },
            ),
            link_layout,
        )
        self.button("Restore normal signals for targets", lambda: self.send("interference/clear"))
        self.active = QLabel()
        self.active.setWordWrap(True)
        self.form.addWidget(self.active)
        self.fleet_state = QLabel()
        self.fleet_state.setWordWrap(True)
        self.form.addWidget(self.fleet_state)
        self.scope = QLabel(
            "Communication faults affect both directions of the application MAVLink link. "
            "QGroundControl and ROS 2 use separate links. Fewer than four simulated satellites "
            "removes the GNSS fix. Other sensor failure modes are not supported by this adapter."
        )
        self.scope.setWordWrap(True)
        self.form.addWidget(self.scope)
        self.finish()
        client.telemetry_received.connect(self.telemetry)
        client.resource_received.connect(self.resource)

    def resource(self, name: str, payload: dict) -> None:
        if name == "vehicles":
            self.fleet_state.setText(
                "FLEET SIGNAL STATUS\n"
                + "\n".join(
                    f"{vehicle['label']}: GNSS "
                    f"{'fix' if vehicle.get('messages', {}).get('GPS_RAW_INT', {}).get('fix_type', 0) >= 3 else 'no fix'}"
                    f" · {vehicle.get('communication', {}).get('loss_percent', 0)}% loss"
                    f" · {vehicle.get('communication', {}).get('latency_ms', 0)} ms"
                    for vehicle in payload["items"]
                )
            )
        if name == "workspace":
            self.scope.setToolTip(str(payload["environment"]))
            environment = payload["environment"]
            satellites = environment["satellites"] if environment["gnss_enabled"] else 0
            self.active.setText(
                f"VIEWED VEHICLE: {payload['vehicle']['label']}\n"
                f"GNSS: {'Normal' if satellites >= 4 else 'NO FIX'} · {satellites} satellites\n"
                f"Delay: {environment['link']['latency_ms']} ms\n"
                f"Packet loss: {environment['link']['loss_percent']}%"
            )
            self.active.setStyleSheet(
                "color: #f3cf8d; font-size: 15px; padding: 10px; background: #243549; border-radius: 6px;"
            )

    def reset_panel(self) -> None:
        super().reset_panel()
        for widget, value in (
            (self.satellites, 10),
            (self.latency, 0),
            (self.loss, 0),
            (self.seed, 0),
        ):
            widget.setValue(value)

    def telemetry(self, snapshot: dict) -> None:
        link = snapshot.get("communication", {})
        self.active.setToolTip(
            f"Packets sent / dropped: {link.get('sent_packets', 0)} / {link.get('dropped_packets', 0)}"
        )
