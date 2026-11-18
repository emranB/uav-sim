"""Registry keeps tab discovery out of the main window."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from PySide6.QtWidgets import QWidget

    from desktop.client import SimulationClient


@dataclass(frozen=True)
class PanelSpec:
    key: str
    title: str
    factory: Callable[[], QWidget]


class PanelRegistry:
    def __init__(self) -> None:
        self._specs: dict[str, PanelSpec] = {}

    def register(self, spec: PanelSpec) -> None:
        if spec.key in self._specs:
            raise ValueError(f"Duplicate panel: {spec.key}")
        self._specs[spec.key] = spec

    def all(self) -> tuple[PanelSpec, ...]:
        return tuple(self._specs.values())


def default_registry(client: SimulationClient) -> PanelRegistry:
    from PySide6.QtWidgets import QScrollArea

    from control_panel.interference.panel import InterferencePanel
    from control_panel.logs.panel import LogsPanel
    from control_panel.missions.panel import MissionsPanel
    from control_panel.sensors.panel import SensorsPanel
    from control_panel.settings.panel import SettingsPanel
    from control_panel.telemetry.panel import TelemetryPanel
    from control_panel.vehicles.panel import VehiclesPanel

    def scrollable(panel_type):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel_type(client))
        return scroll

    registry = PanelRegistry()
    for key, title, panel_type in (
        ("sensors", "Sensors", SensorsPanel),
        ("interference", "Interference", InterferencePanel),
        ("telemetry", "Telemetry", TelemetryPanel),
        ("px4_logs", "PX4 Logs", LogsPanel),
        ("vehicles", "Vehicles", VehiclesPanel),
        ("missions", "Missions", MissionsPanel),
        ("settings", "Settings", SettingsPanel),
    ):
        registry.register(PanelSpec(key, title, lambda panel=panel_type: scrollable(panel)))
    if os.environ.get("UAV_NAVIGATION", "false") == "true":
        from control_panel.navigation.fleet import FleetComparisonPanel
        from control_panel.navigation.panel import NavigationPanel

        registry.register(
            PanelSpec("navigation", "Navigation", lambda: scrollable(NavigationPanel))
        )
        registry.register(
            PanelSpec("visual_fleet", "Visual flights", lambda: scrollable(FleetComparisonPanel))
        )
    return registry
