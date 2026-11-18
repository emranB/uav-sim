"""Shared measured-health presentation; vehicle colours never stand for health states."""

from PySide6.QtWidgets import QGridLayout, QLabel, QWidget


def present_health_badge(
    label: QLabel,
    vehicle: dict,
    health: dict,
    display_age_seconds: float,
    *,
    compact: bool = True,
) -> None:
    """Keep source evidence age, display age and controller identity distinct."""
    state, reason = health["state"], health["reason"]
    generation = vehicle.get("controller_generation", vehicle.get("run_id"))
    if (
        not generation
        or display_age_seconds > 1.5
        or health.get("vehicle_id") != vehicle["vehicle_id"]
        or health.get("controller_generation") != generation
    ):
        state, reason = "stale_or_unknown", "Refresh current-generation vehicle evidence"
    symbol, colour = {
        "healthy": ("●", "#69d9bc"),
        "degraded": ("▲", "#e3b773"),
        "unavailable": ("!", "#ff7b85"),
        "stale_or_unknown": ("?", "#91a0af"),
        "unsupported": ("—", "#91a0af"),
    }.get(state, ("?", "#91a0af"))
    short = {
        "GPS fix": "GPS",
        "GPS fusion": "GNSS",
        "Vision fusion": "Vision",
        "Diagnostic log": "Log",
        "Magnetometer": "Mag",
        "Barometer": "Baro",
    }.get(health["system"], health["system"])
    text = (
        f"{symbol} {short}"
        if compact
        else (f"{symbol} {vehicle['label']} · {health['system']}: {state.replace('_', ' ')}")
    )
    label.setText(text)
    label.setAccessibleName(
        f"{vehicle['label']} ({vehicle['vehicle_id']}) {health['system']} {state.replace('_', ' ')}"
    )
    label.setStyleSheet(f"color: {colour};")
    label.setToolTip(
        f"{vehicle['label']} ({vehicle['vehicle_id']}) · {health['system']}: {state.replace('_', ' ')}\n"
        f"Controller: {health.get('controller_generation')}\n{reason}\n"
        f"Evidence age at API snapshot: {health.get('evidence_age_seconds')} s\n"
        f"Requested state: {health.get('requested_state') or 'unknown'}\n"
        f"Display snapshot age: {display_age_seconds:.2f} s"
    )


class VehicleHealthBadges(QWidget):
    """Compact identity-scoped status for the listed fleet, independent of selection."""

    def __init__(self, columns: int = 10, systems: tuple[str, ...] | None = None) -> None:
        super().__init__()
        if not 2 <= columns <= 10:
            raise ValueError("Health badge columns must be in 2..10")
        self.columns = columns
        self.systems = systems
        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setHorizontalSpacing(7)
        self._grid.setVerticalSpacing(3)
        self._labels: dict[tuple[str, str], QLabel] = {}

    def present(self, vehicles: list[dict], display_age_seconds: float) -> None:
        """Present bounded lists; missing and removed evidence never inherits peer health."""
        active = set()
        row_index = 0
        for vehicle in vehicles[:8]:
            identity = vehicle["vehicle_id"]
            health = [
                status
                for status in vehicle.get("health", [])[:10]
                if self.systems is None or status["system"] in self.systems
            ]
            span = max(1, (len(health) + self.columns - 2) // (self.columns - 1))
            name_key = identity, "name"
            name = self._label(name_key)
            name.setText(vehicle["label"])
            name.setToolTip(
                f"{identity} · {vehicle.get('run_id', vehicle.get('controller_generation'))}"
            )
            self._grid.addWidget(name, row_index, 0, span, 1)
            active.add(name_key)
            for index, status in enumerate(health):
                key = identity, status["system"]
                label = self._label(key)
                present_health_badge(label, vehicle, status, display_age_seconds)
                self._grid.addWidget(
                    label, row_index + index // (self.columns - 1), 1 + index % (self.columns - 1)
                )
                active.add(key)
            row_index += span
        for key in set(self._labels) - active:
            self._labels.pop(key).deleteLater()

    def _label(self, key: tuple[str, str]) -> QLabel:
        if key not in self._labels:
            label = QLabel()
            label.setObjectName("health_" + key[0] + "_" + key[1].replace(" ", "_"))
            self._labels[key] = label
        return self._labels[key]
