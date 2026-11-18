"""Fault input requires one explicit target and remains pending until acknowledged."""

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication

from control_panel.sensors.gnss_control import GnssInputControl


class SensorClient(QObject):
    resource_received = Signal(str, dict)
    gnss_operation_result = Signal(str, dict)

    def __init__(self):
        super().__init__()
        self.target_ids = ()
        self.calls = []

    def execute_gnss_targets(self, targets, enabled, request_id):
        self.calls.append(("sensors/gnss", {"enabled": enabled}, targets))
        self.request_id = request_id


def test_explicit_single_target_and_failed_acknowledgement():
    _app = QApplication.instance() or QApplication([])
    client = SensorClient()
    editor = GnssInputControl(client)
    try:
        client.target_ids = ("vehicle_1", "vehicle_2")
        assert not editor.apply.isEnabled()
        editor.resource(
            "vehicles",
            {
                "items": [
                    {"vehicle_id": "vehicle_1", "label": "Atlas"},
                    {"vehicle_id": "vehicle_2", "label": "Boreal"},
                ]
            },
        )
        editor.vehicle.setCurrentIndex(editor.vehicle.findData("vehicle_2"))
        client.target_ids = ("vehicle_1",)
        editor.enabled.setChecked(False)
        editor.apply.click()
        assert client.calls == [("sensors/gnss", {"enabled": False}, ("vehicle_2",))]
        assert not editor.apply.isEnabled()
        client.gnss_operation_result.emit(
            client.request_id, {"accepted": False, "error": "no vehicle acknowledgement"}
        )
        assert editor.apply.isEnabled()
        assert "failed" in editor.status.text()
    finally:
        editor.close()
