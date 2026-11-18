"""Explicit group admission and independent flight shortcut outcomes."""

from unittest.mock import Mock

import pytest

from common.simulation_manager.fleet import FleetOperations
from common.simulation_manager.flight_actions import flight_action_status


def services() -> dict:
    result = {identity: Mock() for identity in ("vehicle_1", "vehicle_2")}
    for service in result.values():
        service.flight_actions.return_value = flight_action_status(
            {"connected": True, "armed": False, "ready": True}
        )
        service.execute.return_value = {"accepted": True}
    return result


def test_empty_or_implicit_shortcut_selection_is_rejected() -> None:
    fleet = FleetOperations(services())
    for body in ({}, {"target_ids": []}):
        with pytest.raises(ValueError):
            fleet.execute("flight/start", body)


def test_unready_selected_vehicle_blocks_group_before_any_dispatch() -> None:
    vehicles = services()
    vehicles["vehicle_2"].flight_actions.return_value = flight_action_status(
        {"connected": False, "ready": False, "armed": False}
    )
    result = FleetOperations(vehicles).execute(
        "flight/start", {"target_ids": ["vehicle_1", "vehicle_2"]}
    )
    assert not result["accepted"] and not result["preflight_passed"]
    assert "connection" in result["results"]["vehicle_2"]["error"]
    for service in vehicles.values():
        service.execute.assert_not_called()


def test_independent_command_failure_retains_successful_peer_acknowledgement() -> None:
    vehicles = services()
    vehicles["vehicle_1"].execute.side_effect = TimeoutError("Atlas mission ACK timed out")
    result = FleetOperations(vehicles).execute(
        "flight/start", {"target_ids": ["vehicle_1", "vehicle_2"]}
    )
    assert result["preflight_passed"] and not result["accepted"]
    assert not result["results"]["vehicle_1"]["accepted"]
    assert result["results"]["vehicle_2"]["accepted"]
    vehicles["vehicle_2"].execute.assert_called_once_with("flight/start", {})


@pytest.mark.parametrize("target", ["vehicle_1", "vehicle_2"])
def test_single_selection_does_not_command_peer(target: str) -> None:
    vehicles = services()
    result = FleetOperations(vehicles).execute("flight/start", {"target_ids": [target]})
    assert result["accepted"]
    for identity, service in vehicles.items():
        if identity == target:
            service.execute.assert_called_once_with("flight/start", {})
        else:
            service.execute.assert_not_called()
