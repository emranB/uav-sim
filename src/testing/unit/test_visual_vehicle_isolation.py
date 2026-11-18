"""Exercise identity, generation, freshness and authority isolation without SITL."""

import socket
import threading
import time
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock

import pytest

from common.configuration.fleet import load_fleet
from common.configuration.visual_navigation import VisualEndpoints, validate_visual_endpoints
from common.simulation_manager.fleet import FleetOperations
from integrations.px4.visual_aiding import VisualAidingGuard, send_status


def test_visual_endpoint_allocations_do_not_overlap_vehicle_links() -> None:
    fleet = load_fleet(Path("configs/vehicles/fleet.json"))
    allocated = set()
    application_ports = {port for v in fleet for port in (v.vehicle_port, v.application_port)}
    for instance in range(9):
        vehicle = replace(fleet[0], instance=instance, system_id=instance + 1)
        endpoints = VisualEndpoints.for_vehicle(vehicle)
        ports = set(vars(endpoints).values())
        assert len(ports) == 5
        assert not ports & (allocated | application_ports)
        allocated.update(ports)
    validate_visual_endpoints(fleet)
    with pytest.raises(ValueError, match="conflicts"):
        validate_visual_endpoints((fleet[0], replace(fleet[1], application_port=14612)))


def test_visual_guards_reject_cross_vehicle_and_old_controller_samples() -> None:
    generations = {"vehicle_1": "atlas-1", "vehicle_2": "boreal-1"}
    guards = {
        vehicle_id: VisualAidingGuard(
            port=0,
            vehicle_id=vehicle_id,
            controller_generation=lambda identity=vehicle_id: generations[identity],
        )
        for vehicle_id in generations
    }
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:

            def publish(target: str, identity: str, generation: str, age: float = 0) -> None:
                record = {
                    "vehicle_id": identity,
                    "controller_run_id": generation,
                    "monotonic": time.monotonic() - age,
                    "vision_xy_active": True,
                    "source_samples_fresh": True,
                    "xy_valid": True,
                    "z_valid": True,
                    "capture_stamp_valid": True,
                    "xy_reset_counter": 0,
                    "healthy": True,
                    "map_generation": 2,
                    "timestamp_ns": 100,
                    "horizontal_ned_m": [1, 2],
                }
                for kind in ("pose", "aiding"):
                    send_status(sender, guards[target].port, {**record, "kind": kind})

            for identity, generation in generations.items():
                publish(identity, identity, generation)
                assert guards[identity](generation) == (2, 0)
            publish("vehicle_1", "vehicle_2", "atlas-1")
            assert guards["vehicle_1"]("atlas-1") == (2, 0)
            generations["vehicle_1"] = "atlas-2"
            publish("vehicle_1", "vehicle_1", "atlas-1")
            assert guards["vehicle_1"].snapshot() == {}
            with pytest.raises(RuntimeError, match="unavailable"):
                guards["vehicle_1"]("atlas-2")
            assert guards["vehicle_2"]("boreal-1") == (2, 0)
            publish("vehicle_1", "vehicle_1", "atlas-2", age=2)
            with pytest.raises(RuntimeError, match="stale"):
                guards["vehicle_1"]("atlas-2")
            publish("vehicle_1", "vehicle_1", "atlas-2")
            assert guards["vehicle_1"]("atlas-2") == (2, 0)
    finally:
        for guard in guards.values():
            guard.close()


def test_status_stays_current_without_api_polling() -> None:
    guard = VisualAidingGuard(port=0, vehicle_id="vehicle_2")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for sequence in range(100):
                send_status(
                    sender,
                    guard.port,
                    {
                        "vehicle_id": "vehicle_2",
                        "kind": "aiding",
                        "controller_run_id": "boreal",
                        "monotonic": time.monotonic(),
                        "sequence": sequence,
                    },
                )
                time.sleep(0.005)
            snapshot = guard.snapshot()
            assert snapshot["aiding"]["sequence"] == 99
            assert time.monotonic() - snapshot["aiding"]["monotonic"] < 0.25
    finally:
        guard.close()


@pytest.mark.parametrize("operation", ["pause", "resume"])
def test_shared_clock_controls_reject_an_active_peer(operation: str) -> None:
    atlas, boreal = Mock(), Mock()
    atlas.navigation_active = True
    boreal.navigation_active = False
    fleet = FleetOperations({"vehicle_1": atlas, "vehicle_2": boreal})
    with pytest.raises(RuntimeError, match="vehicle_1"):
        fleet.execute(operation, {"vehicle_id": "vehicle_2"})
    atlas.execute.assert_not_called()
    boreal.execute.assert_not_called()
    fleet.execute("navigation/cancel", {"vehicle_id": "vehicle_2"})
    boreal.execute.assert_called_once_with("navigation/cancel", {})


def test_simultaneous_navigation_admissions_keep_independent_vehicle_targets() -> None:
    entered, release = threading.Event(), threading.Event()
    atlas, boreal = Mock(), Mock()

    def admit_atlas(operation: str, body: dict) -> dict:
        entered.set()
        assert release.wait(1)
        return {"accepted": True}

    atlas.execute.side_effect = admit_atlas
    boreal.execute.return_value = {"accepted": True}
    fleet = FleetOperations({"vehicle_1": atlas, "vehicle_2": boreal})
    results = {}

    def start(identity: str) -> None:
        results[identity] = fleet.execute("navigation/start", {"vehicle_id": identity})

    first = threading.Thread(target=start, args=("vehicle_1",))
    second = threading.Thread(target=start, args=("vehicle_2",))
    first.start()
    try:
        assert entered.wait(1)
        second.start()
        release.set()
        first.join(timeout=2)
        second.join(timeout=2)
        assert results == {"vehicle_1": {"accepted": True}, "vehicle_2": {"accepted": True}}
        boreal.execute.assert_called_once_with("navigation/start", {})
    finally:
        release.set()
        first.join(timeout=2)
        if second.ident is not None:
            second.join(timeout=2)
