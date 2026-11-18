"""Saved mission identity, bounded frame conversion and unsupported intent rejection."""

import copy
import hashlib
import json
import threading
import time
from pathlib import Path
from unittest.mock import patch

import pytest

from common.configuration.mission_store import MissionStore
from common.navigation.mission import VisualMission
from common.navigation.occupancy import GridConfiguration, ObservedGrid
from common.navigation.session import NavigationSession
from testing.unit.test_navigation_freshness_wait import (
    ControlledSensing,
    WaitingVehicle,
    await_condition,
)
from testing.unit.test_navigation_session import EstimatedVehicle, MeasuredRanges


def plan() -> dict:
    return json.loads(Path("configs/missions/visual-comparison.plan").read_text())


def test_range_return_still_requires_visual_localization_before_arming(tmp_path: Path) -> None:
    vehicle = WaitingVehicle()
    session = NavigationSession(
        "vehicle_1",
        vehicle,
        MeasuredRanges(),
        threading.Lock(),
        tmp_path,
    )
    with pytest.raises(ValueError, match="visual navigation profile"):
        session.start(
            {"obstacle_mode": "range", "return_to_start": True, "goal_offset_enu_m": [2.5, 1.5]}
        )
    assert not vehicle.armed


def test_range_return_rejects_lost_visual_aiding_before_arming(tmp_path: Path) -> None:
    vehicle = WaitingVehicle()

    def missing_aiding(run_id: str) -> tuple[int, int]:
        raise RuntimeError("Visual aiding stale")

    session = NavigationSession(
        "vehicle_1",
        vehicle,
        MeasuredRanges(),
        threading.Lock(),
        tmp_path,
        localization_guard=missing_aiding,
    )
    with pytest.raises(RuntimeError, match="Visual aiding stale"):
        session.start(
            {"obstacle_mode": "range", "return_to_start": True, "goal_offset_enu_m": [2.5, 1.5]}
        )
    assert not vehicle.armed and session.thread is None


def test_running_range_mission_uses_declared_grid_extent(tmp_path: Path) -> None:
    vehicle = WaitingVehicle()
    configured_grids = []

    def grid_at_launch(origin, configuration):
        grid = ObservedGrid(origin, configuration)
        configured_grids.append(grid)
        return grid

    session = NavigationSession(
        "vehicle_1",
        vehicle,
        MeasuredRanges(),
        threading.Lock(),
        tmp_path,
        localization_guard=lambda run_id: (0, 0),
        grid_configuration=GridConfiguration(half_width_m=50),
    )
    with patch("common.navigation.session.ObservedGrid", side_effect=grid_at_launch):
        session.start(
            {"obstacle_mode": "range", "return_to_start": True, "goal_offset_enu_m": [2.5, 1.5]}
        )
        try:
            await_condition(lambda: bool(configured_grids))
            grid = configured_grids[0]
            assert grid.inside(grid.cell((0, 40)))
            assert not grid.inside(grid.cell((0, 51)))
        finally:
            session.cancel()
            session.thread.join(timeout=5)


def test_saved_mission_uses_same_waypoint_offsets_at_each_independent_home(tmp_path: Path) -> None:
    store = MissionStore(tmp_path)
    store.save("comparison", plan())
    loaded, digest = store.load_with_digest("comparison")
    assert digest == hashlib.sha256(store.path("comparison").read_bytes()).hexdigest()
    mission = VisualMission.from_plan("comparison", digest, loaded)
    assert mission.goal_offset_enu_m == pytest.approx((2.5, 1.5), abs=1e-6)
    assert mission.waypoint_hold_sim_seconds == 10
    assert mission.sha256 == digest


def test_saved_mission_rejects_a_missing_simulation_clock_before_arming(tmp_path: Path) -> None:
    vehicle = EstimatedVehicle()
    authority = threading.Lock()
    session = NavigationSession(
        "vehicle_1",
        vehicle,
        MeasuredRanges(),
        authority,
        tmp_path,
        stereo=object(),
        localization_guard=lambda run_id: (0, 0),
    )
    mission = VisualMission.from_plan("comparison", "a" * 64, plan())
    with pytest.raises(RuntimeError, match="simulation clock"):
        session.start(
            {
                "obstacle_mode": "camera",
                "return_to_start": True,
                "goal_offset_enu_m": list(mission.goal_offset_enu_m),
            },
            mission=mission,
        )
    assert not vehicle.armed and session.thread is None
    assert authority.acquire(blocking=False)
    authority.release()


def test_saved_waypoint_hold_refreshes_observations_and_still_lands_on_loss(tmp_path: Path) -> None:
    vehicle = WaitingVehicle()
    sensing = ControlledSensing()
    mission = VisualMission("comparison", "a" * 64, (2.5, 1.5), 10.0)
    session = NavigationSession(
        "vehicle_1",
        vehicle,
        MeasuredRanges(),
        threading.Lock(),
        tmp_path,
        stereo=object(),
        localization_guard=lambda run_id: (0, 0),
    )
    with patch("common.navigation.session.NavigationSensing", return_value=sensing):
        session.start(
            {"obstacle_mode": "camera", "return_to_start": True, "goal_offset_enu_m": [2.5, 1.5]},
            mission=mission,
        )
        try:
            await_condition(lambda: vehicle.armed)
            vehicle.position = (2.5, 1.5, 4.0)
            await_condition(lambda: session.snapshot().get("current_waypoint") == "outbound-1-hold")
            initial_observation = session.snapshot().get("control_sensor_simulation_seconds", 0)
            time.sleep(2.0)
            assert session.thread.is_alive()
            assert session.snapshot()["control_sensor_simulation_seconds"] > initial_observation
            assert session.snapshot()["freshness_wait_count"] == 0
            sensing.blocked = True
            session.thread.join(timeout=3)
            assert session.snapshot()["outcome"] == "failed"
            assert "observation wait" in session.snapshot()["error"]
            assert not vehicle.armed and vehicle.landings == 1
        finally:
            session.cancel()
            session.thread.join(timeout=5)


@pytest.mark.parametrize("change", ["extra-waypoint", "height", "yaw", "hold", "home", "radius"])
def test_unsupported_mission_intent_is_rejected_before_flight(change: str) -> None:
    mission = copy.deepcopy(plan())
    items = mission["mission"]["items"]
    if change == "extra-waypoint":
        items.insert(2, copy.deepcopy(items[1]))
    elif change == "height":
        items[1]["params"][6] = 8
    elif change == "yaw":
        items[1]["params"][3] = 45
    elif change == "hold":
        items[1]["params"][0] = 20
    elif change == "home":
        mission["mission"]["plannedHomePosition"] = [90, 0, 0]
    else:
        items[1]["params"][1] = 2
    with pytest.raises(ValueError):
        VisualMission.from_plan("comparison", "a" * 64, mission)


def test_downloaded_home_metadata_accepts_centimetre_noise_but_rejects_other_launch():
    value = plan()
    value["mission"]["plannedHomePosition"][1] += 0.0000002
    VisualMission.from_plan("comparison", "a" * 64, value)
    value["mission"]["plannedHomePosition"][1] += 0.00001
    with pytest.raises(ValueError, match="within 0.25 m"):
        VisualMission.from_plan("comparison", "a" * 64, value)
