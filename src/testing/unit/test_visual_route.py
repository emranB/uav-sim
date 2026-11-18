"""Ordered visual route intent, home binding and unchanged source-loss landing."""

import hashlib
import json
import threading
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import pytest

from common.configuration.visual_missions import VisualMissionLimits
from common.navigation.mission import VisualMission
from common.navigation.session import NavigationSession
from gazebo.navigation_scene import add_inspection_buildings
from testing.unit.test_navigation_freshness_wait import (
    ControlledSensing,
    WaitingVehicle,
    await_condition,
)
from testing.unit.test_navigation_session import MeasuredRanges


def extended_route() -> VisualMission:
    content = Path("configs/missions/extended-mixed-gps-out-and-back.plan").read_bytes()
    return VisualMission.from_plan(
        "extended-mixed-gps-out-and-back",
        hashlib.sha256(content).hexdigest(),
        json.loads(content),
        VisualMissionLimits(16, 60, 600),
    )


def test_route_preserves_order_holds_and_declared_distance() -> None:
    route = extended_route()
    for point, expected in zip(
        route.outbound_waypoints, [(-7.5, 0), (-7.5, 8), (-3.8, 8), (-3.8, -4)]
    ):
        assert point.offset_enu_m == pytest.approx(expected, abs=1e-6)
    assert [p.hold_sim_seconds for p in route.outbound_waypoints] == [1, 1, 1, 1]
    assert 36 < route.planned_distance_m < 37
    assert route.maximum_navigation_wall_seconds == 600


def test_radius_and_total_route_budgets_reject_before_flight() -> None:
    plan = json.loads(Path("configs/missions/extended-mixed-gps-out-and-back.plan").read_text())
    with pytest.raises(ValueError, match="metres from home"):
        VisualMission.from_plan("long", "a" * 64, plan)
    with pytest.raises(ValueError, match="distance budget"):
        VisualMission.from_plan("long", "a" * 64, plan, VisualMissionLimits(16, 20, 600))


@pytest.mark.parametrize(
    "values", [(float("nan"), 60, 600), (101, 60, 600), (16, 601, 600), (16, 60, 2401)]
)
def test_route_policy_rejects_unbounded_configuration(values: tuple) -> None:
    with pytest.raises(ValueError):
        VisualMissionLimits(*values)


@pytest.mark.parametrize("home_x", [0.0, 6.0])
def test_every_supplied_corner_is_required_before_return_and_landing(
    tmp_path: Path, home_x: float
) -> None:
    class ArrivingVehicle(WaitingVehicle):
        def begin_navigation(self) -> None:
            self.armed, self.mode = True, "OFFBOARD"
            self.position = (home_x, 0.0, 4.0)

        def send_navigation_velocity(self, velocity_enu_m_s: tuple) -> None:
            self.velocities.append(velocity_enu_m_s)

    route = extended_route()
    vehicle = ArrivingVehicle()
    vehicle.position = (home_x, 0.0, 0.0)
    sensing = ControlledSensing()
    sensing.grid.free = {(x, y): 0.0 for x in range(-44, 45) for y in range(-44, 45)}
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
            {
                "obstacle_mode": "camera",
                "return_to_start": True,
                "goal_offset_enu_m": list(route.goal_offset_enu_m),
            },
            mission=route,
        )
        try:
            await_condition(lambda: vehicle.armed)
            for index, waypoint in enumerate(route.outbound_waypoints, 1):
                vehicle.position = (home_x + waypoint.offset_enu_m[0], waypoint.offset_enu_m[1], 4)
                await_condition(lambda: len(session.snapshot()["waypoint_events"]) == index)
                if index < len(route.outbound_waypoints):
                    await_condition(
                        lambda: session.snapshot()["current_waypoint"] == f"outbound-{index + 1}"
                    )
                    assert session.snapshot()["leg"] == "outbound"
            await_condition(lambda: session.snapshot()["leg"] == "return")
            vehicle.position = (home_x, 0.0, 4.0)
            session.thread.join(timeout=5)
            assert not session.thread.is_alive()
            assert session.snapshot()["state"] == "reached"
            assert [x["waypoint_id"] for x in session.snapshot()["waypoint_events"]] == [
                "outbound-1",
                "outbound-2",
                "outbound-3",
                "outbound-4",
                "home",
            ]
            assert not vehicle.armed and vehicle.landings == 1
        finally:
            session.cancel()
            session.thread.join(timeout=5)


def test_retained_building_asset_has_matching_scaled_physics_and_visuals() -> None:
    scene = json.loads(
        Path("configs/navigation/stereo/extended-mixed-gps-buildings.json").read_text()
    )
    world = ET.Element("world")
    add_inspection_buildings(world, scene["inspection_buildings"])
    assert len(world.findall("model")) == 3
    for model in world.findall("model"):
        assert model.findtext("link/collision/geometry/box/size") == "1.5 1.5 6.0"
        assert model.findtext("link/visual/geometry/box/size") == "1.5 1.5 6.0"
