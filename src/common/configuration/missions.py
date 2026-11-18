"""Validate the supported simple-waypoint subset of QGroundControl plans."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MissionItem:
    command: int
    frame: int
    parameters: tuple[float, ...]


def parse_plan(plan: dict) -> tuple[MissionItem, ...]:
    """Reject unsupported commands instead of silently changing a saved mission."""
    if plan.get("fileType") != "Plan" or plan.get("version") != 1:
        raise ValueError("Expected a QGroundControl Plan file with version 1")
    mission = plan.get("mission")
    if not isinstance(mission, dict):
        raise ValueError("Plan must contain a mission object")
    items = mission.get("items", [])
    if not isinstance(items, list) or not items or len(items) > 500:
        raise ValueError("A mission requires between 1 and 500 simple items")
    result = []
    for item in items:
        if not isinstance(item, dict):
            raise ValueError("Each mission item must be an object")
        if item.get("type") != "SimpleItem":
            raise ValueError("Only SimpleItem missions are currently supported")
        command, frame = item.get("command"), item.get("frame")
        if type(command) is not int or type(frame) is not int:
            raise ValueError("Mission command and frame must be integers")
        if item.get("autoContinue", True) is not True:
            raise ValueError("Mission items that stop automatic progression are not supported")
        geographic = command in {16, 21, 22} and frame in {3, 6}
        return_to_launch = command == 20 and frame in {2, 3, 6}
        if not geographic and not return_to_launch:
            raise ValueError(
                "Only relative-altitude waypoint, takeoff, land and return-to-launch are supported"
            )
        parameters = item.get("params", [])
        if not isinstance(parameters, list) or len(parameters) != 7:
            raise ValueError("Each mission item requires seven parameters")
        for index, value in enumerate(parameters):
            if value is None and index == 3:
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("Mission parameters must be numbers, with optional null yaw")
        # QGC uses null for unspecified yaw. Preserve it as MAVLink's NaN sentinel.
        try:
            values = tuple(float(value) if value is not None else math.nan for value in parameters)
        except OverflowError as error:
            raise ValueError("Mission parameters exceed the finite numeric range") from error
        if parameters[3] is not None and not math.isfinite(values[3]):
            raise ValueError("Mission yaw must be finite or null for unspecified yaw")
        if not all(math.isfinite(values[index]) for index in (0, 1, 2, 4, 5, 6)):
            raise ValueError("Mission parameters must be finite, except unspecified yaw")
        if not -90 <= values[4] <= 90 or not -180 <= values[5] <= 180:
            raise ValueError("Mission coordinates are outside geographic bounds")
        if not 0 <= values[6] <= 500:
            raise ValueError("Relative altitude must be in 0..500 metres")
        result.append(MissionItem(command, frame, values))
    return tuple(result)
