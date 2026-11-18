"""Native sensor bits become measured evidence only at the MAVLink boundary."""

from common.interfaces.health import HealthState
from common.interfaces.replay import ReplayObservation, ReplayRecorderHealth
from common.lifecycle.sensor_health import observed_health
from integrations.mavlink.health import reported_sensor_health


def test_native_presence_health_receipt_and_peer_identity_are_distinct() -> None:
    snapshot = {
        "connected": True,
        "heartbeat_received_monotonic": 10,
        "system_status_received_monotonic": 10,
        "messages": {
            "SYS_STATUS": {
                "onboard_control_sensors_present": 15,
                "onboard_control_sensors_health": 11,
            }
        },
    }
    evidence = reported_sensor_health(snapshot)
    observed = ReplayObservation(
        "atlas", "Atlas", "new", 1, None, None, ReplayRecorderHealth(True), systems=evidence
    )
    rows = {
        row.system: row for row in observed_health(observed, True, 10.1, visual_supported=False)
    }
    assert rows["IMU"].state == HealthState.HEALTHY
    assert rows["Magnetometer"].state == HealthState.DEGRADED
    assert rows["Barometer"].state == HealthState.HEALTHY
    assert rows["Link"].state == HealthState.HEALTHY
    assert all(
        row.vehicle_id == "atlas" and row.controller_generation == "new" for row in rows.values()
    )
    stale = observed_health(observed, True, 16, visual_supported=False)
    assert all(
        row.state == HealthState.UNKNOWN
        for row in stale
        if row.system in {"IMU", "Magnetometer", "Barometer", "Link"}
    )


def test_absent_sensor_is_different_from_unreported_timestamp() -> None:
    snapshot = {
        "messages": {
            "SYS_STATUS": {
                "onboard_control_sensors_present": 3,
                "onboard_control_sensors_health": 3,
            }
        },
        "system_status_received_monotonic": 10,
    }
    observed = ReplayObservation(
        "boreal",
        "Boreal",
        "run",
        1,
        None,
        None,
        ReplayRecorderHealth(True),
        systems=reported_sensor_health(snapshot),
    )
    rows = {
        row.system: row for row in observed_health(observed, True, 10.1, visual_supported=False)
    }
    assert rows["Magnetometer"].state == HealthState.UNAVAILABLE
    snapshot.pop("system_status_received_monotonic")
    observed = ReplayObservation(
        "boreal",
        "Boreal",
        "run",
        1,
        None,
        None,
        ReplayRecorderHealth(True),
        systems=reported_sensor_health(snapshot),
    )
    rows = {
        row.system: row for row in observed_health(observed, True, 10.1, visual_supported=False)
    }
    assert rows["Magnetometer"].state == HealthState.UNKNOWN
