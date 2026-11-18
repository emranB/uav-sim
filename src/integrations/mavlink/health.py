"""Translate native SYS_STATUS sensor masks into transport-neutral reported evidence."""

from common.interfaces.health import SystemHealthEvidence


def reported_sensor_health(snapshot: dict) -> tuple[SystemHealthEvidence, ...]:
    """Decode standard MAV_SYS_STATUS_SENSOR bits once at the protocol boundary."""
    status = (snapshot.get("messages") or {}).get("SYS_STATUS") or {}
    present = status.get("onboard_control_sensors_present")
    healthy = status.get("onboard_control_sensors_health")
    received = snapshot.get("system_status_received_monotonic")
    rows = []
    # MAVLink common.xml: gyro=1, accelerometer=2, magnetometer=4, absolute pressure=8.
    for name, mask in (("IMU", 3), ("Magnetometer", 4), ("Barometer", 8)):
        available = type(present) is int and type(healthy) is int
        rows.append(
            SystemHealthEvidence(
                name,
                True,
                present & mask == mask if available else None,
                healthy & mask == mask if available else None,
                received,
                5.0,
                "Native SYS_STATUS presence and health flags"
                if available
                else "Native SYS_STATUS sensor flags have not been received",
            )
        )
    rows.append(
        SystemHealthEvidence(
            "Link",
            True,
            None,
            snapshot.get("connected"),
            snapshot.get("heartbeat_received_monotonic"),
            snapshot.get("heartbeat_timeout_seconds", 5.0),
            "MAVLink heartbeat freshness; API link only",
        )
    )
    return tuple(rows)
