"""Supported flight shortcuts and per-vehicle command admission requirements."""


def flight_action_status(snapshot: dict, navigation_active: bool = False) -> dict:
    """Describe acknowledged application operations, without issuing controller commands."""
    connected = snapshot.get("connected") is True
    armed = snapshot.get("armed") is True
    start_reason = (
        "Wait for the vehicle connection"
        if not connected
        else "Land and disarm before starting a mission"
        if armed
        else "Cancel the active navigation run first"
        if navigation_active
        else "Wait for position, preflight checks and recording readiness"
        if not snapshot.get("ready")
        else ""
    )
    land_reason = (
        "Wait for the vehicle connection"
        if not connected
        else "Vehicle is already disarmed"
        if not armed
        else ""
    )
    return {
        "start": {"supported": True, "enabled": not start_reason, "reason": start_reason},
        "land": {"supported": True, "enabled": not land_reason, "reason": land_reason},
        "takeoff": {
            "supported": False,
            "enabled": False,
            "reason": "Standalone takeoff is unavailable; load and start a takeoff mission",
        },
        "return": {
            "supported": False,
            "enabled": False,
            "reason": "Standalone return-home is unavailable; use the saved mission return",
        },
        "pause": {
            "supported": False,
            "enabled": False,
            "reason": "Mission pause is unavailable; world pause affects all vehicles",
        },
    }
