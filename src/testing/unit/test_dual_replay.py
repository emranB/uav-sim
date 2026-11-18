"""Sensor indicators follow actual freshness and fusion, not requested configuration."""

from integrations.blender.dual_replay import aiding_labels


def test_gps_disabled_does_not_claim_vision_active() -> None:
    text, denied = aiding_labels(
        {
            "aiding": {
                "received_monotonic": 10,
                "gps_requested_enabled": False,
                "gps_fix_type": 0,
                "gnss_xy_active": False,
                "vision_xy_active": False,
            }
        },
        10.1,
    )
    assert denied is True and "GPS: NO FIX" in text
    assert "Vision: INACTIVE" in text and "IMU: UNKNOWN" in text


def test_stale_evidence_is_unknown_even_if_all_flags_were_active() -> None:
    text, denied = aiding_labels(
        {
            "aiding": {
                "received_monotonic": 10,
                "gps_requested_enabled": True,
                "gps_fix_type": 3,
                "gnss_xy_active": True,
                "vision_xy_active": True,
            }
        },
        11,
    )
    assert denied is None and "ACTIVE" not in text
    assert text.count("UNKNOWN") == 4


def test_requested_disable_does_not_relabel_a_still_observed_gps_fix() -> None:
    text, denied = aiding_labels(
        {
            "aiding": {
                "received_monotonic": 10,
                "gps_requested_enabled": False,
                "gps_fix_type": 3,
                "gnss_xy_active": True,
                "vision_xy_active": True,
            },
            "systems": [
                {
                    "system": "IMU",
                    "healthy": True,
                    "received_monotonic": 10,
                    "maximum_age_seconds": 1,
                }
            ],
        },
        10.1,
    )
    assert denied is True and "GPS: ACTIVE" in text
    assert "Fusion: ACTIVE" in text and "IMU: HEALTHY" in text
