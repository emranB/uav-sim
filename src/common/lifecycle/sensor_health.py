"""Classify observed status without interpreting disabled fusion as broken hardware."""

from common.interfaces.health import HealthState, SensorHealth
from common.interfaces.replay import ReplayObservation


def observed_health(
    observation: ReplayObservation,
    connected: bool,
    observed_monotonic: float,
    *,
    visual_supported: bool,
) -> tuple[SensorHealth, ...]:
    """Report current native evidence; old generations and stale links remain unknown."""
    aiding = observation.aiding
    connected = connected and bool(observation.controller_generation)
    age = (
        observed_monotonic - aiding.received_monotonic
        if aiding and aiding.received_monotonic is not None
        else None
    )
    fresh = (
        connected
        and age is not None
        and 0 <= age <= 0.5
        and aiding.source_samples_fresh is not False
    )
    requested = (
        None
        if not aiding
        else (
            "enabled"
            if aiding.gps_requested_enabled is True
            else "disabled"
            if aiding.gps_requested_enabled is False
            else None
        )
    )

    def status(
        system: str, state: HealthState, reason: str, evidence_age=age, requested_state=None
    ) -> SensorHealth:
        return SensorHealth(
            observation.vehicle_id,
            observation.controller_generation,
            system,
            state,
            reason,
            evidence_age,
            requested_state,
        )

    gps_state, gps_reason = HealthState.UNKNOWN, "Fresh GPS measurement unavailable"
    fusion_state, fusion_reason = HealthState.UNKNOWN, "Fresh native fusion evidence unavailable"
    if fresh:
        if aiding.gps_fix_type is not None:
            if aiding.gps_fix_type < 2:
                gps_state, gps_reason = (
                    HealthState.UNAVAILABLE,
                    "Native GPS reports no position fix",
                )
            elif aiding.gps_fix_type == 2:
                gps_state, gps_reason = HealthState.DEGRADED, "Native GPS reports only a 2D fix"
            else:
                gps_state, gps_reason = HealthState.HEALTHY, "Native GPS reports a 3D fix"
        if aiding.gnss_xy_active is not None:
            if requested == "disabled":
                fusion_state = (
                    HealthState.DEGRADED if aiding.gnss_xy_active else HealthState.HEALTHY
                )
                fusion_reason = (
                    "Disable requested; native GNSS fusion is still active"
                    if aiding.gnss_xy_active
                    else "GNSS fusion intentionally disabled; hardware state is separate"
                )
            elif aiding.gnss_xy_active:
                fusion_state, fusion_reason = (
                    HealthState.HEALTHY,
                    "Native horizontal GNSS fusion active",
                )
            else:
                fusion_state, fusion_reason = (
                    HealthState.DEGRADED,
                    "Native horizontal GNSS fusion inactive",
                )
    vision_state = HealthState.UNKNOWN
    vision_reason = "Fresh estimated pose unavailable"
    pose_age = (
        observed_monotonic - aiding.visual_pose_received_monotonic
        if aiding and aiding.visual_pose_received_monotonic is not None
        else None
    )
    if not visual_supported:
        vision_state, vision_reason = (
            HealthState.UNSUPPORTED,
            "Visual localization is not configured",
        )
    elif connected and pose_age is not None and 0 <= pose_age <= 0.25 and fresh:
        if aiding.visual_pose_healthy is False:
            vision_state, vision_reason = (
                HealthState.UNAVAILABLE,
                "Visual source reports unhealthy tracking",
            )
        elif aiding.vision_xy_active is True and aiding.visual_pose_healthy is True:
            vision_state, vision_reason = (
                HealthState.HEALTHY,
                "Fresh pose and native external-vision fusion active",
            )
        elif aiding.vision_xy_active is False:
            vision_state, vision_reason = (
                HealthState.DEGRADED,
                "Native external-vision fusion inactive",
            )
    recorder = observation.recorder
    recording_state = (
        HealthState.HEALTHY
        if recorder.available
        else HealthState.UNAVAILABLE
        if recorder.error
        else HealthState.UNKNOWN
    )
    additional = []
    for system in observation.systems:
        system_age = (
            observed_monotonic - system.received_monotonic
            if system.received_monotonic is not None
            else None
        )
        state = HealthState.UNKNOWN
        reason = system.reason
        if not system.supported:
            state = HealthState.UNSUPPORTED
        elif connected and system_age is not None and 0 <= system_age <= system.maximum_age_seconds:
            if system.present is False:
                state = HealthState.UNAVAILABLE
            elif system.healthy is True:
                state = HealthState.HEALTHY
            elif system.healthy is False:
                state = HealthState.DEGRADED
        else:
            reason = "Fresh evidence unavailable. " + reason
        additional.append(status(system.system, state, reason, system_age))
    return (
        status("GPS fix", gps_state, gps_reason, requested_state=requested),
        status("GPS fusion", fusion_state, fusion_reason, requested_state=requested),
        status("Vision fusion", vision_state, vision_reason, pose_age),
        status(
            "Diagnostic log",
            recording_state,
            recorder.error
            or (
                f"Writer available; {recorder.bytes_written} bytes"
                if recorder.available
                else "Diagnostic writer unavailable"
            ),
            None,
        ),
    ) + tuple(additional)
