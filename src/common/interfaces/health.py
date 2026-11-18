"""Measured sensor/system health, distinct from user-requested configuration."""

from dataclasses import dataclass
from enum import Enum


class HealthState(str, Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNAVAILABLE = "unavailable"
    UNKNOWN = "stale_or_unknown"
    UNSUPPORTED = "unsupported"


@dataclass(frozen=True)
class SensorHealth:
    """Vehicle-scoped evidence at an explicit observation time, never a control toggle."""

    vehicle_id: str
    controller_generation: str | None
    system: str
    state: HealthState
    reason: str
    evidence_age_seconds: float | None
    requested_state: str | None = None


@dataclass(frozen=True)
class SystemHealthEvidence:
    """Transport-neutral reported health and the receipt-time freshness policy."""

    system: str
    supported: bool
    present: bool | None
    healthy: bool | None
    received_monotonic: float | None
    maximum_age_seconds: float
    reason: str
