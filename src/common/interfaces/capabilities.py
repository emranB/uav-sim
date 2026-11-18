"""Explicit capability availability, independent of widgets and transports."""

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Capability:
    """Describe an implemented capability separately from its runtime readiness."""

    key: str
    title: str
    implemented: bool
    reason: str


class CapabilityUnavailable(RuntimeError):
    """A requested feature cannot execute; never substitute a successful no-op."""

    def __init__(self, capability: Capability, vehicle_id: str) -> None:
        self.capability = capability
        self.vehicle_id = vehicle_id
        super().__init__(
            f"{vehicle_id}: capability unavailable: {capability.title}. {capability.reason}"
        )

    def result(self) -> dict:
        """Return a versioned failure suitable for logs, API responses and tests."""
        return {
            "schema_version": 1,
            "accepted": False,
            "code": "CAPABILITY_UNAVAILABLE",
            "capability_id": self.capability.key,
            "vehicle_id": self.vehicle_id,
            "error": str(self),
        }


class CapabilityCatalog:
    """Expose only declared features and reject unknown or unavailable requests."""

    def __init__(self, capabilities: tuple[Capability, ...]) -> None:
        self._capabilities = {item.key: item for item in capabilities}
        if len(self._capabilities) != len(capabilities):
            raise ValueError("Capability identifiers must be unique")

    def snapshot(self, vehicle_id: str) -> dict:
        return {
            "schema_version": 1,
            "vehicle_id": vehicle_id,
            "items": [asdict(item) for item in self._capabilities.values()],
        }

    def require(self, key: str, vehicle_id: str) -> Capability:
        """Reject unavailable features before callers change runtime state."""
        if not isinstance(key, str) or key not in self._capabilities:
            raise ValueError("Choose a declared capability identifier")
        capability = self._capabilities[key]
        if not capability.implemented:
            raise CapabilityUnavailable(capability, vehicle_id)
        return capability
