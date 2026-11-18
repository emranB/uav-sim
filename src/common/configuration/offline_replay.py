"""Explicit resource budgets shared by offline replay export and rendering."""

from collections.abc import Mapping
from dataclasses import dataclass


@dataclass(frozen=True)
class OfflineReplayLimits:
    """Bound wall time and total bytes, including the retained input package."""

    maximum_wall_seconds: int = 1800
    maximum_output_bytes: int = 128 * 1024**2
    maximum_store_bytes: int = 512 * 1024**2

    def __post_init__(self) -> None:
        if type(self.maximum_wall_seconds) is not int or not 1 <= self.maximum_wall_seconds <= 3600:
            raise ValueError("Offline replay deadline must lie within1..3600 wall seconds")
        if (
            type(self.maximum_output_bytes) is not int
            or not 64 * 1024**2 <= self.maximum_output_bytes <= 1024**3
        ):
            raise ValueError("Offline replay job budget must lie within64..1024 MiB")
        if (
            type(self.maximum_store_bytes) is not int
            or not self.maximum_output_bytes <= self.maximum_store_bytes <= 8 * 1024**3
        ):
            raise ValueError("Offline replay store budget must cover one job and not exceed8 GiB")

    @classmethod
    def from_environment(cls, environment: Mapping[str, str]) -> "OfflineReplayLimits":
        """Read deployment settings; defaults preserve the ordinary short-demo policy."""
        return cls(
            int(environment.get("UAV_REPLAY_MAX_WALL_SECONDS", "1800")),
            int(environment.get("UAV_REPLAY_OUTPUT_MAX_BYTES", str(128 * 1024**2))),
            int(environment.get("UAV_REPLAY_JOB_STORE_MAX_BYTES", str(512 * 1024**2))),
        )
