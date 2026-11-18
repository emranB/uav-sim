"""Machine-level settings, separate from portable experiment definitions."""

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RuntimeSettings:
    """Requested deployment settings; these do not establish hardware capabilities."""

    render_mode: str = "cpu"
    headless: bool = True
    ros_domain_id: int = 42
    rmw_implementation: str = "rmw_fastrtps_cpp"
    output_directory: Path = Path("outputs")
    heartbeat_timeout_seconds: float = 5.0
    real_time_factor: float = 1.0

    @classmethod
    def from_environment(cls, environment: Mapping[str, str]) -> "RuntimeSettings":
        """Reject invalid settings before starting services or writing output files."""
        render_mode = environment.get("UAV_RENDER_MODE", "cpu").strip().lower()
        if render_mode not in {"cpu", "gpu"}:
            raise ValueError("UAV_RENDER_MODE must be cpu or gpu")
        headless = environment.get("UAV_HEADLESS", "true").strip().lower()
        if headless not in {"true", "false"}:
            raise ValueError("UAV_HEADLESS must be true or false")
        try:
            domain_id = int(environment.get("ROS_DOMAIN_ID", "42"))
        except ValueError as error:
            raise ValueError("ROS_DOMAIN_ID must be an integer in 0..232") from error
        if not 0 <= domain_id <= 232:
            raise ValueError("ROS_DOMAIN_ID must be an integer in 0..232")
        rmw = environment.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp").strip()
        output_directory = environment.get("OUTPUT_DIR", "outputs").strip()
        if not rmw or not output_directory:
            raise ValueError("RMW_IMPLEMENTATION and OUTPUT_DIR must not be empty")
        try:
            heartbeat_timeout = float(environment.get("UAV_HEARTBEAT_TIMEOUT_SECONDS", "5"))
        except ValueError as error:
            raise ValueError("UAV_HEARTBEAT_TIMEOUT_SECONDS must be a number in 1..30") from error
        if not math.isfinite(heartbeat_timeout) or not 1 <= heartbeat_timeout <= 30:
            raise ValueError("UAV_HEARTBEAT_TIMEOUT_SECONDS must be finite and in 1..30")
        real_time_factor = float(environment.get("UAV_REAL_TIME_FACTOR", "1"))
        if not math.isfinite(real_time_factor) or not 0.05 <= real_time_factor <= 2:
            raise ValueError("UAV_REAL_TIME_FACTOR must be finite and in 0.05..2")
        return cls(
            render_mode,
            headless == "true",
            domain_id,
            rmw,
            Path(output_directory),
            heartbeat_timeout,
            real_time_factor,
        )

    def rendering_environment(self, inherited: Mapping[str, str]) -> dict[str, str]:
        """Prepare Linux renderer settings for launchers without mutating the caller."""
        environment = dict(inherited)
        if self.render_mode == "cpu":
            environment["LIBGL_ALWAYS_SOFTWARE"] = "true"
        else:
            environment.pop("LIBGL_ALWAYS_SOFTWARE", None)
        return environment

    def requested_capabilities(self) -> dict[str, object]:
        """Expose configuration intent separately from verified renderer availability."""
        return {
            "render_mode": self.render_mode,
            "headless": self.headless,
            "ros_domain_id": self.ros_domain_id,
            "rmw_implementation": self.rmw_implementation,
            "heartbeat_timeout_seconds": self.heartbeat_timeout_seconds,
            "real_time_factor": self.real_time_factor,
        }
