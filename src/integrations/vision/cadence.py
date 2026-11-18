"""Choose a regular processing cadence from a bounded camera handoff."""

import math


def live_frame_rate(capture_rate_hz: int, requested_rate_hz: int) -> int:
    """Bound estimator cadence by the actual capture rate, in simulation Hz."""
    for rate in (capture_rate_hz, requested_rate_hz):
        if type(rate) is not int or not 1 <= rate <= 10:
            raise ValueError("Capture and estimator rates must be integers in 1..10 Hz")
    return min(capture_rate_hz, requested_rate_hz)


def next_stereo_frame(status: dict, previous_timestamp_ns: int, rate_hz: int) -> dict | None:
    """Select the oldest eligible pair; avoid latest-only jumps under CPU load."""
    if type(rate_hz) is not int or not 1 <= rate_hz <= 10:
        raise ValueError("Live processing cadence must be an integer in 1..10 Hz")
    frames = status.get("available_frames") or [
        {
            "simulation_timestamp_ns": status["simulation_timestamp_ns"],
            "received_wall_time": status["received_wall_time"],
        }
    ]
    if not isinstance(frames, list) or len(frames) > 12:
        raise ValueError("Live stereo handoff exceeds its 12-pair budget")
    for frame in frames:
        if (
            not isinstance(frame, dict)
            or type(frame.get("simulation_timestamp_ns")) is not int
            or frame["simulation_timestamp_ns"] < 0
            or type(frame.get("received_wall_time")) not in {int, float}
            or not math.isfinite(frame["received_wall_time"])
        ):
            raise ValueError("Live stereo handoff requires finite timing for each pair")
    if previous_timestamp_ns < 0:
        return frames[-1]
    period_ns = round(1e9 / rate_hz)
    # Gazebo quantizes publication to physics steps. Allow one small step of jitter.
    target_ns = previous_timestamp_ns + period_ns - 5_000_000
    return next((frame for frame in frames if frame["simulation_timestamp_ns"] >= target_ns), None)
