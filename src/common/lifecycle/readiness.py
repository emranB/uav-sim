"""Bounded controller initialization barriers for application lifecycle operations."""

import math
import time
from collections.abc import Callable


def wait_for_reset_readiness(
    vehicle_id: str,
    snapshot: Callable[[], dict],
    previous_generation: str | None,
    timeout_seconds: float = 20.0,
    *,
    clock: Callable[[], float] = time.monotonic,
    wait: Callable[[float], None] = time.sleep,
) -> str:
    """Require a new, ready, disarmed controller for half a wall-clock second.

    A new process or heartbeat alone does not establish estimator initialization.
    No controller parameters or commands are changed by this passive barrier.
    """
    if (
        type(timeout_seconds) not in {int, float}
        or not math.isfinite(timeout_seconds)
        or timeout_seconds <= 0
    ):
        raise ValueError("Reset readiness timeout must be finite and positive")
    deadline = clock() + timeout_seconds
    ready_since = None
    ready_generation = None
    while True:
        now = clock()
        state = snapshot()
        generation = state.get("run_id")
        if state.get("connected") and state.get("armed") is True:
            raise RuntimeError(f"{vehicle_id}: vehicle armed during reset initialization")
        ready = (
            isinstance(generation, str)
            and bool(generation)
            and generation != previous_generation
            and state.get("connected") is True
            and state.get("ready") is True
            and state.get("armed") is False
        )
        if ready:
            if ready_since is None or ready_generation != generation:
                ready_since, ready_generation = now, generation
            elif now - ready_since >= 0.5:
                return generation
        else:
            ready_since, ready_generation = None, None
        if now >= deadline:
            raise TimeoutError(
                f"{vehicle_id}: reset controller did not reach a fresh ready/disarmed state "
                f"within {timeout_seconds:g} seconds"
            )
        wait(min(0.05, deadline - now))
