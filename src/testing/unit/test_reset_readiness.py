"""Reset completion requires measured readiness in the new controller generation."""

import pytest

from common.lifecycle.readiness import wait_for_reset_readiness


class TestClock:
    __test__ = False

    def __init__(self) -> None:
        self.seconds = 0.0

    def now(self) -> float:
        return self.seconds

    def wait(self, seconds: float) -> None:
        self.seconds += seconds


def test_old_ready_generation_and_transient_ready_do_not_complete_reset() -> None:
    clock = TestClock()

    def snapshot() -> dict:
        return {
            "run_id": "old" if clock.seconds < 0.2 else "new",
            "connected": True,
            "armed": False,
            "ready": not 0.4 <= clock.seconds < 0.6,
        }

    generation = wait_for_reset_readiness(
        "vehicle_2",
        snapshot,
        "old",
        2,
        clock=clock.now,
        wait=clock.wait,
    )
    assert generation == "new"
    assert 1.1 <= clock.seconds < 1.2


def test_liveness_without_readiness_times_out_without_commands() -> None:
    clock = TestClock()
    with pytest.raises(TimeoutError, match="vehicle_2.*fresh ready/disarmed"):
        wait_for_reset_readiness(
            "vehicle_2",
            lambda: {"run_id": "new", "connected": True, "armed": False, "ready": False},
            "old",
            1,
            clock=clock.now,
            wait=clock.wait,
        )
    assert clock.seconds == 1


def test_unexpected_arming_never_reports_reset_complete() -> None:
    with pytest.raises(RuntimeError, match="armed during reset"):
        wait_for_reset_readiness(
            "vehicle_1",
            lambda: {"run_id": "new", "connected": True, "armed": True, "ready": True},
            "old",
        )


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf"), "10", True])
def test_invalid_timeout_is_rejected_before_observing_vehicle(timeout) -> None:
    def snapshot() -> dict:
        raise AssertionError("Invalid timeout must fail before observation")

    with pytest.raises(ValueError, match="finite and positive"):
        wait_for_reset_readiness("vehicle_1", snapshot, "old", timeout)
