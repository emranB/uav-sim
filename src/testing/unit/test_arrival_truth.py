"""Truth scoring retains measurements and identity without manufacturing samples."""

import pytest

from testing.simulation.arrival_truth import ArrivalTruthHistory


def test_actual_samples_remain_bounded_and_snapshots_are_independent() -> None:
    history = ArrivalTruthHistory("vehicle_2", maximum_samples=2)
    for stamp in (1.0, 1.6, 1.7):
        history.append("vehicle_2", stamp, (6.0, stamp, 4.0))
    samples = history.snapshot()
    assert [sample["simulation_time_seconds"] for sample in samples] == [1.6, 1.7]
    assert history.dropped_samples == 1
    samples[0]["position_enu_m"][0] = 999
    assert history.snapshot()[0]["position_enu_m"][0] == 6


@pytest.mark.parametrize(
    "identity,stamp,position",
    [
        ("vehicle_1", 1.0, (0, 0, 4)),
        ("vehicle_2", float("nan"), (0, 0, 4)),
        ("vehicle_2", 1.0, (0, float("inf"), 4)),
    ],
)
def test_wrong_identity_and_nonfinite_truth_are_rejected(identity, stamp, position) -> None:
    history = ArrivalTruthHistory("vehicle_2")
    with pytest.raises(ValueError):
        history.append(identity, stamp, position)
    assert not history.snapshot()
