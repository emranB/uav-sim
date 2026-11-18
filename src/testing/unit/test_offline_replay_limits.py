"""Long offline jobs remain bounded without expanding ordinary profile defaults."""

import pytest

from common.configuration.offline_replay import OfflineReplayLimits


def test_defaults_preserve_short_demo_budgets() -> None:
    assert OfflineReplayLimits.from_environment({}) == OfflineReplayLimits()


def test_declared_long_capture_budget_covers_inputs_and_two_camera_outputs() -> None:
    limits = OfflineReplayLimits.from_environment(
        {
            "UAV_REPLAY_MAX_WALL_SECONDS": "3600",
            "UAV_REPLAY_OUTPUT_MAX_BYTES": str(512 * 1024**2),
            "UAV_REPLAY_JOB_STORE_MAX_BYTES": str(2 * 1024**3),
        }
    )
    assert limits.maximum_output_bytes == 512 * 1024**2
    assert limits.maximum_store_bytes == 2 * 1024**3


@pytest.mark.parametrize(
    "values",
    [
        (3601, 128 * 1024**2, 512 * 1024**2),
        (1800, 2 * 1024**3, 4 * 1024**3),
        (1800, 512 * 1024**2, 128 * 1024**2),
    ],
)
def test_unbounded_or_inconsistent_resource_policy_is_rejected(values) -> None:
    with pytest.raises(ValueError):
        OfflineReplayLimits(*values)
