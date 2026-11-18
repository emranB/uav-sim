"""Metric trajectory evaluation without estimator or simulator dependencies."""

import numpy as np
import pytest

from integrations.vision.evaluation import camera_position, evaluate_trajectory, rigid_alignment


def test_rigid_frame_change_is_removed_without_fitting_scale() -> None:
    truth = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 2.0, 0.0], [0.0, 2.0, 1.0]])
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    estimate = truth @ rotation + [4, 5, 6]
    fitted, translation = rigid_alignment(estimate, truth)
    np.testing.assert_allclose(estimate @ fitted + translation, truth, atol=1e-12)
    fitted, translation = rigid_alignment(estimate * 2, truth)
    assert np.linalg.norm(estimate * 2 @ fitted + translation - truth) > 1
    assert np.linalg.det(fitted) == pytest.approx(1)


def test_stationary_alignment_remains_unknown() -> None:
    with pytest.raises(ValueError, match="underconstrained"):
        rigid_alignment(np.zeros((4, 3)), np.zeros((4, 3)))


def test_camera_offset_rotates_with_body() -> None:
    half = np.sqrt(0.5)
    np.testing.assert_allclose(
        camera_position([1, 2, 3], [0, 0, half, half], [0.4, 0.08, 0.15]),
        [0.92, 2.4, 3.15],
        atol=1e-12,
    )


def test_invalid_stale_unmatched_and_duplicates_are_not_accuracy_samples() -> None:
    truth = [
        dict(
            epoch=1,
            stale=False,
            simulation_time_seconds=t,
            position_enu_m=p,
            orientation_xyzw=[0, 0, 0, 1],
        )
        for t, p in [(0, [0, 0, 0]), (0.1, [1, 0, 0]), (0.2, [1, 1, 0])]
    ]
    observations = [
        dict(
            epoch=1,
            session_id="a",
            map_id=0,
            map_generation=0,
            simulation_timestamp_ns=int(t * 1e9),
            position_map_m=p,
            position_valid=True,
            stale=False,
        )
        for t, p in [(0, [0, 0, 0]), (0.1, [1, 0, 0]), (0.2, [1, 1, 0])]
    ]
    observations += [
        observations[0],
        {**observations[0], "simulation_timestamp_ns": 50_000_000, "position_valid": False},
        {**observations[0], "simulation_timestamp_ns": 60_000_000, "stale": True},
        {**observations[0], "epoch": 2},
    ]
    result = evaluate_trajectory(observations, truth, [0, 0, 0])
    assert result["segments"][0]["position_rmse_m"] < 1e-12
    assert result["segments"][0]["samples"] == 3
    assert result["invalid_observations"] == 1
    assert result["stale_observations"] == 1
    assert result["unmatched_observations"] == 1


def test_tiny_stationary_jitter_cannot_certify_alignment() -> None:
    points = np.array([[0, 0, 0], [1e-8, 0, 0], [0, 1e-8, 0]])
    with pytest.raises(ValueError, match="underconstrained"):
        rigid_alignment(points, points)


def test_interpolation_is_bounded_and_maps_remain_separate() -> None:
    truth = [
        dict(
            epoch=1,
            stale=False,
            simulation_time_seconds=t,
            position_enu_m=p,
            orientation_xyzw=[0, 0, 0, 1],
        )
        for t, p in [
            (0, [0, 0, 0]),
            (0.1, [1, 0, 0]),
            (0.2, [1, 1, 0]),
            (0.3, [0, 1, 0]),
            (2, [0, 0, 0]),
        ]
    ]
    observations = [
        dict(
            epoch=1,
            session_id="a",
            map_id=0,
            map_generation=0,
            simulation_timestamp_ns=int(t * 1e9),
            position_map_m=p,
            position_valid=True,
            stale=False,
        )
        for t, p in [(0.05, [0.5, 0, 0]), (0.15, [1, 0.5, 0]), (0.25, [0.5, 1, 0]), (1, [0, 0, 0])]
    ]
    observations.append({**observations[0], "map_id": 1})
    result = evaluate_trajectory(observations, truth, [0, 0, 0])
    assert result["unmatched_observations"] == 1
    assert len(result["segments"]) == 2
    assert result["segments"][0]["position_rmse_m"] < 1e-12
    assert result["segments"][1]["status"] == "unknown"


def test_observation_backend_labels_are_not_timing_samples() -> None:
    from testing.simulation.observation import summarize

    record = dict(
        scene=dict(epoch=1, stale=False, poses=[]),
        mapping=dict(
            vehicle_1=dict(
                session_id="a",
                map_id=0,
                map_generation=0,
                position_valid=False,
                simulation_timestamp_ns=1,
                surface_backend="vpi-cuda",
                processing_seconds=0.01,
                stale=False,
            )
        ),
    )
    result = summarize([record], [0.4, 0.08, 0.15])
    assert result["vehicle_1"]["timings"]["processing_seconds"]["p50"] == 0.01
    assert "surface_backend" not in result["vehicle_1"]["timings"]


def test_evaluation_mount_requires_declared_calibration() -> None:
    from integrations.vision.evaluation import camera_offset_from_calibration

    assert camera_offset_from_calibration({"baseline_m": 0.4, "mount_height_m": 0.5}) == [
        0.4,
        0.2,
        0.5,
    ]
    with pytest.raises(ValueError, match="declared"):
        camera_offset_from_calibration({"baseline_m": 0.4})


def test_origin_resets_never_join_in_one_alignment() -> None:
    truth = [
        dict(
            epoch=1,
            stale=False,
            simulation_time_seconds=i * 0.1,
            position_enu_m=[i % 3, i // 3, 0],
            orientation_xyzw=[0, 0, 0, 1],
        )
        for i in range(6)
    ]
    observations = [
        dict(
            epoch=1,
            session_id="a",
            map_id=0,
            map_generation=i // 3,
            simulation_timestamp_ns=i * 100_000_000,
            position_valid=True,
            position_map_m=[i % 3, i // 3, 0],
            stale=False,
        )
        for i in range(6)
    ]
    result = evaluate_trajectory(observations, truth, [0, 0, 0])
    assert [segment["map_generation"] for segment in result["segments"]] == [0, 1]
    assert [segment["samples"] for segment in result["segments"]] == [3, 3]
    legacy = [
        {key: value for key, value in item.items() if key != "map_generation"}
        for item in observations
    ]
    legacy.insert(
        3,
        {
            **legacy[2],
            "simulation_timestamp_ns": 250_000_000,
            "map_id": -1,
            "position_valid": False,
        },
    )
    result = evaluate_trajectory(legacy, truth, [0, 0, 0])
    assert len(result["segments"]) == 2
