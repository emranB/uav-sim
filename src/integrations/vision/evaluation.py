"""Offline metric-stereo evaluation; this module never supplies estimator inputs."""

from bisect import bisect_left
from collections import defaultdict

import numpy as np


def camera_offset_from_calibration(calibration: dict) -> list[float]:
    """Read the declared stereo mount; missing geometry must never use probe defaults."""
    baseline = calibration.get("baseline_m")
    height = calibration.get("mount_height_m")
    if (
        type(baseline) not in {int, float}
        or not np.isfinite(baseline)
        or not 0.05 <= baseline <= 0.8
        or type(height) not in {int, float}
        or not np.isfinite(height)
        or not 0.05 <= height <= 2
    ):
        raise ValueError("Evaluation requires declared stereo baseline and mount height")
    return [0.4, baseline / 2, height]


def camera_position(position: list, orientation: list, offset: list) -> np.ndarray:
    """Move body-origin ENU truth to the left camera using a body-frame offset in metres."""
    quaternion = np.asarray(orientation, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("Truth requires a finite XYZW quaternion")
    if abs(np.linalg.norm(quaternion) - 1) > 0.01:
        raise ValueError("Truth quaternion must be normalized")
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    coordinates = np.asarray([position, offset], dtype=float)
    if coordinates.shape != (2, 3) or not np.isfinite(coordinates).all():
        raise ValueError("Truth position and camera offset require three finite metres")
    return coordinates[0] + rotation @ coordinates[1]


def rigid_alignment(estimate: np.ndarray, truth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Fit a proper SE(3) transform without scale; reject unobservable line trajectories."""
    if estimate.shape != truth.shape or estimate.ndim != 2 or estimate.shape[1] != 3:
        raise ValueError("Alignment requires equally sized Nx3 trajectories")
    if len(estimate) < 3 or not np.isfinite([estimate, truth]).all():
        raise ValueError("Alignment requires at least three finite pairs")
    centered_estimate = estimate - estimate.mean(axis=0)
    centered_truth = truth - truth.mean(axis=0)
    # Floating-point jitter at rest must not make an unobservable alignment look measured.
    if any(
        np.linalg.svd(points, compute_uv=False)[1] / np.sqrt(len(points)) < 0.01
        for points in (centered_estimate, centered_truth)
    ):
        raise ValueError("Alignment is underconstrained by stationary or collinear samples")
    left, _, right = np.linalg.svd(centered_estimate.T @ centered_truth)
    correction = np.diag([1.0, 1.0, np.linalg.det(left @ right)])
    rotation = left @ correction @ right
    return rotation, truth.mean(axis=0) - estimate.mean(axis=0) @ rotation


def evaluate_trajectory(
    observations: list[dict],
    truth_samples: list[dict],
    camera_offset_m: list[float],
    maximum_truth_gap_seconds: float = 0.2,
) -> dict:
    """Interpolate camera truth at estimator timestamps, separately per epoch/session/map.

    Full-trajectory rigid alignment is evaluation-only and can conceal constant frame
    offsets. Relative displacement error retains metric scale and measures accumulated
    shape error. Missing/invalid poses never become zero-error samples.
    """
    if not 0 < maximum_truth_gap_seconds <= 1:
        raise ValueError("Truth interpolation gap must be in (0, 1] seconds")
    truth_by_epoch = defaultdict(dict)
    for sample in truth_samples:
        if not sample.get("stale", True):
            timestamp = float(sample["simulation_time_seconds"])
            if not np.isfinite(timestamp):
                raise ValueError("Truth timestamp must be finite")
            truth_by_epoch[sample["epoch"]][timestamp] = camera_position(
                sample["position_enu_m"], sample["orientation_xyzw"], camera_offset_m
            )
    timelines = {epoch: sorted(samples.items()) for epoch, samples in truth_by_epoch.items()}
    groups = defaultdict(list)
    invalid = unmatched = stale = 0
    seen = set()
    legacy_segments = defaultdict(int)
    for observation in sorted(observations, key=lambda item: item["simulation_timestamp_ns"]):
        base_key = (observation["epoch"], observation["session_id"], observation["map_id"])
        generation = observation.get("map_generation")
        key = (*base_key, generation)

        timestamp = observation["simulation_timestamp_ns"] / 1e9
        identity = (*key, timestamp)
        if identity in seen:
            continue
        seen.add(identity)
        if not observation["position_valid"]:
            if generation is None:
                legacy_segments[base_key[:2]] += 1
            invalid += 1
            continue
        if observation.get("stale", True):
            stale += 1
            continue
        timeline = timelines.get(key[0], [])
        stamps = [item[0] for item in timeline]
        index = bisect_left(stamps, timestamp)
        if index < len(stamps) and abs(stamps[index] - timestamp) < 1e-8:
            target = timeline[index][1]
        elif (
            0 < index < len(stamps)
            and stamps[index] - stamps[index - 1] <= maximum_truth_gap_seconds
        ):
            fraction = (timestamp - stamps[index - 1]) / (stamps[index] - stamps[index - 1])
            target = timeline[index - 1][1] * (1 - fraction) + timeline[index][1] * fraction
        else:
            unmatched += 1
            continue
        position = np.asarray(observation["position_map_m"], dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("Estimated position must contain three finite metres")
        # Old records cannot prove map-origin continuity across a tracking interruption.
        group_key = (*key, legacy_segments[base_key[:2]] if generation is None else 0)
        groups[group_key].append((timestamp, position, target))
    results = []
    for key, pairs in groups.items():
        pairs.sort(key=lambda pair: pair[0])
        estimate = np.array([pair[1] for pair in pairs])
        truth = np.array([pair[2] for pair in pairs])
        result = dict(
            epoch=key[0],
            session_id=key[1],
            map_id=key[2],
            map_generation=key[3],
            legacy_tracking_segment=key[4] if key[3] is None else None,
            samples=len(pairs),
        )
        try:
            rotation, translation = rigid_alignment(estimate, truth)
            aligned = estimate @ rotation + translation
            errors = np.linalg.norm(aligned - truth, axis=1)
            relative = (aligned - aligned[0]) - (truth - truth[0])
            distance = float(np.linalg.norm(np.diff(truth, axis=0), axis=1).sum())
            result.update(
                status="measured",
                position_rmse_m=float(np.sqrt(np.mean(errors**2))),
                maximum_position_error_m=float(errors.max()),
                endpoint_displacement_error_m=float(np.linalg.norm(relative[-1])),
                maximum_displacement_error_m=float(np.linalg.norm(relative, axis=1).max()),
                truth_path_length_m=distance,
                duration_seconds=pairs[-1][0] - pairs[0][0],
            )
        except ValueError as error:
            result.update(status="unknown", reason=str(error))
        results.append(result)
    return dict(
        schema_version=1,
        status="measured"
        if any(result["status"] == "measured" for result in results)
        else "unknown",
        alignment="per coordinate-frame generation SE(3), no scale fit, evaluation only",
        legacy_identity_limitation="Records without map_generation are split at observed losses; "
        "unobserved origin resets remain unknown",
        unique_observations=len(seen),
        invalid_observations=invalid,
        stale_observations=stale,
        unmatched_observations=unmatched,
        segments=results,
        acceptance="No accuracy threshold certified; descriptive measurements only",
        control_authority=False,
    )
