"""Measured optical rays to local ENU, with explicit conservative uncertainty."""

import math
from functools import cache

from common.configuration.navigation_stereo import NavigationStereoCalibration
from common.interfaces.navigation import NavigationPose, RangeScan
from common.interfaces.obstacles import ObstacleRay, StereoObstacleFrame


def rotate_flu(
    vector: tuple[float, float, float], pose: NavigationPose
) -> tuple[float, float, float]:
    """Apply ENU Rz(yaw) Ry(pitch) Rx(roll) to an FLU vector."""
    x, y, z = vector
    cr, sr = math.cos(pose.roll_rad), math.sin(pose.roll_rad)
    cp, sp = math.cos(pose.pitch_rad), math.sin(pose.pitch_rad)
    cy, sy = math.cos(pose.yaw_enu_rad), math.sin(pose.yaw_enu_rad)
    y, z = cr * y - sr * z, sr * y + cr * z
    x, z = cp * x + sp * z, -sp * x + cp * z
    return cy * x - sy * y, sy * x + cy * y, z


def stereo_world_rays(
    frame: StereoObstacleFrame, pose: NavigationPose, calibration: NavigationStereoCalibration
) -> tuple[ObstacleRay, ...]:
    """Project only finite measured pixels near the flight slice; invalid stays unknown.

    The one-disparity-pixel interval is a conservative engineering proxy, not a
    certified covariance. Include estimated pose and half-sample angular margins.
    """
    rays = []
    counts = [0] * calibration.rig_count
    # Pose and rig transforms are constant within a captured frame. Keep the
    # existing rotation arithmetic but avoid repeating trigonometry per pixel.
    cr, sr = math.cos(pose.roll_rad), math.sin(pose.roll_rad)
    cp, sp = math.cos(pose.pitch_rad), math.sin(pose.pitch_rad)
    cy_pose, sy_pose = math.cos(pose.yaw_enu_rad), math.sin(pose.yaw_enu_rad)

    def rotate(vector: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z = vector
        y, z = cr * y - sr * z, sr * y + cr * z
        x, z = cp * x + sp * z, -sp * x + cp * z
        return cy_pose * x - sy_pose * y, sy_pose * x + cy_pose * y, z

    rigs = []
    for rig in range(calibration.rig_count):
        yaw = calibration.yaw_rad(rig)
        cy, sy = math.cos(yaw), math.sin(yaw)
        mount = (
            -sy * calibration.baseline_m / 2,
            cy * calibration.baseline_m / 2,
            calibration.mount_height_m,
        )
        origin_delta = rotate(mount)
        origin = tuple(pose.position_enu_m[i] + origin_delta[i] for i in range(3))
        rigs.append((cy, sy, origin))
    focal_px = calibration.focal_px
    slice_height_m = pose.position_enu_m[2] + calibration.mount_height_m
    for right, down, forward, rig in frame.points:
        cy, sy, origin = rigs[rig]
        optical_body = (cy * forward + sy * right, sy * forward - cy * right, -down)
        delta = rotate(optical_body)
        if abs(origin[2] + delta[2] - slice_height_m) > 0.2:
            continue
        distance = math.hypot(delta[0], delta[1])
        if not 1.0 <= distance <= 20:
            continue
        uncertainty = (
            0.25 + (forward * forward / (focal_px * calibration.baseline_m)) * distance / forward
        )
        if distance - uncertainty < 0.5:
            continue
        rays.append(
            ObstacleRay(
                origin[:2],
                (delta[0] / distance, delta[1] / distance),
                max(0, distance - uncertainty - 0.1),
                max(0, distance - uncertainty),
                distance + uncertainty,
                0.25 + distance * 2 / focal_px,
            )
        )
        counts[rig] += 1
    if min(counts) < 8:
        raise RuntimeError(f"Stereo flight-slice coverage insufficient per rig: {counts}")
    return tuple(rays)


def range_world_rays(scan: RangeScan, pose: NavigationPose) -> tuple[ObstacleRay, ...]:
    """Project a measured range scan at its estimated pose; no message truth pose."""
    mount = rotate_flu((0, 0, 0.35), pose)
    origin = tuple(pose.position_enu_m[i] + mount[i] for i in range(3))
    rays = []
    for index, distance in enumerate(scan.ranges_m):
        if math.isnan(distance) or distance < 0.8:
            continue
        angle = scan.angle_min_rad + index * scan.angle_step_rad
        direction = rotate_flu((math.cos(angle), math.sin(angle), 0), pose)
        horizontal = math.hypot(direction[0], direction[1])
        length = min(distance, scan.maximum_range_m) * horizontal
        hit = math.isfinite(distance) and distance < scan.maximum_range_m - 0.1
        rays.append(
            ObstacleRay(
                origin[:2],
                (direction[0] / horizontal, direction[1] / horizontal),
                max(0, length - 0.25),
                max(0, length - 0.15) if hit else None,
                length + 0.15 if hit else None,
                0.15,
            )
        )
    return tuple(rays)


def check_source_consistency(
    camera: tuple[ObstacleRay, ...], scan: RangeScan, pose: NavigationPose
) -> dict:
    """Reject substantial disagreement away from range depth discontinuities.

    A wide local angular neighbourhood must agree before comparison, avoiding
    expected parallax/occlusion edges from the offset stereo optical centres.
    """
    width = max(1, math.ceil(0.0524 / scan.angle_step_rad))

    @cache
    def reference_at(index: int) -> float | None:
        # Dense stereo rays often map to the same range beam. Its neighbourhood
        # and median are fixed for this scan.
        neighbourhood = [
            min(scan.maximum_range_m, scan.ranges_m[i])
            for i in range(max(0, index - width), min(len(scan.ranges_m), index + width + 1))
            if scan.ranges_m[i] >= 0.8
        ]
        if len(neighbourhood) < 3 or max(neighbourhood) - min(neighbourhood) > 0.6:
            return None
        return sorted(neighbourhood)[len(neighbourhood) // 2]

    compared = 0
    conflicts = 0
    for ray in camera:
        if ray.occupied_start_m is None:
            continue
        distance = (ray.occupied_start_m + ray.occupied_end_m) / 2
        endpoint = (
            ray.origin_xy_m[0] + distance * ray.direction_xy[0],
            ray.origin_xy_m[1] + distance * ray.direction_xy[1],
        )
        bearing = (
            math.atan2(endpoint[1] - pose.position_enu_m[1], endpoint[0] - pose.position_enu_m[0])
            - pose.yaw_enu_rad
        )
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        index = round((bearing - scan.angle_min_rad) / scan.angle_step_rad)
        reference = reference_at(index)
        if reference is None:
            continue
        compared += 1
        radial = math.dist(endpoint, pose.position_enu_m[:2])
        tolerance = (ray.occupied_end_m - ray.occupied_start_m) / 2 + 0.8
        if abs(reference - radial) > tolerance:
            conflicts += 1
    ratio = conflicts / compared if compared else 0
    if conflicts >= 4 and ratio > 0.03:
        raise RuntimeError(
            f"Inconsistent stereo/range evidence: {conflicts}/{compared} comparisons"
        )
    return {"comparisons": compared, "conflicts": conflicts, "conflict_fraction": ratio}
