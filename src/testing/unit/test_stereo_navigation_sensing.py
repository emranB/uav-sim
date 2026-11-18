"""Optical transforms, uncertainty, source disagreement and range independence."""

import math
import time
import unittest
from dataclasses import replace
from unittest.mock import patch

from common.configuration.navigation_stereo import NavigationStereoCalibration
from common.interfaces.navigation import NavigationPose, RangeScan
from common.interfaces.obstacles import ObstacleRay, StereoObstacleFrame
from common.navigation.occupancy import GridConfiguration, ObservedGrid
from common.navigation.sensing import NavigationSensing
from common.navigation.stereo_projection import check_source_consistency, stereo_world_rays


class CameraMeasurements:
    def latest(self, run: str) -> StereoObstacleFrame:
        points = tuple((x * 0.1, 0.0, 6.0, rig) for rig in range(6) for x in range(-10, 11))
        return StereoObstacleFrame(
            "vehicle_1",
            run,
            "camera-session",
            10,
            time.monotonic(),
            time.monotonic(),
            NavigationStereoCalibration().identifier,
            points,
            (0.5,) * 6,
            0.1,
            "cpu",
        )


class EstimatedLocalization:
    def navigation_pose(self) -> NavigationPose:
        return self.navigation_pose_at(10)

    def navigation_pose_at(self, timestamp: float) -> NavigationPose:
        return NavigationPose((0, 0, 4), 0, time.monotonic(), "run", True, "OFFBOARD", timestamp)


class ForbiddenRanges:
    def latest(self) -> RangeScan:
        raise AssertionError("Camera obstacle mode must not consume range measurements")


class StereoSensingTests(unittest.TestCase):
    def test_camera_mode_never_reads_ranges(self) -> None:
        source = NavigationSensing(
            "camera",
            EstimatedLocalization(),
            ForbiddenRanges(),
            CameraMeasurements(),
            "run",
            (0, 0),
        )
        grid = source.update()
        self.assertTrue(grid.occupied)
        self.assertTrue(grid.free)
        self.assertEqual(source.metrics["range_frames_used"], 0)
        self.assertEqual(source.metrics["obstacle_sources_used"], ["stereo"])

    def test_optical_forward_right_and_rig_mount_are_transformed_once(self) -> None:
        frame = CameraMeasurements().latest("run")
        pose = NavigationPose((2, 3, 4), math.pi / 2, 1, "run", True, "OFFBOARD", 10)
        rays = stereo_world_rays(frame, pose, NavigationStereoCalibration())
        ray = next(r for r in rays if abs(r.direction_xy[0]) < 1e-6 and r.direction_xy[1] > 0.99)
        self.assertAlmostEqual(ray.origin_xy_m[0], 1.8)
        self.assertAlmostEqual(ray.origin_xy_m[1], 3)
        self.assertLess(ray.free_distance_m, 6)
        self.assertGreater(ray.occupied_end_m, 6)

    def test_uncertain_hits_win_over_same_scan_free_evidence(self) -> None:
        grid = ObservedGrid((0, 0), GridConfiguration())
        grid.observe_rays(
            1,
            (
                ObstacleRay((0, 0), (1, 0), 4, 4.1, 5.9, 0.2),
                ObstacleRay((0, 0), (1, 0), 10, None, None),
            ),
        )
        self.assertIn(grid.cell((5, 0)), grid.occupied)
        self.assertNotIn(grid.cell((5, 0)), grid.traversable())

    def test_inconsistent_flat_surface_sources_are_rejected(self) -> None:
        scan = RangeScan(10, 1, -math.pi, 2 * math.pi / 719, 20, (4.0,) * 720)
        rays = tuple(
            ObstacleRay((0, 0), (math.cos(a), math.sin(a)), 7.8, 7.9, 8.1, 0.1)
            for a in [i * 0.02 for i in range(10)]
        )
        with self.assertRaisesRegex(RuntimeError, "Inconsistent"):
            check_source_consistency(
                rays, scan, NavigationPose((0, 0, 4), 0, 1, "run", True, "OFFBOARD", 10)
            )

    def test_new_source_generation_and_stale_sensor_clock_are_rejected(self) -> None:
        camera = CameraMeasurements()
        localization = EstimatedLocalization()
        sensing = NavigationSensing(
            "camera", localization, ForbiddenRanges(), camera, "run", (0, 0)
        )
        changed = replace(camera.latest("run"), source_session_id="restarted")
        with patch.object(camera, "latest", return_value=changed):
            with self.assertRaisesRegex(RuntimeError, "restarted"):
                sensing.health()
        future = replace(localization.navigation_pose(), simulation_timestamp_seconds=11.0)
        with patch.object(localization, "navigation_pose", return_value=future):
            with self.assertRaisesRegex(RuntimeError, "sensor timestamp stale"):
                sensing.health()

    def test_fused_map_keeps_stereo_uncertainty_inside_range_free_prefix(self) -> None:
        class MeasuredRanges:
            def latest(self) -> RangeScan:
                return RangeScan(
                    10, time.monotonic(), -math.pi, 2 * math.pi / 719, 20, (6.0,) * 720
                )

            def nearest(self, timestamp_seconds: float, maximum_skew_seconds: float) -> RangeScan:
                return self.latest()

        source = NavigationSensing(
            "fused", EstimatedLocalization(), MeasuredRanges(), CameraMeasurements(), "run", (0, 0)
        )
        grid = source.update()
        self.assertIn(grid.cell((5.2, 0)), grid.occupied)
        self.assertNotIn(grid.cell((5.2, 0)), grid.traversable())
        self.assertEqual(source.metrics["obstacle_sources_used"], ["stereo", "horizontal_range"])
        self.assertEqual(source.metrics["source_consistency"]["conflicts"], 0)

    def test_fused_range_clock_is_checked_independently_of_source_skew(self) -> None:
        class OlderRanges:
            def latest(self) -> RangeScan:
                return RangeScan(
                    9.6, time.monotonic(), -math.pi, 2 * math.pi / 719, 20, (6.0,) * 720
                )

            def nearest(self, timestamp_seconds: float, maximum_skew_seconds: float) -> RangeScan:
                return self.latest()

        localization = EstimatedLocalization()
        source = NavigationSensing(
            "fused", localization, OlderRanges(), CameraMeasurements(), "run", (0, 0)
        )
        current = replace(localization.navigation_pose(), simulation_timestamp_seconds=10.6)
        with patch.object(localization, "navigation_pose", return_value=current):
            with self.assertRaisesRegex(RuntimeError, "Range sensor timestamp stale"):
                source.health()

    def test_lateral_uncertainty_does_not_extend_radial_occupied_interval(self) -> None:
        grid = ObservedGrid((0, 0), GridConfiguration())
        grid.observe_rays(1.0, (ObstacleRay((0, 0), (1, 0), 3.8, 4.0, 6.0, 0.8),))
        self.assertIn(grid.cell((5.0, 0.8)), grid.occupied)
        self.assertIn(grid.cell((5.0, -0.8)), grid.occupied)
        self.assertNotIn(grid.cell((3.6, 0)), grid.occupied)
        self.assertNotIn(grid.cell((6.4, 0)), grid.occupied)

    def test_rotated_uncertainty_remains_in_radial_and_lateral_bounds(self) -> None:
        direction = (math.sqrt(0.5), math.sqrt(0.5))
        grid = ObservedGrid((0, 0), GridConfiguration())
        grid.observe_rays(1.0, (ObstacleRay((0, 0), direction, 3.8, 4.0, 6.0, 0.8),))
        raster_tolerance = math.sqrt(2) * grid.config.resolution_m / 2
        for cell in grid.occupied:
            x, y = grid.point(cell)
            radial = x * direction[0] + y * direction[1]
            lateral = -x * direction[1] + y * direction[0]
            self.assertGreaterEqual(radial, 4.0 - raster_tolerance)
            self.assertLessEqual(radial, 6.0 + raster_tolerance)
            self.assertLessEqual(abs(lateral), 0.8 + raster_tolerance)

    def test_two_pixel_rows_resolve_far_slice_alias_without_lowering_coverage(self) -> None:
        calibration = NavigationStereoCalibration()
        frame = CameraMeasurements().latest("run")
        pose = replace(
            EstimatedLocalization().navigation_pose(), pitch_rad=math.atan(2 / calibration.focal_px)
        )
        other_rigs = tuple(point for point in frame.points if point[3] != 0)

        def far_plane(row_stride: int) -> tuple:
            return tuple(
                (
                    (column - 160) * 19 / calibration.focal_px,
                    (row - 120) * 19 / calibration.focal_px,
                    19.0,
                    0,
                )
                for row in range(0, 240, row_stride)
                for column in range(140, 181, 4)
            )

        sparse = replace(frame, points=other_rigs + far_plane(4))
        with self.assertRaisesRegex(RuntimeError, "coverage insufficient"):
            stereo_world_rays(sparse, pose, calibration)
        dense = replace(frame, points=other_rigs + far_plane(2), sampling_stride_px=(4, 2))
        rays = stereo_world_rays(dense, pose, calibration)
        self.assertGreaterEqual(len(rays), len(other_rigs) + 8)

    def test_pose_lookup_wait_cannot_extend_camera_wall_freshness(self) -> None:
        camera, localization = CameraMeasurements(), EstimatedLocalization()
        source = NavigationSensing("camera", localization, ForbiddenRanges(), camera, "run", (0, 0))
        frame = replace(camera.latest("run"), captured_monotonic=time.monotonic() - 1.7)
        pose = localization.navigation_pose_at(10.0)

        def delayed_pose(timestamp_seconds: float) -> NavigationPose:
            time.sleep(0.15)
            return pose

        with (
            patch.object(camera, "latest", return_value=frame),
            patch.object(localization, "navigation_pose_at", side_effect=delayed_pose),
        ):
            with self.assertRaisesRegex(RuntimeError, "expired while awaiting"):
                source.health()
