"""Maintain one stereo horizontal reference through the qualified GNSS handoff."""

import math
import threading
from dataclasses import replace

from common.interfaces.navigation import NavigationPose, NavigationVehicle
from integrations.px4.visual_aiding import VisualAidingGuard


class StereoHorizontalNavigation:
    """Keep the route and obstacle map in stereo coordinates for the mixed-source run."""

    def __init__(self, controller: NavigationVehicle, aiding: VisualAidingGuard) -> None:
        self.controller = controller
        self.aiding = aiding
        self.lock = threading.Lock()
        self.origin: tuple[int, int] | None = None
        self.attitude_height_origin: tuple[int, int] | None = None
        self.start_height_m = 0.0
        self.start_horizontal_m = (0.0, 0.0)
        self.initial_mag_aligned_in_flight: bool | None = None
        self.heading_offset_rad = 0.0
        self.takeoff_heading_reset_observed = False
        self.gnss_reset_observed = False

    def _position(
        self, pose: NavigationPose, stamp: float, endpoint_seconds: float
    ) -> NavigationPose:
        with self.lock:
            status, stream = self.aiding.healthy_sources(pose.run_id)
            map_generation = stream["map_generation"]
            reset_counter = status["xy_reset_counter"]
            attitude_height = status["heading_reset_counter"], status["z_reset_counter"]
            if self.origin is None:
                if pose.armed or status["gps_fix_type"] < 3 or not status["gnss_xy_active"]:
                    raise RuntimeError(
                        "Mixed-source navigation requires disarmed GPS and stereo aiding"
                    )
                self.origin = map_generation, reset_counter
                self.attitude_height_origin = attitude_height
                self.start_height_m = pose.position_enu_m[2]
                self.start_horizontal_m = pose.position_enu_m[:2]
                self.initial_mag_aligned_in_flight = status.get("mag_aligned_in_flight")
            previous_heading, previous_height = self.attitude_height_origin
            heading_counter, height_counter = attitude_height
            if height_counter != previous_height:
                raise RuntimeError("PX4 height origin changed during stereo navigation")
            if heading_counter != previous_heading:
                delta = status["delta_heading_rad"]
                # PX4 deliberately realigns magnetic heading above 1.5 m. The
                # explicit flag identifies that one initial-climb reset even
                # when its correction exceeds the small unclassified allowance.
                initial_flight_alignment = (
                    self.initial_mag_aligned_in_flight is False
                    and status.get("mag_aligned_in_flight") is True
                    and math.dist(pose.position_enu_m[:2], self.start_horizontal_m) <= 0.75
                )
                if (
                    self.takeoff_heading_reset_observed
                    or (heading_counter - previous_heading) % 256 != 1
                    or abs(pose.position_enu_m[2] - self.start_height_m) > 2.5
                    or not math.isfinite(delta)
                    or (abs(delta) > 0.02 and not initial_flight_alignment)
                    or status["gps_loss_injected"]
                ):
                    raise RuntimeError("Unexpected PX4 heading reset during stereo navigation")
                self.heading_offset_rad = delta
                self.takeoff_heading_reset_observed = True
                self.attitude_height_origin = attitude_height
            previous_map, previous_counter = self.origin
            if previous_map != map_generation:
                raise RuntimeError("Stereo coordinate origin changed during the GPS transition")
            if previous_counter != reset_counter:
                if (
                    self.gnss_reset_observed
                    or (reset_counter - previous_counter) % 256 != 1
                    or status["gps_fix_type"] != 0
                    or status["gnss_xy_active"]
                    or not status["gps_loss_injected"]
                ):
                    raise RuntimeError("Unexpected PX4 position reset during stereo navigation")
                self.gnss_reset_observed = True
                self.origin = map_generation, reset_counter
            east, north = self.aiding.horizontal_enu_at(
                stamp,
                pose.run_id,
                map_generation,
                endpoint_seconds=endpoint_seconds,
            )
            return replace(
                pose,
                position_enu_m=(east, north, pose.position_enu_m[2]),
                yaw_enu_rad=pose.yaw_enu_rad + self.heading_offset_rad,
            )

    def navigation_pose(self) -> NavigationPose:
        pose = self.controller.navigation_pose()
        if pose.simulation_timestamp_seconds is None:
            raise RuntimeError("PX4 pose is missing its simulation timestamp")
        return self._position(pose, pose.simulation_timestamp_seconds, 0.08)

    def navigation_pose_at(self, timestamp_seconds: float) -> NavigationPose:
        return self._position(
            self.controller.navigation_pose_at(timestamp_seconds),
            timestamp_seconds,
            0.04,
        )

    def send_navigation_velocity(self, velocity_enu_m_s: tuple[float, float, float]) -> None:
        with self.lock:
            angle = -self.heading_offset_rad
        east, north, up = velocity_enu_m_s
        cosine, sine = math.cos(angle), math.sin(angle)
        self.controller.send_navigation_velocity(
            (cosine * east - sine * north, sine * east + cosine * north, up)
        )

    def navigation_armed(self) -> bool:
        return self.controller.navigation_armed()

    def begin_navigation(self) -> None:
        self.controller.begin_navigation()

    def end_navigation(self) -> None:
        self.controller.end_navigation()

    def land(self) -> None:
        self.controller.land()
