"""Declared per-vehicle perception features; configuration is not readiness."""

from common.interfaces.capabilities import Capability, CapabilityCatalog


def vehicle_perception(
    stereo_enabled: bool = False,
    mapping_enabled: bool = False,
    navigation_enabled: bool = False,
) -> CapabilityCatalog:
    """Report the actual vehicle pipeline, not an independent sensor test rig."""
    return CapabilityCatalog(
        (
            Capability(
                "stereo_capture",
                "Stereo cameras",
                stereo_enabled,
                "Rendered cameras are configured; check frame freshness before use."
                if stereo_enabled
                else "The active vehicle has no integrated stereo image stream.",
            ),
            Capability(
                "stereo_depth",
                "Live stereo depth",
                mapping_enabled,
                "Observation-only stereo processing; check fresh valid measurements."
                if mapping_enabled
                else "No per-vehicle depth service is configured.",
            ),
            Capability(
                "visual_odometry",
                "Live visual odometry",
                mapping_enabled,
                "ORB-SLAM camera-motion estimates; no flight control authority."
                if mapping_enabled
                else "No live per-vehicle visual motion service is configured.",
            ),
            Capability(
                "visual_slam",
                "Live visual SLAM",
                mapping_enabled,
                "Independent ORB-SLAM3 sessions; readiness requires current tracked frames."
                if mapping_enabled
                else "No live per-vehicle mapping service is configured.",
            ),
            Capability(
                "surface_geometry",
                "Measured surface geometry",
                mapping_enabled,
                "Stereo-derived surface samples; unseen space remains unknown."
                if mapping_enabled
                else "No measured surface service is configured.",
            ),
            Capability(
                "object_detection",
                "Semantic object detection",
                False,
                "No semantic classifier is connected. Geometric image-region analysis is separate.",
            ),
            Capability(
                "obstacle_avoidance",
                "Fixed-altitude obstacle avoidance",
                navigation_enabled,
                "Static navigation with explicitly selected obstacle sensing and PX4 estimates; check source readiness."
                if navigation_enabled
                else "Enable the explicit simulation avoidance overlay.",
            ),
            Capability(
                "path_planning",
                "Observed-free-space planning",
                navigation_enabled,
                "Bounded local grid planning; unknown space is not traversable."
                if navigation_enabled
                else "Enable the explicit simulation avoidance overlay.",
            ),
            Capability(
                "recovery",
                "Autonomous recovery",
                False,
                "This scenario has no executable recovery capability.",
            ),
            Capability(
                "coordination",
                "Cooperative recovery",
                False,
                "This scenario has no executable cooperative recovery capability.",
            ),
        )
    )
