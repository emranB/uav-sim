"""Exercise sensor-age scheduling through the real navigation session and sender."""

import tempfile
import threading
import time
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from common.interfaces.navigation import NavigationPose
from common.navigation.occupancy import GridConfiguration, ObservedGrid
from common.navigation.session import NavigationSession
from testing.unit.test_navigation_session import EstimatedVehicle, MeasuredRanges


def test_camera_mapping_refreshes_before_prior_sensor_evidence_expires() -> None:
    class RealTimeEstimatedVehicle(EstimatedVehicle):
        def navigation_pose(self) -> NavigationPose:
            return replace(super().navigation_pose(), simulation_timestamp_seconds=time.monotonic())

        def begin_navigation(self) -> None:
            super().begin_navigation()
            self.position = (0.0, 0.0, 4.0)

    class DelayedCameraSensing:
        def __init__(self) -> None:
            self.updates = 0
            self.metrics = {}
            self.grid = ObservedGrid((0, 0), GridConfiguration())
            self.grid.free = {(x, y): 0.0 for x in range(-10, 11) for y in range(-10, 31)}

        def health(self) -> None:
            pass

        def update(self) -> ObservedGrid:
            self.updates += 1
            now = time.monotonic()
            self.observation_deadline = now + 1.8
            # A valid but delayed measurement leaves 0.3s for processing/refresh;
            # the original 0.4s planning interval exhausts the sensor-age budget.
            self.observation_timestamp_seconds = now - 0.6
            self.observation_received_monotonic = now - 0.6
            self.grid.timestamp_seconds = now - 0.6
            return self.grid

    with tempfile.TemporaryDirectory() as name:
        vehicle, sensing = RealTimeEstimatedVehicle(), DelayedCameraSensing()
        session = NavigationSession(
            "vehicle_1", vehicle, MeasuredRanges(), threading.Lock(), Path(name), stereo=object()
        )
        with patch("common.navigation.session.NavigationSensing", return_value=sensing):
            session.start({"goal_offset_enu_m": [0, 10], "obstacle_mode": "camera"})
            deadline = time.monotonic() + 5
            while sensing.updates < 6 and session.snapshot()["active"]:
                assert time.monotonic() < deadline
                time.sleep(0.02)
            session.cancel()
            session.thread.join(timeout=5)
        assert not session.thread.is_alive()
        assert sensing.updates >= 6
        assert session.snapshot()["outcome"] == "cancelled"
        assert not session.snapshot()["error"]
        assert session.snapshot()["maximum_control_sensor_age_sim_seconds"] <= 0.9
        assert not vehicle.armed
