"""A concurrent newer map cannot be compared against a pose read before publication."""

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


def test_new_map_publication_does_not_create_false_future_sensor_age() -> None:
    first_map = threading.Event()
    mapping_started = threading.Event()
    pose_read = threading.Event()
    new_map = threading.Event()

    class ConcurrentVehicle(EstimatedVehicle):
        simulation_time = 10.0
        intercepted = False

        def begin_navigation(self) -> None:
            super().begin_navigation()
            self.position = (0, 0, 4)

        def navigation_pose(self) -> NavigationPose:
            sampled = replace(
                super().navigation_pose(), simulation_timestamp_seconds=self.simulation_time
            )
            if (
                "stream" in threading.current_thread().name
                and self.armed
                and mapping_started.is_set()
                and not self.intercepted
            ):
                self.intercepted = True
                pose_read.set()
                assert new_map.wait(2), "Map publication did not reach the controlled boundary"
            return sampled

    vehicle = ConcurrentVehicle()

    class ConcurrentSensing:
        def __init__(self) -> None:
            self.metrics = {}
            self.grid = ObservedGrid((0, 0), GridConfiguration())
            self.grid.free = {(x, y): 10.0 for x in range(-10, 11) for y in range(-10, 31)}

        def health(self) -> None:
            pass

        def update(self) -> ObservedGrid:
            if first_map.is_set() and not new_map.is_set():
                # Enter mapping after the loop's cached-health lock has been
                # released, before intercepting the stream's next pose read.
                # Otherwise the fixture can block that health check while
                # waiting for mapping that the loop cannot yet reach.
                mapping_started.set()
                assert pose_read.wait(2), "The stream did not reach the controlled boundary"
            if pose_read.is_set():
                vehicle.simulation_time = 10.2
            self.observation_timestamp_seconds = vehicle.simulation_time
            self.observation_received_monotonic = time.monotonic()
            self.observation_deadline = time.monotonic() + 1.8
            self.grid.timestamp_seconds = vehicle.simulation_time
            return self.grid

    class ObservedSession(NavigationSession):
        def update(self, **values: object) -> None:
            super().update(**values)
            # Exercise the second map after the first route/evidence pair was
            # published. A completed raster alone is not a published route.
            if values.get("state") == "navigating":
                first_map.set()
            if "mapping_seconds" in values and pose_read.is_set():
                new_map.set()

    with tempfile.TemporaryDirectory() as name:
        session = ObservedSession(
            "vehicle_1", vehicle, MeasuredRanges(), threading.Lock(), Path(name), stereo=object()
        )
        with patch("common.navigation.session.NavigationSensing", return_value=ConcurrentSensing()):
            session.start({"obstacle_mode": "camera", "goal_offset_enu_m": [0, 10]})
            try:
                assert new_map.wait(4), "The race was not exercised"
                time.sleep(0.2)
            finally:
                session.cancel()
                session.thread.join(timeout=5)
        assert not session.thread.is_alive()
        assert session.snapshot()["outcome"] == "cancelled"
        assert not session.snapshot()["error"]
        assert not vehicle.armed
