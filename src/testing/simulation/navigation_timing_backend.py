"""Opt-in bounded tracing of actual source reads, estimated arrivals and control checks."""

import argparse
import threading
import time
from pathlib import Path
from unittest.mock import patch

from common.interfaces.obstacles import StereoObstacleFrame
from common.navigation.sensing import NavigationSensing
from common.navigation.session import NavigationSession
from common.simulation_manager import server
from integrations.mavlink.connection import SimulatedVehicleConnection
from integrations.vision.navigation_source import FileStereoObstacles
from testing.simulation.timing_capture import TimingRecorder


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", required=True)
    arguments = parser.parse_args()
    recorder = TimingRecorder(Path(arguments.directory))
    original_latest = FileStereoObstacles.latest
    original_update = NavigationSession.update
    original_send = SimulatedVehicleConnection.send_navigation_velocity
    original_map = NavigationSensing.update

    def mapped(sensing: NavigationSensing) -> object:
        result = original_map(sensing)
        recorder.emit(
            {
                "kind": "actual-map-ready",
                "observed_monotonic": time.monotonic(),
                "sensor_simulation_seconds": sensing.observation_timestamp_seconds,
                "source_session_id": sensing.metrics.get("source_session_id"),
                "obstacle_mode": sensing.mode,
                "metrics": sensing.metrics,
            }
        )
        return result

    def latest(source: FileStereoObstacles, controller_run_id: str) -> StereoObstacleFrame:
        entered = time.monotonic()
        try:
            frame = original_latest(source, controller_run_id)
        except (OSError, ValueError, RuntimeError) as error:
            recorder.emit(
                {
                    "kind": "actual-source-read-error",
                    "vehicle_id": source.vehicle_id,
                    "controller_run_id": controller_run_id,
                    "read_entered_monotonic": entered,
                    "read_completed_monotonic": time.monotonic(),
                    "error": str(error),
                }
            )
            raise
        recorder.emit(
            {
                "kind": "actual-obstacle-source-read",
                "vehicle_id": frame.vehicle_id,
                "controller_run_id": frame.controller_run_id,
                "source_session_id": frame.source_session_id,
                "sensor_simulation_seconds": frame.timestamp_seconds,
                "reader_thread": threading.current_thread().name,
                "read_entered_monotonic": entered,
                "read_completed_monotonic": time.monotonic(),
                "first_delivery_monotonic": frame.captured_monotonic,
                "processing_finished_monotonic": frame.published_monotonic,
            }
        )
        return frame

    def update(session: NavigationSession, **values: object) -> None:
        original_update(session, **values)
        if "control_checked_monotonic" in values:
            snapshot = session.snapshot()
            recorder.emit(
                {
                    "kind": "actual-control-age-check",
                    "vehicle_id": session.vehicle_id,
                    "controller_run_id": snapshot.get("controller_run_id"),
                    "source_session_id": snapshot.get("source_session_id"),
                    "obstacle_mode": snapshot.get("obstacle_mode"),
                    **values,
                }
            )
        if values.get("state") == "landing":
            recorder.emit(
                {
                    "kind": "actual-navigation-termination",
                    "vehicle_id": session.vehicle_id,
                    "observed_monotonic": time.monotonic(),
                    **values,
                }
            )

    def send(connection: SimulatedVehicleConnection, velocity_enu_m_s: tuple) -> None:
        original_send(connection, velocity_enu_m_s)
        recorder.emit(
            {
                "kind": "actual-velocity-send",
                "vehicle_id": connection.vehicle_id,
                "sent_monotonic": time.monotonic(),
                "velocity_enu_m_s": velocity_enu_m_s,
            }
        )

    try:
        with (
            patch.object(FileStereoObstacles, "latest", latest),
            patch.object(NavigationSession, "update", update),
            patch.object(SimulatedVehicleConnection, "send_navigation_velocity", send),
            patch.object(NavigationSensing, "update", mapped),
        ):
            server.main()
    finally:
        recorder.close()


if __name__ == "__main__":
    main()
