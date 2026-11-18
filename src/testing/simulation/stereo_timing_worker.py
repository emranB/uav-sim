"""Opt-in bounded producer timing capture around the unchanged stereo processing path."""

import argparse
import threading
import time
from pathlib import Path
from unittest.mock import patch

from integrations.vision import navigation_stereo
from testing.simulation.timing_capture import TimingRecorder


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    arguments = parser.parse_args()
    recorder = TimingRecorder(arguments.directory)
    owners = {}
    original_write = navigation_stereo.write_atomic_json

    class TimedWorker(navigation_stereo.VehicleStereoObstacles):
        def __init__(self, *args: object) -> None:
            self._timing = {}
            self._timing_lock = threading.Lock()
            super().__init__(*args)
            owners[str(self.output)] = self

        def receive(self, rig: int, side: str, message: object) -> None:
            received = time.monotonic()
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nsec
            key = (self.source_session_id, stamp)
            with self._timing_lock:
                timing = self._timing.setdefault(key, {"sides": set()})
                timing["sides"].add((rig, side))
                if len(timing["sides"]) == 12:
                    timing.setdefault("paired_monotonic", received)
                while len(self._timing) > 128:
                    self._timing.pop(next(iter(self._timing)))
            recorder.emit(
                {
                    "kind": "image-arrival",
                    "vehicle_id": self.definition.vehicle_id,
                    "worker_source_session_id": self.source_session_id,
                    "sensor_timestamp_ns": stamp,
                    "rig": rig,
                    "side": side,
                    "received_monotonic": received,
                }
            )
            super().receive(rig, side, message)

        def process(self, batch: tuple) -> None:
            key = (self.source_session_id, batch[0])
            with self._timing_lock:
                self._timing.setdefault(key, {})["worker_entered_monotonic"] = time.monotonic()
            super().process(batch)

    def published(path: Path, payload: dict) -> None:
        before = time.monotonic()
        original_write(path, payload)
        after = time.monotonic()
        worker = owners.get(str(path))
        if worker is None or not payload.get("ready"):
            return
        key = (payload["source_session_id"], round(payload["timestamp_seconds"] * 1e9))
        with worker._timing_lock:
            timing = dict(worker._timing.get(key, {}))
        recorder.emit(
            {
                "kind": "completed-observation",
                "vehicle_id": payload["vehicle_id"],
                "controller_run_id": payload["controller_run_id"],
                "source_session_id": payload["source_session_id"],
                "calibration_id": payload["calibration_id"],
                "sensor_simulation_seconds": payload["timestamp_seconds"],
                "first_delivery_monotonic": payload["captured_monotonic"],
                "processing_finished_monotonic": payload["published_monotonic"],
                "processing_seconds": payload["processing_seconds"],
                "publication_started_monotonic": before,
                "publication_finished_monotonic": after,
                "dropped_batches": payload["dropped_batches"],
                "dropped_out_of_order_batches": payload["dropped_out_of_order_batches"],
                **{field: value for field, value in timing.items() if field != "sides"},
            }
        )

    try:
        with (
            patch.object(navigation_stereo, "VehicleStereoObstacles", TimedWorker),
            patch.object(navigation_stereo, "write_atomic_json", published),
        ):
            return navigation_stereo.main()
    finally:
        recorder.close()


if __name__ == "__main__":
    raise SystemExit(main())
