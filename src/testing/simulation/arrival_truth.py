"""Bounded evaluation-only truth history, independent of HTTP polling and control."""

import math
import threading
from collections import deque


class ArrivalTruthHistory:
    """Retain one vehicle's actual timestamped ENU positions without interpolation."""

    def __init__(self, vehicle_id: str, maximum_samples: int = 20000) -> None:
        self.vehicle_id = vehicle_id
        self._samples: deque[dict] = deque(maxlen=maximum_samples)
        self._lock = threading.Lock()
        self.dropped_samples = 0

    def append(self, vehicle_id: str, timestamp_seconds: float, position_enu_m: tuple) -> None:
        """Reject other identities and invalid measurements at the observer boundary."""
        if vehicle_id != self.vehicle_id:
            raise ValueError("Arrival truth vehicle identity mismatch")
        if len(position_enu_m) != 3 or not all(
            math.isfinite(value) for value in (timestamp_seconds, *position_enu_m)
        ):
            raise ValueError("Arrival truth requires finite time and ENU position")
        with self._lock:
            if len(self._samples) == self._samples.maxlen:
                self.dropped_samples += 1
            self._samples.append(
                {
                    "vehicle_id": vehicle_id,
                    "simulation_time_seconds": timestamp_seconds,
                    "position_enu_m": list(position_enu_m),
                }
            )

    def snapshot(self) -> list[dict]:
        """Return an independent snapshot for evidence and nearest-sample scoring."""
        with self._lock:
            return [
                {**sample, "position_enu_m": list(sample["position_enu_m"])}
                for sample in self._samples
            ]
