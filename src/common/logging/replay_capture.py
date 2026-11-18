"""Explicit, bounded replay intervals observed independently of command authority."""

import json
import threading
import time
from dataclasses import asdict
from typing import TYPE_CHECKING, TextIO

from common.interfaces.replay import ReplayObservationSource
from common.logging.automatic_replay import AutomaticReplayRenders
from common.logging.replay_store import ReplayCaptureStore
from common.logging.storage import ensure_capacity, write_atomic, write_json_atomic

if TYPE_CHECKING:
    from common.logging.replay_export import ReplayExportJobs


class ReplayCapture:
    """Own one interval. Start acknowledges persisted headers; Stop never commands flight.

    Sources must provide bounded, read-only snapshots. Wall monotonic time defines the
    selected interval. Simulation stamps and generations are recorded without joining
    discontinuities. No planned route is substituted for unavailable observations.
    """

    def __init__(
        self,
        source: ReplayObservationSource,
        store: ReplayCaptureStore,
        jobs: "ReplayExportJobs | None" = None,
        render_on_stop: bool = True,
    ) -> None:
        if type(render_on_stop) is not bool:
            raise ValueError("Render-on-stop policy must be boolean")
        self.render_on_stop = render_on_stop
        self.source = source
        self.store = store
        self.jobs = jobs
        self.automatic = AutomaticReplayRenders(store, jobs) if jobs else None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._status: dict = {"state": "idle", "vehicle_ids": [], "camera_video_recorded": False}
        self._stop_time: float | None = None
        self._stop_reason = "user_stop"
        self.store.recover_interrupted()

    def snapshot(self) -> dict:
        """Return actual writer acknowledgement and per-vehicle evidence availability."""
        with self._lock:
            status = json.loads(json.dumps(self._status))
            if status["state"] in {"capturing", "finalizing"}:
                end = self._stop_time if self._stop_time is not None else time.monotonic()
                status["duration_wall_seconds"] = max(0, end - status["started_monotonic"])
            return status

    def inventory(self) -> dict:
        """Expose active interval and finalized packages independently of focus."""
        return {
            "capture": self.snapshot(),
            "items": self.store.inventory(),
            "offline_job": self.jobs.snapshot() if self.jobs else {"state": "unavailable"},
            "automatic_job": self.automatic.snapshot()
            if self.automatic
            else {"state": "unavailable"},
            "renderer": self.jobs.availability() if self.jobs else {"available": False},
        }

    def execute(self, operation: str, body: dict) -> dict:
        """Dispatch explicit replay intent, with no mission-triggered capture."""
        if operation == "replay/start":
            return self.start(body.get("target_ids"))
        if operation == "replay/stop":
            return self.stop()
        if operation == "replay/keep":
            result = self.store.keep(body.get("capture_id"))
            if self.automatic:
                try:
                    result["automatic_render"] = self.automatic.enqueue(
                        body.get("capture_id"), body.get("title", "Recorded 3D mission replay")
                    )
                except (OSError, ValueError, RuntimeError) as error:
                    result["automatic_render_error"] = str(error)
            return result
        if operation in {"replay/export", "replay/render", "replay/cancel"}:
            if self.jobs is None:
                raise RuntimeError("Offline replay jobs are unavailable")
            if operation == "replay/cancel":
                if self.automatic and self.automatic.snapshot().get("state") in {
                    "queued",
                    "rendering",
                }:
                    return self.automatic.cancel()
                return self.jobs.cancel()
            return self.jobs.start(body, render=operation == "replay/render")
        raise ValueError(f"Unknown replay operation: {operation}")

    def start(self, vehicle_ids: list[str]) -> dict:
        """Snapshot an explicit selection and persist writer headers before acknowledging."""
        if (
            not isinstance(vehicle_ids, list)
            or not vehicle_ids
            or len(vehicle_ids) > self.store.limits.maximum_vehicles
            or any(not isinstance(identifier, str) for identifier in vehicle_ids)
            or len(set(vehicle_ids)) != len(vehicle_ids)
            or not set(vehicle_ids).issubset(self.source.vehicle_ids())
        ):
            raise ValueError("Choose unique configured replay vehicle identifiers")
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("Replay capture is already active or finalizing")
            capture_id, path, lease = self.store.allocate()
            self._stop.clear()
            self._stop_time = None
            self._stop_reason = "user_stop"
            started = time.monotonic()
            self._status = {
                "capture_id": capture_id,
                "state": "starting",
                "vehicle_ids": list(vehicle_ids),
                "vehicle_names": {},
                "started_monotonic": started,
                "started_wall_time": time.time(),
                "path": str(path),
                "vehicles": {},
                "camera_video_recorded": False,
            }
            writers: dict[str, TextIO] = {}
            try:
                context = self.source.context()
                encoded = json.dumps(context, allow_nan=False).encode()
                context_budget = (
                    32 * 1024**2 if context.get("retained_geographic_assets") else 1024**2
                )
                if len(encoded) > min(context_budget, self.store.limits.maximum_bytes // 4):
                    raise ValueError("Replay context exceeds its bounded metadata allowance")
                write_atomic(path / "context.json", encoded + b"\n", reserve_bytes=0)
                for index, identifier in enumerate(vehicle_ids):
                    self._status["vehicles"][identifier] = {
                        "writer_available": False,
                        "samples": 0,
                        "bytes_written": 0,
                        "error": None,
                        "observation_error": None,
                    }
                    try:
                        writer = (path / f"vehicle-{index}.jsonl").open("x", encoding="utf-8")
                        writers[identifier] = writer
                        header = self._encode(
                            {"kind": "header", "vehicle_id": identifier, "capture_id": capture_id}
                        )
                        writer.write(header)
                        writer.flush()
                        self._status["vehicles"][identifier].update(
                            writer_available=True,
                            bytes_written=len(header.encode()),
                        )
                    except OSError as error:
                        self._status["vehicles"][identifier]["error"] = str(error)
                if not any(row["writer_available"] for row in self._status["vehicles"].values()):
                    raise OSError("No replay writer acknowledged a persisted header")
                write_json_atomic(path / "draft.json", self._status)
                self._status["state"] = "capturing"
                self._worker = threading.Thread(
                    target=self._run,
                    args=(writers, lease),
                    name="replay-capture",
                    daemon=True,
                )
                self._worker.start()
                return {"accepted": True, **self.snapshot()}
            except (OSError, ValueError, TypeError) as error:
                for writer in writers.values():
                    writer.close()
                self._status.update(state="error", error=str(error), partial_mission=True)
                try:
                    self.store.seal(capture_id, self._status)
                    self.store.keep(capture_id)
                finally:
                    lease.close()
                raise

    def stop(self, reason: str = "user_stop") -> dict:
        """Finalize the selected interval asynchronously; leave all vehicle state alone."""
        with self._lock:
            if self._status["state"] not in {"capturing", "finalizing"}:
                if self._status["state"] == "stopped":
                    return {"accepted": True, "duplicate": True, **self.snapshot()}
                raise RuntimeError("No replay capture is active")
            if self._stop_time is None:
                self._stop_time = time.monotonic()
                self._stop_reason = reason
            self._status["state"] = "finalizing"
            self._stop.set()
            return {"accepted": True, **self.snapshot()}

    def close(self, timeout_seconds: float = 5.0, *, reason: str = "shutdown") -> None:
        """Retain an interrupted interval before dependent observation sources close."""
        if self.automatic is not None:
            self.automatic.close()
        with self._lock:
            if self.jobs is not None:
                self.jobs.close()
            worker = self._worker
            active = worker is not None and worker.is_alive()
            if active:
                self.stop(reason)
        if active:
            worker.join(timeout_seconds)
            if worker.is_alive():
                raise TimeoutError("Replay capture did not finalize before shutdown deadline")
        if reason == "master_reset" and self.jobs is not None:
            self.automatic = AutomaticReplayRenders(self.store, self.jobs)

    def _run(self, writers: dict[str, TextIO], lease: TextIO) -> None:
        last: dict[str, tuple] = {}
        simulation_stamps: dict[str, float] = {}
        bytes_written = sum(row["bytes_written"] for row in self._status["vehicles"].values())
        context_path = self.store.path(self._status["capture_id"]) / "context.json"
        bytes_written += context_path.stat().st_size
        error_message = None
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                if now - self._status["started_monotonic"] >= self.store.limits.maximum_seconds:
                    self.stop("duration_limit")
                    break
                for identifier, writer in writers.items():
                    with self._lock:
                        row = self._status["vehicles"][identifier]
                        if not row["writer_available"] or self._stop.is_set():
                            continue
                    try:
                        try:
                            observation = self.source.observe(identifier)
                        except (OSError, ValueError, RuntimeError) as error:
                            with self._lock:
                                row["observation_error"] = str(error)
                            continue
                        if observation.vehicle_id != identifier:
                            raise ValueError("Replay source returned a different vehicle identity")
                        pose = observation.pose
                        with self._lock:
                            self._status["vehicle_names"][identifier] = observation.label
                            row["diagnostic_recorder"] = asdict(observation.recorder)
                            row["observation_error"] = observation.pose_error
                        identity = (
                            observation.controller_generation,
                            observation.world_epoch,
                            observation.aiding.map_generation if observation.aiding else None,
                            observation.aiding.xy_reset_counter if observation.aiding else None,
                            observation.aiding.z_reset_counter if observation.aiding else None,
                            observation.aiding.heading_reset_counter
                            if observation.aiding
                            else None,
                        )
                        stamp = (identity, pose.received_monotonic if pose else now)
                        previous = last.get(identifier)
                        if previous == stamp:
                            continue
                        with self._lock:
                            end = (
                                self._stop_time if self._stop_time is not None else time.monotonic()
                            )
                            if self._stop.is_set():
                                continue
                            inside_interval = pose is None or (
                                self._status["started_monotonic"] <= pose.received_monotonic <= end
                            )
                            if not inside_interval:
                                row["observation_error"] = "Pose outside selected capture interval"
                                continue
                            if pose and pose.run_id != observation.controller_generation:
                                row["observation_error"] = (
                                    "Pose belongs to a different controller generation"
                                )
                                continue
                            simulation_stamp = pose.simulation_timestamp_seconds if pose else None
                            time_jump = simulation_stamp is not None and (
                                simulation_stamp
                                < simulation_stamps.get(identifier, simulation_stamp)
                            )
                            payload = {
                                "kind": "observation",
                                "vehicle_id": identifier,
                                "observed_monotonic": time.monotonic(),
                                "observed_wall_time": time.time(),
                                "segment_changed": (
                                    previous is None or previous[0] != identity or time_jump
                                ),
                                "simulation_time_jump": time_jump,
                                "observation": asdict(observation),
                            }
                            line = self._encode(payload)
                            size = len(line.encode())
                            if (
                                bytes_written + size > self.store.limits.maximum_bytes
                                or row["samples"] >= self.store.limits.maximum_samples_per_vehicle
                            ):
                                self.stop("capacity_limit")
                                break
                            ensure_capacity(self.store.directory, size)
                            writer.write(line)
                            writer.flush()
                            row["samples"] += int(pose is not None)
                            row["bytes_written"] += size
                            bytes_written += size
                            last[identifier] = stamp
                            if simulation_stamp is not None:
                                simulation_stamps[identifier] = simulation_stamp
                    except (OSError, ValueError, RuntimeError, TypeError) as error:
                        with self._lock:
                            row["observation_error"] = str(error)
                            if isinstance(error, OSError):
                                row.update(writer_available=False, error=str(error))
                if not any(row["writer_available"] for row in self._status["vehicles"].values()):
                    error_message = (
                        "All replay writers failed; original diagnostic logging is independent"
                    )
                    self.stop("writer_failure")
                self._stop.wait(self.store.limits.sample_period_seconds)
        except (OSError, ValueError, RuntimeError) as error:
            error_message = str(error)
        finally:
            for identifier, writer in writers.items():
                try:
                    writer.close()
                except OSError as error:
                    self._status["vehicles"][identifier].update(
                        error=str(error),
                        writer_available=False,
                    )
                    error_message = str(error)
            with self._lock:
                end = self._stop_time if self._stop_time is not None else time.monotonic()
                self._status.update(
                    state="error"
                    if error_message
                    else (
                        "interrupted"
                        if self._stop_reason in {"shutdown", "master_reset"}
                        else "stopped"
                    ),
                    error=error_message,
                    stopped_monotonic=end,
                    stop_reason=self._stop_reason,
                    duration_wall_seconds=max(0, end - self._status["started_monotonic"]),
                    partial_mission=True,
                    renderable=False,
                    partial_capture=any(
                        row.get("error") or not row["samples"]
                        for row in self._status["vehicles"].values()
                    ),
                )
                try:
                    self.store.seal(self._status["capture_id"], self._status)
                    if self._status["state"] in {"error", "interrupted"}:
                        self.store.keep(self._status["capture_id"])
                except (OSError, ValueError, RuntimeError) as error:
                    self._status.update(state="error", error=f"Replay finalization failed: {error}")
                finally:
                    lease.close()
            if (
                self._status["state"] == "stopped"
                and self.automatic is not None
                and self.render_on_stop
            ):
                try:
                    self.automatic.enqueue(self._status["capture_id"])
                except (OSError, ValueError, RuntimeError) as error:
                    retained = False
                    message = str(error)
                    try:
                        self.store.keep(self._status["capture_id"])
                        retained = True
                    except (OSError, ValueError, RuntimeError) as retention_error:
                        message += f"; capture retention failed: {retention_error}"
                    with self._lock:
                        self._status["automatic_render_error"] = message
                        self._status["automatic_capture_retained"] = retained

    @staticmethod
    def _encode(payload: dict) -> str:
        return json.dumps(payload, allow_nan=False, separators=(",", ":")) + "\n"
