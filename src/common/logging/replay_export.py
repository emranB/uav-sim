"""Bounded offline export of sealed evidence; no vehicle or simulator access."""

import csv
import hashlib
import json
import math
import threading
import time
import uuid
from pathlib import Path

from common.configuration.offline_replay import OfflineReplayLimits
from common.interfaces.replay import ReplayRenderer
from common.logging.replay_store import ReplayCaptureStore
from common.logging.storage import directory_bytes, ensure_capacity, write_json_atomic


class ReplayExportJobs:
    """One cancellable streaming job at a time, reading only immutable capture inputs."""

    maximum_output_bytes = 128 * 1024**2
    maximum_store_bytes = 512 * 1024**2
    maximum_wall_seconds = 1800

    def __init__(
        self,
        store: ReplayCaptureStore,
        directory: Path,
        renderer: ReplayRenderer | None = None,
        maximum_wall_seconds: int = 1800,
        maximum_output_bytes: int = 128 * 1024**2,
        maximum_store_bytes: int = 512 * 1024**2,
    ) -> None:
        OfflineReplayLimits(maximum_wall_seconds, maximum_output_bytes, maximum_store_bytes)
        self.maximum_wall_seconds = maximum_wall_seconds
        self.maximum_output_bytes = maximum_output_bytes
        self.maximum_store_bytes = maximum_store_bytes
        self.store = store
        self.directory = directory
        self.renderer = renderer
        directory.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._worker: threading.Thread | None = None
        self._status = {"state": "idle"}
        recovery_errors = []
        for path in directory.glob("job-*/status.json"):
            try:
                status = self._read(path)
                if status.get("state") in {"queued", "running", "rendering"}:
                    write_json_atomic(
                        path,
                        {
                            **status,
                            "state": "interrupted",
                            "error": "Offline process ended before job completion",
                        },
                    )
                    self.store.keep(status["capture_id"])
            except (OSError, ValueError, KeyError, TypeError) as error:
                if len(recovery_errors) < 100:
                    recovery_errors.append(f"{path.name}: {error}")
        if recovery_errors:
            self._status["recovery_errors"] = recovery_errors

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._status)

    def availability(self) -> dict:
        return (
            self.renderer.availability()
            if self.renderer
            else {
                "available": False,
                "reason": "Optional Blender renderer is not configured",
            }
        )

    def start(self, body: dict, *, render: bool = False) -> dict:
        """Admit the exact requested offset interval, never silently shorten it."""
        identifier = body.get("capture_id")
        manifest = self.store.manifest(identifier)
        duration = manifest.get("duration_wall_seconds")
        if not isinstance(duration, (int, float)) or duration <= 0:
            raise ValueError("Capture has no finalized positive interval")
        start = body.get("start_offset_seconds", 0.0)
        end = body.get("end_offset_seconds", duration)
        if any(
            type(value) not in {int, float} or not math.isfinite(value) for value in (start, end)
        ):
            raise ValueError("Export offsets must be finite wall seconds")
        if not 0 <= start < end <= duration or end - start > 1800:
            raise ValueError("Export interval must lie entirely within the finalized capture")
        if render and not self.availability()["available"]:
            raise RuntimeError(self.availability()["reason"])
        title = body.get("title", "Recorded mission estimates")
        if (
            not isinstance(title, str)
            or len(title) > 120
            or any(ord(value) < 32 for value in title)
        ):
            raise ValueError("Replay title must be at most 120 printable characters")
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("An offline replay job is already active")
            used = directory_bytes(self.directory)
            if used + self.maximum_output_bytes > self.maximum_store_bytes:
                raise OSError("Offline replay job store is full; existing exports were preserved")
            ensure_capacity(self.directory, self.maximum_output_bytes)
            self.store.protect(identifier)
            job_id = f"job-{uuid.uuid4().hex}"
            destination = self.directory / job_id
            try:
                destination.mkdir()
                self._status = {
                    "job_id": job_id,
                    "capture_id": identifier,
                    "state": "queued",
                    "start_offset_seconds": start,
                    "end_offset_seconds": end,
                    "path": str(destination),
                    "progress": 0.0,
                    "render_requested": render,
                    "created_wall_time": time.time(),
                    "title": title,
                    "error": None,
                }
                self._publish()
                self._cancel.clear()
                self._worker = threading.Thread(
                    target=self._run,
                    args=(manifest, destination),
                    daemon=True,
                    name="replay-export",
                )
                self._worker.start()
            except (OSError, RuntimeError):
                self.store.release(identifier)
                raise
            return {"accepted": True, **self.snapshot()}

    def cancel(self) -> dict:
        """Cancel this offline reader/render job; original capture evidence stays intact."""
        with self._lock:
            if self._worker is None or not self._worker.is_alive():
                raise RuntimeError("No offline replay job is active")
            self._cancel.set()
            return {"accepted": True, "job_id": self._status["job_id"]}

    def close(self) -> None:
        with self._lock:
            worker = self._worker
            if worker is None or not worker.is_alive():
                return
            self._cancel.set()
        worker.join(10)
        if worker.is_alive():
            raise TimeoutError("Offline replay job did not acknowledge cancellation")

    def _publish(self) -> None:
        write_json_atomic(Path(self._status["path"]) / "status.json", self._status)

    def _check(self, destination: Path, deadline: float) -> None:
        if self._cancel.is_set():
            raise InterruptedError("Offline replay job cancelled; capture evidence retained")
        if time.monotonic() > deadline:
            raise TimeoutError("Offline replay job exceeded its wall-clock deadline")
        used = directory_bytes(destination)
        if used > self.maximum_output_bytes:
            raise OSError("Offline replay output reached its byte limit")
        ensure_capacity(destination, 65536)

    def _run(self, manifest: dict, destination: Path) -> None:
        identifier = manifest["capture_id"]
        deadline = time.monotonic() + self.maximum_wall_seconds
        count = 0
        try:
            self.store.manifest(identifier, verify_data=True)
            source = self.store.path(identifier)
            context = self._read(source / "context.json")
            write_json_atomic(destination / "context.json", context)
            start = manifest["started_monotonic"] + self._status["start_offset_seconds"]
            end = manifest["started_monotonic"] + self._status["end_offset_seconds"]
            paths = [source / name for name in manifest["files"] if name.endswith(".jsonl")]
            total_bytes = sum(path.stat().st_size for path in paths)
            processed = 0
            with (
                (destination / "observations.jsonl").open("x", encoding="utf-8") as events,
                (destination / "estimated-trajectories.csv").open(
                    "x", encoding="utf-8", newline=""
                ) as positions,
                (destination / "evaluation-truth.csv").open(
                    "x", encoding="utf-8", newline=""
                ) as truth,
            ):
                estimated_writer, truth_writer = csv.writer(positions), csv.writer(truth)
                estimated_writer.writerow(
                    [
                        "vehicle_id",
                        "elapsed_wall_seconds",
                        "simulation_seconds",
                        "controller_generation",
                        "world_epoch",
                        "map_generation",
                        "segment",
                        "east_m",
                        "north_m",
                        "up_m",
                        "yaw_enu_rad",
                        "frame",
                        "armed",
                        "flight_mode",
                    ]
                )
                truth_writer.writerow(
                    [
                        "vehicle_id",
                        "elapsed_wall_seconds",
                        "simulation_seconds",
                        "world_epoch",
                        "east_m",
                        "north_m",
                        "up_m",
                        "frame",
                        "purpose",
                    ]
                )
                for path in paths:
                    segment = 0
                    with path.open("rb") as stream:
                        while encoded := stream.readline(65537):
                            self._check(destination, deadline)
                            if len(encoded) > 65536:
                                raise ValueError("Replay input line exceeds 64 KiB")
                            processed += len(encoded)
                            record = json.loads(encoded)
                            if record.get("kind") != "observation":
                                continue
                            observation = record["observation"]
                            identity = record["vehicle_id"]
                            if (
                                observation["vehicle_id"] != identity
                                or identity not in manifest["vehicle_ids"]
                            ):
                                raise ValueError(
                                    "Replay observation identity differs from sealed selection"
                                )
                            if record.get("segment_changed"):
                                segment += 1
                            pose = observation.get("pose")
                            stamp = (
                                pose["received_monotonic"] if pose else record["observed_monotonic"]
                            )
                            if not start <= stamp <= end:
                                continue
                            events.write(
                                json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n"
                            )
                            elapsed = stamp - start
                            if pose:
                                if pose["run_id"] != observation["controller_generation"]:
                                    raise ValueError(
                                        "Estimated pose belongs to a different generation"
                                    )
                                estimated_writer.writerow(
                                    [
                                        identity,
                                        elapsed,
                                        pose.get("simulation_timestamp_seconds"),
                                        observation["controller_generation"],
                                        observation["world_epoch"],
                                        (observation.get("aiding") or {}).get("map_generation"),
                                        segment,
                                        *pose["position_enu_m"],
                                        pose["yaw_enu_rad"],
                                        "estimated local ENU",
                                        pose["armed"],
                                        pose["flight_mode"],
                                    ]
                                )
                                count += 1
                            actual = observation.get("evaluation_truth")
                            if actual:
                                truth_writer.writerow(
                                    [
                                        identity,
                                        elapsed,
                                        actual["simulation_time_seconds"],
                                        observation["world_epoch"],
                                        *actual["position_enu_m"],
                                        "scene ENU",
                                        "evaluation only",
                                    ]
                                )
                            with self._lock:
                                self._status.update(
                                    state="running", progress=processed / max(1, total_bytes)
                                )
            self._check(destination, deadline)
            write_json_atomic(
                destination / "export-manifest.json",
                {
                    "schema_version": 1,
                    "title": self._status["title"],
                    "capture_id": identifier,
                    "capture_manifest_sha256": hashlib.sha256(
                        (source / "manifest.json").read_bytes()
                    ).hexdigest(),
                    "start_monotonic": start,
                    "end_monotonic": end,
                    "estimated_pose_count": count,
                    "data_kind": "observed trajectory/event reconstruction inputs",
                    "camera_video_recorded": False,
                    "scene_complete": context.get("complete_render_scene_retained", False),
                    "vehicle_ids": manifest["vehicle_ids"],
                    "files": {
                        name: {
                            "bytes": (destination / name).stat().st_size,
                            "sha256": hashlib.sha256((destination / name).read_bytes()).hexdigest(),
                        }
                        for name in (
                            "observations.jsonl",
                            "estimated-trajectories.csv",
                            "evaluation-truth.csv",
                            "context.json",
                        )
                    },
                },
            )
            if self._status["render_requested"]:
                if count < 2:
                    raise ValueError(
                        "Render requires at least two actual estimated poses in the chosen interval"
                    )
                with self._lock:
                    self._status.update(state="rendering", progress=0.0)
                    self._publish()

                def progress(value: float) -> None:
                    with self._lock:
                        self._status["progress"] = value

                self.renderer.render(
                    destination,
                    self._cancel,
                    lambda: self._check(destination, deadline),
                    progress,
                )
            self._check(destination, deadline)
            with self._lock:
                self._status.update(state="completed", progress=1.0, estimated_pose_count=count)
                movie = destination / "mission-replay.mp4"
                if movie.is_file():
                    self._status["video_path"] = str(movie)
        except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
            with self._lock:
                self._status.update(
                    state="cancelled" if isinstance(error, InterruptedError) else "error",
                    error=str(error),
                )
        finally:
            with self._lock:
                if self._status["state"] in {"error", "cancelled"}:
                    try:
                        self.store.keep(identifier)
                    except (OSError, ValueError) as error:
                        self._status["retention_error"] = str(error)
                self._status["finished_wall_time"] = time.time()
                try:
                    self._publish()
                except OSError as error:
                    self._status.update(
                        state="error", error=f"Cannot persist offline job status: {error}"
                    )
            self.store.release(identifier)

    @staticmethod
    def _read(path: Path) -> dict:
        maximum_bytes = 32 * 1024**2 if path.name == "context.json" else 1024**2
        if path.is_symlink() or path.stat().st_size > maximum_bytes:
            raise ValueError("Replay metadata is outside its bounded package")
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("Replay export metadata must be an object")
        return payload


def verify_replay_export(directory: Path) -> dict:
    """Verify every retained renderer input; accept only the declared export file names."""
    manifest = ReplayExportJobs._read(directory / "export-manifest.json")
    expected_names = {
        "observations.jsonl",
        "estimated-trajectories.csv",
        "evaluation-truth.csv",
        "context.json",
    }
    if (
        set(manifest.get("files", {})) != expected_names
        or manifest.get("camera_video_recorded") is not False
    ):
        raise ValueError("Offline render requires a sealed observed-data export")
    for name, expected in manifest["files"].items():
        path = directory / name
        if path.is_symlink() or path.stat().st_size != expected["bytes"]:
            raise ValueError(f"Offline export input changed: {name}")
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024**2), b""):
                digest.update(chunk)
        if digest.hexdigest() != expected["sha256"]:
            raise ValueError(f"Offline export input changed: {name}")
    return manifest
