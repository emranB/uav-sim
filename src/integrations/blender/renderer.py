"""Optional isolated Blender scene process, bounded independently of simulation services."""

import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from common.logging.replay_export import verify_replay_export
from common.logging.storage import write_json_atomic
from integrations.blender.scene_inputs import retain_aircraft_assets


class BlenderReplayRenderer:
    """Render recorded 3D aircraft and retained world geometry; never access live flight or sensing owners."""

    maximum_rss_bytes = 3 * 1024**3

    def __init__(self, executable: str | None = None) -> None:
        self.executable = executable or os.environ.get("UAV_BLENDER_EXECUTABLE", "blender")

    def availability(self) -> dict:
        executable = shutil.which(self.executable)
        reason = (
            "Optional Blender executable is unavailable; set UAV_BLENDER_EXECUTABLE"
            if executable is None
            else "FFmpeg is unavailable; the recorded-data overlay requires FFmpeg"
            if not shutil.which("ffmpeg")
            else "CPU resource-limit utilities are unavailable"
            if not shutil.which("nice") or not shutil.which("prlimit")
            else None
        )
        limit, used = Path("/sys/fs/cgroup/memory.max"), Path("/sys/fs/cgroup/memory.current")
        if reason is None and limit.is_file() and limit.read_text().strip() != "max":
            if int(limit.read_text()) - int(used.read_text()) < 1024**3:
                reason = "Insufficient isolated render memory; use the optional offline worker"
        return {
            "available": reason is None,
            "reason": reason,
            "executable": executable,
            "kind": "animated 3D Blender reconstruction; not camera footage",
            "cpu_threads": 2,
            "maximum_rss_bytes": self.maximum_rss_bytes,
        }

    def render(
        self,
        directory: Path,
        cancelled: threading.Event,
        check: Callable[[], None],
        progress: Callable[[float], None],
    ) -> None:
        directory = directory.resolve()
        availability = self.availability()
        if not availability["available"]:
            raise RuntimeError(availability["reason"])
        verify_replay_export(directory)
        model_root = Path(
            os.environ.get("UAV_REPLAY_MODEL_ROOT", "/opt/px4/Tools/simulation/gz/models")
        )
        revision_file = Path(
            os.environ.get(
                "UAV_REPLAY_MODEL_REVISION_FILE", "/opt/px4/build/px4_sitl_default/models-revision"
            )
        )
        assets = retain_aircraft_assets(directory, model_root, revision_file)
        export = json.loads((directory / "export-manifest.json").read_text())
        options = {
            "source_python": str(Path(__file__).resolve().parents[2]),
            "export_path": str(directory),
            "output": str(directory),
            "assets": str(assets),
            "fps": 10,
            "width": int(os.environ.get("UAV_REPLAY_RENDER_WIDTH", "1280")),
            "height": int(os.environ.get("UAV_REPLAY_RENDER_HEIGHT", "720")),
            "samples": int(os.environ.get("UAV_REPLAY_RENDER_SAMPLES", "4")),
            "title": export["title"],
            "engine": os.environ.get("UAV_REPLAY_ENGINE", "CYCLES"),
            "compute": os.environ.get("UAV_REPLAY_COMPUTE", "CPU"),
        }
        if os.environ.get("UAV_REPLAY_LAYOUT") == "dual-drone":
            self._render_dual_replay(directory, options, cancelled, check, progress)
            return
        if os.environ.get("UAV_REPLAY_NATIVE_COMPOSITE") == "true":
            self._render_native_composite(directory, options, cancelled, check, progress)
            return
        if options["engine"] not in {"CYCLES", "BLENDER_EEVEE_NEXT"} or options["compute"] not in {
            "CPU",
            "CUDA",
            "OPTIX",
        }:
            raise ValueError("Unsupported explicit offline render engine/device")
        if (
            not 640 <= options["width"] <= 1920
            or not 360 <= options["height"] <= 1080
            or options["width"] * 9 != options["height"] * 16
            or not 1 <= options["samples"] <= 64
        ):
            raise ValueError(
                "Offline render settings require bounded 16:9 dimensions and 1..64 samples"
            )
        (directory / "scene-options.json").write_text(json.dumps(options, indent=2) + "\n")
        script = Path(__file__).with_name("render_scene.py").resolve()
        command = [
            availability["executable"],
            "--background",
            "--factory-startup",
            "--threads",
            "2",
            "--python-exit-code",
            "1",
            "--python",
            str(script),
            "--",
            str(directory / "scene-options.json"),
        ]
        self._run(command, directory, cancelled, check, progress, "blender.log")
        if any(
            not (directory / name).is_file()
            for name in ("render-manifest.json", "mission-replay.mp4", "preview.png")
        ):
            raise RuntimeError("Blender did not acknowledge the animated 3D scene output")
        self._run(
            [
                shutil.which("ffmpeg"),
                "-nostdin",
                "-v",
                "error",
                "-i",
                "mission-replay.mp4",
                "-f",
                "null",
                "-",
            ],
            directory,
            cancelled,
            check,
            lambda value: None,
            "decode.log",
        )
        progress(1.0)

    def _render_dual_replay(self, directory, options, cancelled, check, progress) -> None:
        from common.logging.replay_timeline import RecordedReplay
        from integrations.blender.dual_replay import (
            composite_command,
            mission_interval,
            write_labels,
        )

        replay = RecordedReplay(directory)
        start, end = mission_interval(replay)
        reports = {}
        script = Path(__file__).with_name("render_scene.py").resolve()
        fractions = [0.0, 0.0]
        progress_lock = threading.Lock()
        aborting = threading.Event()

        def camera_progress(index: int, value: float) -> None:
            with progress_lock:
                fractions[index] = max(fractions[index], value)
                progress(sum(fractions) * 0.45)

        def camera_check() -> None:
            if aborting.is_set():
                raise RuntimeError("The other aircraft view failed; stopping this render")
            check()

        def render_camera(index: int, identity: str, name: str) -> tuple[str, dict]:
            output = directory / name
            output.mkdir()
            selected = {
                **options,
                "output": str(output),
                "focus_vehicle_id": identity,
                "start_seconds": start,
                "end_seconds": end,
                "width": 800,
                "height": 450,
            }
            args = output / "scene-options.json"
            args.write_text(json.dumps(selected, indent=2) + "\n")
            command = [
                self.availability()["executable"],
                "--background",
                "--factory-startup",
                "--threads",
                "2",
                "--python-exit-code",
                "1",
                "--python",
                str(script),
                "--",
                str(args),
            ]
            self._run(
                command,
                output,
                cancelled,
                camera_check,
                lambda value: camera_progress(index, value),
                "blender.log",
            )
            camera_progress(index, 1.0)
            return identity, json.loads((output / "render-manifest.json").read_text())

        parallel = os.environ.get("UAV_REPLAY_PARALLEL_CAMERAS", "false")
        if parallel not in {"true", "false"}:
            raise ValueError("Parallel replay cameras must be true or false")
        cameras = ((0, "vehicle_1", "atlas"), (1, "vehicle_2", "boreal"))
        with ThreadPoolExecutor(max_workers=2 if parallel == "true" else 1) as executor:
            futures = [executor.submit(render_camera, *camera) for camera in cameras]
            try:
                for future in as_completed(futures):
                    identity, report = future.result()
                    reports[identity] = report
            except BaseException:
                aborting.set()
                for future in futures:
                    future.cancel()
                raise
        write_labels(replay, directory / "dual-labels.ass", start, end)
        self._run(
            composite_command(directory, end - start),
            directory,
            cancelled,
            check,
            lambda value: None,
            "composite.log",
        )
        for command, name in (
            (
                [
                    "ffmpeg",
                    "-nostdin",
                    "-v",
                    "error",
                    "-i",
                    "mission-replay.mp4",
                    "-frames:v",
                    "1",
                    "preview.png",
                ],
                "preview.log",
            ),
            (
                [
                    "ffmpeg",
                    "-nostdin",
                    "-v",
                    "error",
                    "-i",
                    "mission-replay.mp4",
                    "-f",
                    "null",
                    "-",
                ],
                "decode.log",
            ),
        ):
            self._run(command, directory, cancelled, check, lambda value: None, name)
        write_json_atomic(
            directory / "render-manifest.json",
            {
                "passed": True,
                "kind": "Two synchronized recorded-pose Blender cameras",
                "width": 800,
                "height": 1050,
                "fps": 10,
                "start_seconds": start,
                "end_seconds": end,
                "fully_decoded": True,
                "aircraft_views": reports,
                "sensor_labels": "Recorded native evidence; unknown and stale remain explicit",
                "rotor_animation": False,
                "parallel_aircraft_views": parallel == "true",
            },
        )
        progress(1.0)

    def _render_native_composite(self, directory, options, cancelled, check, progress) -> None:
        from integrations.blender.native_composite import (
            composite_command,
            load_native_recording,
        )

        recording = load_native_recording(directory, check)
        options["title"] = recording.get("title", options["title"])
        script = Path(__file__).with_name("render_scene.py").resolve()
        reports = {}
        for index, (identity, name) in enumerate((("vehicle_1", "atlas"), ("vehicle_2", "boreal"))):
            output = directory / name
            output.mkdir()
            selected = {
                **options,
                "output": str(output),
                "focus_vehicle_id": identity,
                "start_seconds": recording["start_seconds"],
                "end_seconds": recording["end_seconds"],
                "width": 800,
                "height": 450,
            }
            args = output / "scene-options.json"
            args.write_text(json.dumps(selected, indent=2) + "\n")
            command = [
                self.availability()["executable"],
                "--background",
                "--factory-startup",
                "--threads",
                "2",
                "--python-exit-code",
                "1",
                "--python",
                str(script),
                "--",
                str(args),
            ]
            self._run(
                command,
                output,
                cancelled,
                check,
                lambda value, i=index: progress((i + value) * 0.45),
                "blender.log",
            )
            reports[identity] = json.loads((output / "render-manifest.json").read_text())
        self._run(
            composite_command(directory, recording, options["title"]),
            directory,
            cancelled,
            check,
            lambda value: None,
            "composite.log",
        )
        self._run(
            [
                "ffmpeg",
                "-nostdin",
                "-v",
                "error",
                "-i",
                "mission-replay.mp4",
                "-frames:v",
                "1",
                "preview.png",
            ],
            directory,
            cancelled,
            check,
            lambda value: None,
            "preview.log",
        )
        self._run(
            ["ffmpeg", "-nostdin", "-v", "error", "-i", "mission-replay.mp4", "-f", "null", "-"],
            directory,
            cancelled,
            check,
            lambda value: None,
            "decode.log",
        )
        (directory / "render-manifest.json").write_text(
            json.dumps(
                {
                    "passed": True,
                    "kind": "Actual native screen recording with two synchronized Blender replay cameras",
                    "width": 2464,
                    "height": 1180,
                    "fps": 10,
                    "native_source": recording["source_manifest"],
                    "native_sha256": recording["sha256"],
                    "start_seconds": recording["start_seconds"],
                    "end_seconds": recording["end_seconds"],
                    "native_end_seconds": recording["native_end_seconds"],
                    "renderer_source_revision": os.environ.get("UAV_RENDERER_SOURCE_REVISION"),
                    "renderer_source_files_sha256": {
                        name: hashlib.sha256(
                            Path(__file__).with_name(name).read_bytes()
                        ).hexdigest()
                        for name in (
                            "renderer.py",
                            "render_scene.py",
                            "camera_policy.py",
                            "scene_camera.py",
                            "native_composite.py",
                        )
                    },
                    "aircraft_views": reports,
                    "fully_decoded": True,
                    "rotor_animation": False,
                },
                indent=2,
            )
            + "\n"
        )
        progress(1.0)

    def _run(self, command, directory, cancelled, check, progress, log_name) -> None:
        environment = dict(os.environ)
        environment.update(OMP_NUM_THREADS="2", OPENBLAS_NUM_THREADS="2")
        if environment.get("UAV_REPLAY_COMPUTE", "CPU") == "CPU":
            environment["CUDA_VISIBLE_DEVICES"] = ""
        limited_command = [
            sys.executable,
            str(Path(__file__).with_name("guarded_child.py").resolve()),
            str(os.getpid()),
            "--",
            "nice",
            "-n",
            "10",
            "prlimit",
            "--as=137438953472"
            if environment.get("UAV_REPLAY_COMPUTE", "CPU") != "CPU"
            else "--as=8589934592",
            "--cpu=7200",
            "--",
            *command,
        ]
        with (directory / log_name).open("xb") as log:
            process = subprocess.Popen(
                limited_command,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=environment,
                cwd=directory,
            )
            started, peak_rss = time.monotonic(), 0
            try:
                while process.poll() is None:
                    check()
                    if cancelled.is_set():
                        raise InterruptedError("Offline render cancelled")
                    peak_rss = max(peak_rss, self._rss(process.pid))
                    if peak_rss > self.maximum_rss_bytes:
                        raise RuntimeError(
                            f"Offline renderer exceeded its "
                            f"{self.maximum_rss_bytes // 1024**2} MiB memory limit"
                        )
                    if log_name == "blender.log":
                        progress_path = directory / "render-progress.json"
                        if progress_path.is_file() and progress_path.stat().st_size < 1024:
                            try:
                                status = json.loads(progress_path.read_text())
                            except json.JSONDecodeError:
                                pass
                            else:
                                progress(status["completed_frames"] / status["total_frames"])
                    cancelled.wait(0.1)
                if process.returncode:
                    raise RuntimeError(
                        f"Offline process exited {process.returncode}; inspect {log_name}"
                    )
                (directory / (log_name + ".resources.json")).write_text(
                    json.dumps(
                        {
                            "peak_sampled_rss_bytes": peak_rss,
                            "wall_seconds": time.monotonic() - started,
                            "cpu_threads": 2,
                            "nice": 10,
                            "compute_setting": environment.get("UAV_REPLAY_COMPUTE", "CPU"),
                            "render_engine_setting": environment.get("UAV_REPLAY_ENGINE", "CYCLES"),
                            "maximum_rss_bytes": self.maximum_rss_bytes,
                        },
                        indent=2,
                    )
                    + "\n"
                )
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=2)

    @staticmethod
    def _rss(pid: int) -> int:
        """Read only this owned process and its direct descendants on Linux."""
        pending = [pid]
        total, visited = 0, set()
        while pending:
            current = pending.pop()
            if current in visited:
                continue
            visited.add(current)
            if len(visited) > 32:
                raise RuntimeError("Offline renderer exceeded its child-process limit")
            try:
                for line in Path(f"/proc/{current}/status").read_text().splitlines():
                    if line.startswith("VmRSS:"):
                        total += int(line.split()[1]) * 1024
                children = Path(f"/proc/{current}/task/{current}/children").read_text()
                pending.extend(int(child) for child in children.split())
            except FileNotFoundError:
                continue
        return total
