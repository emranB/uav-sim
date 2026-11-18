"""Optional network-free offline rendering process; no live simulation imports."""

import fcntl
import os
import re
import signal
import threading
import time
from pathlib import Path

from common.configuration.offline_replay import OfflineReplayLimits
from common.logging.storage import directory_bytes, ensure_capacity, write_json_atomic
from integrations.blender.queue import FileReplayRenderer
from integrations.blender.renderer import BlenderReplayRenderer


def main() -> int:
    runtime = Path("/runtime/replay-renderer")
    jobs = Path(os.environ.get("OUTPUT_DIR", "outputs")) / "replay-jobs"
    runtime.mkdir(parents=True, exist_ok=True)
    lease = (runtime / "worker.lock").open("a+")
    fcntl.flock(lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    cancelled = threading.Event()
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda signum, frame: (stopping.set(), cancelled.set()))
    signal.signal(signal.SIGINT, lambda signum, frame: (stopping.set(), cancelled.set()))
    renderer = BlenderReplayRenderer()
    renderer.maximum_rss_bytes = 3 * 1024**3
    limits = OfflineReplayLimits.from_environment(os.environ)
    status = {"state": "idle", "renderer": renderer.availability()}

    def publish() -> None:
        write_json_atomic(runtime / "owner.json", {**status, "received_wall_time": time.time()})

    try:
        while not stopping.is_set():
            publish()
            request_path = runtime / "request.json"
            if not request_path.exists():
                stopping.wait(0.25)
                continue
            request = FileReplayRenderer.read(request_path)
            request_path.unlink()
            identifier, token = request.get("job_id"), request.get("token")
            if (
                not isinstance(identifier, str)
                or not re.fullmatch(r"job-[a-f0-9]{32}", identifier)
                or not isinstance(token, str)
                or not re.fullmatch(r"[a-f0-9]{32}", token)
            ):
                status = {
                    "state": "error",
                    "renderer": {"available": False, "reason": "Invalid render request identity"},
                }
                continue
            directory = jobs / identifier
            if (
                directory.is_symlink()
                or directory.resolve().parent != jobs.resolve()
                or not directory.is_dir()
            ):
                status = {
                    "state": "error",
                    "renderer": {
                        "available": False,
                        "reason": "Render export package is unavailable",
                    },
                }
                continue
            cancelled.clear()
            status = {"state": "running", "job_id": identifier, "renderer": renderer.availability()}
            result = {"state": "running", "token": token, "progress": 0.0}
            deadline = time.monotonic() + limits.maximum_wall_seconds

            def check() -> None:
                if cancelled.is_set() or stopping.is_set():
                    raise InterruptedError("Offline worker cancelled")
                lease = FileReplayRenderer.read(runtime / "lease.json")
                if (
                    lease.get("token") != token
                    or lease.get("job_id") != identifier
                    or not 0 <= time.time() - lease["received_wall_time"] < 3
                ):
                    raise InterruptedError(
                        "Offline API owner lease ended; capture evidence retained"
                    )
                cancellation = runtime / "cancel.json"
                if (
                    cancellation.exists()
                    and FileReplayRenderer.read(cancellation).get("token") == token
                ):
                    raise InterruptedError("Offline job cancellation acknowledged")
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f"Offline worker exceeded {limits.maximum_wall_seconds} wall seconds"
                    )
                size = directory_bytes(directory)
                if size > limits.maximum_output_bytes:
                    raise OSError("Offline render reached its configured output byte budget")
                ensure_capacity(directory, 65536)
                publish()

            def progress(value: float) -> None:
                result["progress"] = value
                write_json_atomic(directory / "renderer-status.json", result)

            try:
                check()
                renderer.render(directory, cancelled, check, progress)
                result.update(state="completed", progress=1.0)
            except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
                result.update(
                    state="cancelled" if isinstance(error, InterruptedError) else "error",
                    error=str(error),
                )
            write_json_atomic(directory / "renderer-status.json", result)
            status = {"state": "idle", "renderer": renderer.availability()}
        return 0
    finally:
        write_json_atomic(
            runtime / "owner.json",
            {
                "state": "stopped",
                "renderer": {"available": False, "reason": "Offline worker stopped"},
                "received_wall_time": time.time(),
            },
        )
        lease.close()


if __name__ == "__main__":
    raise SystemExit(main())
