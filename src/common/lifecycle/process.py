"""Supervise a subprocess and forward container shutdown to its process group."""

import json
import logging
import os
import re
import signal
import socket
import subprocess
import threading
import time
import uuid
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import TextIO

from common.lifecycle.children import descendant_process_ids, process_start_ticks
from common.logging.storage import BoundedRecording, write_json_atomic


def run_process(
    command: list[str],
    window_name: str | None = None,
    restart_file: Path | None = None,
    ready_callback: Callable[[], None] | None = None,
    log_source: str | None = None,
    own_process_group: bool = True,
    generation_file: Path | None = None,
    status_file: Path | None = None,
    after_child_shutdown: Callable[[], None] | None = None,
) -> int:
    """Run one service; optionally publish its X11 window ID for a separate viewer host."""
    initial_generation = (
        generation_file.read_text() if generation_file and generation_file.exists() else None
    )
    process = subprocess.Popen(
        command,
        start_new_session=own_process_group,
        stdout=subprocess.PIPE if log_source else None,
        stderr=subprocess.STDOUT if log_source else None,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    try:
        child_start_ticks = process_start_ticks(process.pid) if status_file or window_name else None
    except FileNotFoundError:
        child_start_ticks = None
    recorder = None
    process_generation = uuid.uuid4().hex
    last_status = 0.0

    def publish_status(running: bool, exit_code: int | None = None) -> None:
        if status_file is not None:
            write_json_atomic(
                status_file,
                {
                    "schema_version": 1,
                    "generation": process_generation,
                    "running": running,
                    "received_wall_time": time.time(),
                    "exit_code": exit_code,
                    "pid": process.pid,
                    "process_start_ticks": child_start_ticks,
                    "machine": socket.gethostname(),
                },
            )

    if log_source:
        recorder = threading.Thread(
            target=record_console, args=(process.stdout, log_source), daemon=True
        )
        recorder.start()
    stopping = False

    def signal_process(signum: int) -> None:
        try:
            if own_process_group:
                os.killpg(process.pid, signum)
            else:
                process.send_signal(signum)
        except ProcessLookupError:
            pass

    def stop(signum: int, frame: object) -> None:
        nonlocal stopping
        stopping = True
        if process.poll() is None:
            signal_process(signal.SIGTERM)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    window_file = Path("/runtime") / f"{window_name}.window" if window_name else None
    if window_file:
        window_file.unlink(missing_ok=True)
    window_owner_file = window_file.with_suffix(".owner.json") if window_file else None
    if window_owner_file:
        window_owner_file.unlink(missing_ok=True)
    restart_requested = False
    try:
        while process.poll() is None:
            if time.monotonic() - last_status >= 0.5:
                publish_status(True)
                last_status = time.monotonic()
            if restart_file and restart_file.exists():
                restart_file.unlink()
                restart_requested = True
                break
            if (
                not stopping
                and generation_file
                and generation_file.exists()
                and generation_file.read_text() != initial_generation
            ):
                logging.info("Process generation changed: %s", generation_file)
                restart_requested = True
                break
            if window_file and window_file.exists():
                result = query_native_window(
                    ["xdotool", "getwindowname", window_file.read_text().strip()],
                )
                if (
                    result is None
                    or result.returncode
                    or owned_native_window_pid(
                        window_file.read_text().strip(), process.pid, child_start_ticks
                    )
                    is None
                ):
                    window_file.unlink(missing_ok=True)
                    window_owner_file.unlink(missing_ok=True)
            if window_file and not window_file.exists():
                result = query_native_window(
                    ["xdotool", "search", "--any", "--name", "--class", window_name],
                )
                if result is not None and result.returncode == 0 and result.stdout.strip():
                    candidates = []
                    for identifier in result.stdout.splitlines():
                        native_pid = owned_native_window_pid(
                            identifier, process.pid, child_start_ticks
                        )
                        if native_pid is None:
                            continue
                        geometry = query_native_window(
                            ["xdotool", "getwindowgeometry", "--shell", identifier],
                        )
                        if geometry is None:
                            continue
                        dimensions = dict(re.findall(r"(WIDTH|HEIGHT)=(\d+)", geometry.stdout))
                        area = int(dimensions.get("WIDTH", 0)) * int(dimensions.get("HEIGHT", 0))
                        if area >= 30000:
                            candidates.append((area, identifier, native_pid))
                    if candidates:
                        if ready_callback:
                            ready_callback()
                        # Publication means startup work has finished; an attaching
                        # client must not race the owner's initial camera setup.
                        _, identifier, native_pid = max(candidates)
                        try:
                            window_owner = {
                                "schema_version": 1,
                                "window_id": int(identifier),
                                "owner_pid": os.getpid(),
                                "owner_start_ticks": process_start_ticks(os.getpid()),
                                "child_pid": process.pid,
                                "native_pid": native_pid,
                                "native_start_ticks": process_start_ticks(native_pid),
                                "machine": socket.gethostname(),
                                "published_wall_time": time.time(),
                            }
                        except FileNotFoundError:
                            window_owner = None
                        if window_owner is not None:
                            write_json_atomic(window_owner_file, window_owner)
                            window_file.write_text(identifier, encoding="utf-8")
            if stopping:
                try:
                    process.wait(timeout=10)
                    break
                except subprocess.TimeoutExpired:
                    signal_process(signal.SIGKILL)
            time.sleep(0.25)
        exit_code = 75 if restart_requested else (0 if stopping else process.returncode)
    finally:
        if window_file:
            window_file.unlink(missing_ok=True)
            window_owner_file.unlink(missing_ok=True)
        if process.poll() is None:
            signal_process(signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                signal_process(signal.SIGKILL)
                process.wait()
        if after_child_shutdown:
            after_child_shutdown()
        if recorder:
            recorder.join(timeout=2)
        if process.stdout:
            process.stdout.close()
        publish_status(False, process.returncode)
    return 0 if stopping else exit_code


def owned_native_window_pid(
    identifier: str, child_pid: int, expected_start_ticks: int | None = None
) -> int | None:
    """Require both this child tree and its X11 client machine, not a matching title."""
    try:
        if (
            expected_start_ticks is not None
            and process_start_ticks(child_pid) != expected_start_ticks
        ):
            return None
    except FileNotFoundError:
        return None
    pid_result = query_native_window(["xdotool", "getwindowpid", identifier])
    machine_result = query_native_window(["xprop", "-id", identifier, "WM_CLIENT_MACHINE"])
    if (
        pid_result is None
        or machine_result is None
        or pid_result.returncode
        or machine_result.returncode
    ):
        return None
    try:
        native_pid = int(pid_result.stdout.strip())
    except ValueError:
        return None
    machine = re.search(r'=\s*"([^"]+)"', machine_result.stdout)
    if machine is None or machine.group(1) != socket.gethostname():
        return None
    if native_pid not in {child_pid, *descendant_process_ids(child_pid)}:
        return None
    return native_pid


def query_native_window(arguments: list[str]) -> subprocess.CompletedProcess | None:
    """Treat display-server timeouts as unavailable observations, not supervisor crashes."""
    try:
        return subprocess.run(arguments, capture_output=True, text=True, timeout=5, check=False)
    except subprocess.TimeoutExpired:
        logging.warning("Native display query timed out: %s", arguments[1])
        return None


def record_console(stream: TextIO, source: str) -> None:
    """Persist console lines while preserving normal container log output."""
    directory = Path(os.environ.get("OUTPUT_DIR", "outputs")) / "logs"
    with BoundedRecording(directory / f"{source}.jsonl") as recording:
        while line := stream.readline(65536):
            text = re.sub(r"\x1b\[[0-9;]*m", "", line.rstrip())
            print(text, flush=True)
            recording.write(
                json.dumps(
                    {
                        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                        "source": source,
                        "vehicle_id": os.environ.get("UAV_VEHICLE_ID"),
                        "message": text,
                    }
                )
                + "\n"
            )
