"""Run native SITL checks and keep a per-check evidence ledger."""

import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

from common.logging.storage import write_json_atomic
from testing.simulation.navigation_acceptance import fleet, request, wait_ready

harness = Path(__file__).resolve().parent
run = Path(os.environ["UAV_CAMPAIGN_ROOT"]).resolve()
original = Path(os.environ["UAV_REGULAR_ROOT"]).resolve()
if (run / "campaign.json").exists():
    raise RuntimeError("Choose a new campaign directory; existing ledgers are immutable")
visual_port = int(os.environ["UAV_CAMPAIGN_VISUAL_PORT"])
regular_port = int(os.environ["UAV_REGULAR_PORT"])
reports = Path("outputs/reports")
port = os.environ["BACKEND_PORT"]
original_env = dict(os.environ)
for key in (
    "COMPOSE_PROJECT_NAME",
    "COMPOSE_FILE",
    "COMPOSE_ENV_FILES",
    "BACKEND_PORT",
    "OUTPUTS_PATH",
    "XAUTHORITY_PATH",
    "UAV_RECORD_LIMIT_PAIRS",
    "ROS_DOMAIN_ID",
):
    original_env.pop(key, None)
for key in (
    "UAV_CAMPAIGN_ROOT",
    "UAV_REGULAR_ROOT",
    "UAV_CAMPAIGN_VISUAL_PORT",
    "UAV_REGULAR_PORT",
    "UAV_MAPPING_INPUT_ARCHIVE_PAIRS",
    "UAV_NAV_STEREO_ARCHIVE_BATCHES",
    "UAV_MAPPING_DIAGNOSTICS",
):
    original_env.pop(key, None)
ledger = {
    "schema_version": 1,
    "state": "running",
    "revision": (run / "final-revision.txt").read_text().strip(),
    "harness_revision": (run / "final-revision.txt").read_text().strip(),
    "application_revision": (run / "application-revision.txt").read_text().strip(),
    "clean_first_attempt": False,
    "source_worktree": str(Path.cwd()),
    "resolved_configuration": str(run / "platform-resolved.json"),
    "image_inventory": str(run / "final-images.json"),
    "visual_configuration": str(run / "visual-resolved.json"),
    "storage_preflight": str(run / "final-platform-storage-preflight.log"),
    "build_commands": str(run / "build-commands.log"),
    "component_logs": [
        str(run / name) for name in ("final-runtime-tests.log", "final-ros-tests.log")
    ],
    "started_wall_time": time.time(),
    "checks": {},
    "cleanup": {},
}


def save() -> None:
    states = [item["state"] for item in ledger["checks"].values()]
    ledger["totals"] = {
        state: states.count(state) for state in ("passed", "failed", "not_run", "running")
    }
    write_json_atomic(run / "campaign.json", ledger)


def launch(name: str, command: list[str], timeout: float, **kwargs: object) -> int:
    target = run / f"{name}.log"
    with (run / "execution-commands.jsonl").open("a") as commands:
        commands.write(
            json.dumps({"name": name, "command": command, "wall_time": time.time()}) + "\n"
        )
    with target.open("w") as log:
        return subprocess.run(
            command, timeout=timeout, stdout=log, stderr=subprocess.STDOUT, **kwargs
        ).returncode


def check(name: str, command: list[str], timeout: float, **kwargs: object) -> None:
    row = ledger["checks"][name]
    row.update(state="running", started_wall_time=time.time(), command=command)
    save()
    evidence_root = (run / "visual-recordings") if name.startswith("visual-") else reports
    before = (
        {path for path in evidence_root.rglob("*.json")}
        if name.startswith("visual-")
        else {path for path in reports.glob("*.json")}
    )
    try:
        code = launch(name, command, timeout, **kwargs)
        row["exit_code"] = code
        if code:
            raise RuntimeError(f"{name} exited {code}. See {name}.log")
        row["state"] = "passed"
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        row.update(state="failed", error=str(error))
        raise
    finally:
        after = set(
            evidence_root.rglob("*.json")
            if name.startswith("visual-")
            else evidence_root.glob("*.json")
        )
        row["evidence"] = [str(path.resolve()) for path in sorted(after - before)]
        row["log"] = str(run / f"{name}.log")
        row["finished_wall_time"] = time.time()
        save()


def compose(*args: str) -> list[str]:
    return ["docker", "compose", *args]


def script(name: str, *args: str) -> list[str]:
    return [sys.executable, f"scripts/validation/{name}.py", *args]


def original_fleet() -> list[dict]:
    with urllib.request.urlopen(f"http://127.0.0.1:{regular_port}/vehicles", timeout=4) as response:
        return json.load(response)["items"]


def vehicle_evidence(vehicles: list[dict]) -> list[dict]:
    fields = ("vehicle_id", "connected", "armed", "ready", "run_id")
    return [{field: vehicle.get(field) for field in fields} for vehicle in vehicles]


def confirmed_disarmed(vehicles: list[dict]) -> bool:
    return bool(vehicles) and all(
        vehicle["connected"] and vehicle["armed"] is False for vehicle in vehicles
    )


def space_apps() -> int:
    with urllib.request.urlopen("http://127.0.0.1:8090", timeout=5) as response:
        return response.status


route = ["--port", port, "--east-offset-m", "2.5", "--north-offset-m", "-8.5"]
select = [
    sys.executable,
    "-c",
    "from testing.simulation.navigation_acceptance import select_scene; "
    "select_scene('stereo/single-01')",
]


def observed(label: str, vehicle_id: str, strict: str, *options: str) -> list[str]:
    return [
        sys.executable,
        str(harness / "run-observed-waypoint.py"),
        label,
        vehicle_id,
        strict,
        *route,
        *options,
    ]


tasks = [
    ("live-camera-qualification", script("cameras"), 125),
    ("select-waypoint-scene", select, 150),
    (
        "bounded-observation-qualification",
        [sys.executable, str(harness / "check-platform-observation.py")],
        110,
    ),
    ("camera-presets-follow", script("camera"), 145),
    ("normal-waypoint", observed("normal-waypoint", "vehicle_1", "true", "--viewer-follow"), 345),
    ("native-integration", script("integration", "--port", port), 160),
    (
        "backend-restart-waypoint",
        observed(
            "backend-restart-waypoint",
            "vehicle_1",
            "true",
            "--restart-control-in-flight",
            "--viewer-follow",
        ),
        345,
    ),
    (
        "pause-stop-reset",
        observed(
            "pause-stop-reset",
            "vehicle_2",
            "false",
            "--stop-early",
            "--pause-in-flight",
            "--reset-from-pause",
        ),
        345,
    ),
    (
        "concurrent-missions",
        script("fleet", "--fly", "--east-offset-m", "2.5", "--north-offset-m", "-8.5"),
        340,
    ),
    ("panels", script("panels", "--port", port), 140),
    (
        "desktop-close-reattach",
        script(
            "viewer_restart",
            "--service",
            "desktop",
            "--verify-close",
            "--unique-report",
            "--port",
            port,
        ),
        150,
    ),
    ("dds-restart", script("dds_recovery", "--port", port), 150),
    ("vehicle-catalog", script("vehicle_catalog"), 180),
    ("lifecycle-cycles", script("lifecycle_cycles", "--port", port, "--cycles", "10"), 720),
    ("camera-stream-restart", script("cameras", "--restart"), 160),
]
cases = json.loads(Path("configs/navigation/stereo/reliability.json").read_text())["nominal_cases"]
for case in cases:
    for mode in ("camera", "fused"):
        tasks.append(
            (
                f"{case['name']}-{mode}",
                [
                    sys.executable,
                    "-m",
                    "testing.simulation.stereo_rtf_benchmark",
                    "--factor",
                    "1.0",
                    "--case",
                    case["name"],
                    "--mode",
                    mode,
                ],
                560,
            )
        )
tasks.append(
    (
        "reciprocal-range-recovery",
        [sys.executable, "-m", "testing.simulation.stereo_postcheck"],
        1600,
    )
)

for name, command, timeout in tasks:
    ledger["checks"].setdefault(
        name, {"state": "not_run", "command": command, "timeout_seconds": timeout}
    )
for name in (
    "visual-short",
    "visual-gps-off",
    "visual-gps-loss",
    "visual-vision-loss",
    "visual-reset-gps-regression",
    "visual-async-recording-faults",
):
    ledger["checks"].setdefault(name, {"state": "not_run"})
save()

original_stopped = False
candidate_started = False
visual_started = False


def visual(*command: str) -> list[str]:
    return [
        "bash",
        "-c",
        'source "$1"; shift; exec "$@"',
        "visual",
        str(run / "visual.env.sh"),
        *command,
    ]


def clean_platform() -> bool:
    initial = list(fleet().values())
    for vehicle in initial:
        if vehicle["armed"]:
            request("stop", {"vehicle_id": vehicle["vehicle_id"]})
    deadline = time.monotonic() + 70
    while time.monotonic() < deadline:
        current = list(fleet().values())
        if confirmed_disarmed(current):
            break
        time.sleep(1)
    cleanup = ledger["cleanup"]
    cleanup["platform_fleet"] = vehicle_evidence(current)
    cleanup["platform_disarm_verified"] = confirmed_disarmed(current)
    launch("platform-service-inventory", compose("ps", "-a"), 20)
    launch("platform-service-logs", compose("logs", "--no-color", "--tail", "350"), 40)
    if cleanup["platform_disarm_verified"]:
        cleanup["platform_stopped"] = launch("stop-isolated-platform", compose("stop"), 90) == 0
    return cleanup.get("platform_stopped", False)


try:
    for artifact in ("final-images.json", "platform-resolved.json"):
        if not (run / artifact).is_file():
            raise RuntimeError(f"Missing build or configuration evidence {artifact}")
    if launch(
        "final-platform-storage-preflight", script("storage_preflight", "--generations", "40"), 30
    ):
        raise RuntimeError("Post-build campaign storage preflight failed")
    free = os.statvfs(run)
    admission = json.loads(
        Path(os.environ["OUTPUTS_PATH"], "reports/storage-preflight-current.json").read_text()
    )
    required = admission["required_total_free_bytes"] + 160 * 2 * (640 * 480 * 3 + 64)
    if free.f_bavail * free.f_frsize < required:
        raise RuntimeError(
            "Remaining disk space is below the combined platform and visual clip budget"
        )
    before = original_fleet()
    if not confirmed_disarmed(before):
        raise RuntimeError(
            "Existing fleet must be connected and disarmed before native GUI handoff"
        )
    ledger["handoff"] = {
        "original_fleet": vehicle_evidence(before),
        "space_apps_http_status": space_apps(),
    }
    save()
    original_stopped = True
    if launch("stop-existing-for-native-gui", compose("stop"), 90, cwd=original, env=original_env):
        raise RuntimeError("Existing deployment could not be stopped cleanly")
    candidate_started = True
    if launch("start-isolated-platform", compose("up", "-d", "--no-build"), 220):
        raise RuntimeError("Isolated platform did not start")
    wait_ready()
    if len(fleet()) != 2 or not confirmed_disarmed(list(fleet().values())):
        raise RuntimeError("Isolated platform did not expose two connected, disarmed vehicles")
    for name, command, timeout in tasks:
        if name == "dds-restart":
            readiness = run / "dds-freshness-precondition.json"
            deadline = time.monotonic() + 75
            started = time.time()
            rows = []
            while time.monotonic() < deadline and len(rows) < 3:
                try:
                    output = subprocess.check_output(
                        compose("exec", "-T", "ros2-monitor", "cat", "/runtime/ros2.json"),
                        text=True,
                        stderr=subprocess.DEVNULL,
                        timeout=4,
                    )
                    sample = json.loads(output)
                    if (
                        sample["xy_valid"]
                        and sample["z_valid"]
                        and sample["received_wall_time"] >= started
                        and 0 <= time.time() - sample["received_wall_time"] < 3
                        and (not rows or sample["timestamp_us"] > rows[-1]["timestamp_us"])
                        and request("capabilities")["ros_connected"]
                    ):
                        rows.append(sample)
                except (OSError, ValueError, KeyError, subprocess.SubprocessError):
                    pass
                time.sleep(0.5)
            write_json_atomic(readiness, {"passed": len(rows) == 3, "fresh_samples": rows})
            ledger["checks"][name]["ros_freshness_evidence"] = str(readiness)
            save()
            if len(rows) < 3:
                raise RuntimeError("DDS precondition remained unavailable for 75 seconds")
        check(name, command, timeout)
    if not clean_platform():
        raise RuntimeError("Platform disarm or shutdown verification failed")
    candidate_started = False
    save()
    visual_started = True
    if launch(
        "start-isolated-visual",
        visual(*compose("up", "-d", "--no-build", "control-panel", "navigation-stereo")),
        220,
    ):
        raise RuntimeError("Isolated visual profile did not start")
    deadline = time.monotonic() + 90
    readiness = {}
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(
                f"http://127.0.0.1:{visual_port}/vehicles", timeout=3
            ) as response:
                rows = json.load(response)["items"]
            readiness["fleet"] = vehicle_evidence(rows)
            output = subprocess.check_output(
                visual(str(harness / "visual-px4"), "listener", "vehicle_status", "-n", "1"),
                text=True,
                timeout=8,
                stderr=subprocess.DEVNULL,
            )
            readiness["status"] = output
            if (
                len(rows) == 1
                and rows[0]["ready"]
                and rows[0]["connected"]
                and rows[0]["armed"] is False
                and "arming_state: 1" in output
                and "pre_flight_checks_pass: True" in output
            ):
                break
        except (OSError, ValueError, subprocess.SubprocessError):
            pass
        time.sleep(0.5)
    else:
        write_json_atomic(run / "visual-startup-readiness.json", readiness)
        raise RuntimeError("Visual PX4 startup did not become ready and disarmed within 90 seconds")
    readiness["passed"] = True
    write_json_atomic(run / "visual-startup-readiness.json", readiness)
    for case in ("short", "gps-off", "gps-loss", "vision-loss"):
        check(f"visual-{case}", visual(str(harness / "visual-run-case"), case), 440)
    check("visual-reset-gps-regression", visual(str(harness / "visual-run-reset-regression")), 340)
    check("visual-async-recording-faults", visual(str(harness / "visual-run-recording")), 240)
    ledger["state"] = "complete"
except (OSError, RuntimeError, ValueError, KeyError, subprocess.SubprocessError) as error:
    ledger.update(state="failed", error=str(error))
finally:
    cleanup = ledger["cleanup"]
    if candidate_started:
        try:
            clean_platform()
        except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
            cleanup["platform_disarm_error"] = str(error)
    if visual_started:
        try:
            px4 = str(harness / "visual-px4")
            disarm_file = run / "visual-final-disarm.log"
            code = launch(
                "visual-final-disarm", visual(px4, "listener", "vehicle_status", "-n", "1"), 20
            )
            if code or "arming_state: 1" not in disarm_file.read_text():
                with urllib.request.urlopen(
                    urllib.request.Request(
                        f"http://127.0.0.1:{visual_port}/stop",
                        data=b'{"vehicle_id":"vehicle_1"}',
                        headers={"Content-Type": "application/json"},
                    ),
                    timeout=15,
                ) as response:
                    cleanup["visual_landing_request_status"] = response.status
                for attempt in range(20):
                    time.sleep(2)
                    code = launch(
                        "visual-final-disarm",
                        visual(px4, "listener", "vehicle_status", "-n", "1"),
                        20,
                    )
                    if code == 0 and "arming_state: 1" in disarm_file.read_text():
                        break
            cleanup["visual_disarm_verified"] = (
                code == 0 and "arming_state: 1" in disarm_file.read_text()
            )
            launch(
                "visual-service-logs", visual(*compose("logs", "--no-color", "--tail", "300")), 40
            )
            if cleanup["visual_disarm_verified"]:
                cleanup["visual_stopped"] = (
                    launch("stop-isolated-visual", visual(*compose("stop")), 90) == 0
                )
        except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
            cleanup["visual_disarm_error"] = str(error)
    if original_stopped:
        cleanup["original_restarted"] = (
            launch(
                "restore-existing-deployment", compose("start"), 120, cwd=original, env=original_env
            )
            == 0
        )
        try:
            deadline = time.monotonic() + 100
            while time.monotonic() < deadline:
                try:
                    current = original_fleet()
                    if confirmed_disarmed(current):
                        cleanup["original_fleet_restored"] = vehicle_evidence(current)
                        break
                except (OSError, ValueError):
                    pass
                time.sleep(2)
        except (OSError, ValueError) as error:
            cleanup["original_restore_error"] = str(error)
    try:
        cleanup["space_apps_http_status"] = space_apps()
    except OSError as error:
        cleanup["space_apps_error"] = str(error)
    ledger["finished_wall_time"] = time.time()
    if ledger["state"] == "complete" and not all(
        cleanup.get(field)
        for field in (
            "platform_disarm_verified",
            "platform_stopped",
            "visual_disarm_verified",
            "visual_stopped",
            "original_restarted",
            "original_fleet_restored",
        )
    ):
        ledger.update(
            state="failed", error="Cleanup or original deployment restoration was unverified"
        )
    ledger["passed"] = ledger["state"] == "complete"
    ledger["clean_first_attempt"] = (
        ledger["passed"]
        and len(ledger["checks"]) == 42
        and all(item["state"] == "passed" for item in ledger["checks"].values())
    )
    save()
sys.exit(0 if ledger["state"] == "complete" else 1)
