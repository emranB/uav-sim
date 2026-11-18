"""Run the fixed four-scenario, two-vehicle saved-mission comparison without retries."""

import argparse
import hashlib
import json
import math
import os
import subprocess
import time
import urllib.request
from pathlib import Path

from common.configuration.fleet import load_fleet
from common.configuration.mission_store import MissionStore
from common.logging.storage import ensure_capacity, write_json_atomic

SCENARIOS = (
    ("both-gps-on", ("gps-on", "gps-on")),
    ("mixed-gps", ("gps-on", "gps-off")),
    ("both-gps-off", ("gps-off", "gps-off")),
    ("waypoint-gps-loss", ("gps-loss", "gps-on")),
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", type=int, default=18083)
    parser.add_argument(
        "--mission", type=Path, default=Path("configs/missions/visual-comparison.plan")
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    ledger_path = args.output / "comparison.json"
    if ledger_path.exists():
        parser.error("Use a new comparison evidence directory; failed runs cannot be resumed")
    recordings = Path(os.environ["OUTPUTS_PATH"])
    presentation = recordings / "reports/presentation"
    presentation.mkdir(parents=True, exist_ok=True)
    fleet = tuple(v for v in load_fleet(recordings / "fleet.json") if v.enabled)
    if tuple(v.vehicle_id for v in fleet) != ("vehicle_1", "vehicle_2"):
        parser.error("This fixed matrix requires explicit Atlas and Boreal definitions")
    plan = json.loads(args.mission.read_text())
    mission_digest = hashlib.sha256(args.mission.read_bytes()).hexdigest()
    scene = Path("configs/navigation/stereo/visual-odometry-probe.json")
    ledger = {
        "schema_version": 1,
        "state": "running",
        "passed": False,
        "mission_id": "visual-comparison",
        "mission_sha256": mission_digest,
        "scene_id": scene.stem,
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "started_wall_time": time.time(),
        "scenarios": {name: {"state": "unrun"} for name, _ in SCENARIOS},
        "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "clean_first_attempt": False,
    }
    monitors = []
    publishers = []
    publisher_processes = []
    handles = []
    active_capture_id = None
    process_log = args.output / "execution-commands.jsonl"

    def save() -> None:
        write_json_atomic(ledger_path, ledger)

    def request(route: str, body: dict | None = None) -> dict:
        operation = urllib.request.Request(
            f"http://127.0.0.1:{args.port}/{route}",
            data=json.dumps(body).encode() if body is not None else None,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(operation, timeout=35 if body else 4) as response:
            return json.load(response)

    def launch(name: str, command: list[str]) -> subprocess.Popen:
        with process_log.open("a") as log:
            log.write(
                json.dumps({"name": name, "command": command, "wall_time": time.time()}) + "\n"
            )
        handle = (args.output / f"{name}.log").open("w")
        handles.append(handle)
        return subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT)

    def stop_publishers() -> None:
        for name in publishers:
            subprocess.run(
                ["docker", "stop", "--time", "8", name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=15,
            )
        publishers.clear()
        for process in publisher_processes:
            process.wait(timeout=15)
        publisher_processes.clear()

    def native_ready() -> dict:
        deadline = time.monotonic() + 90
        latest = {}
        while time.monotonic() < deadline:
            rows = request("vehicles")["items"]
            latest = {
                "fleet": [
                    {
                        key: row.get(key)
                        for key in (
                            "vehicle_id",
                            "connected",
                            "armed",
                            "ready",
                            "run_id",
                            "recording",
                        )
                    }
                    for row in rows
                ],
                "native": {},
            }
            if len(rows) == 2 and all(
                r["connected"] and r["armed"] is False and r["ready"] for r in rows
            ):
                for vehicle, service in zip(fleet, ("px4", "px4-second")):
                    latest["native"][vehicle.vehicle_id] = subprocess.check_output(
                        [
                            "docker",
                            "compose",
                            "exec",
                            "-T",
                            service,
                            "/opt/px4/build/px4_sitl_default/bin/px4-listener",
                            "--instance",
                            str(vehicle.instance),
                            "vehicle_status",
                            "-n",
                            "1",
                        ],
                        text=True,
                        timeout=5,
                    )
                if all(
                    "arming_state: 1" in s and "pre_flight_checks_pass: True" in s
                    for s in latest["native"].values()
                ):
                    return {"passed": True, **latest}
            time.sleep(0.5)
        raise RuntimeError(
            "Both native controllers did not become ready and disarmed within 90 seconds"
        )

    def qualify(cases: tuple[str, str]) -> dict:
        deadline = time.monotonic() + 75
        counts = {vehicle.vehicle_id: 0 for vehicle in fleet}
        history = []
        while time.monotonic() < deadline:
            rows = request("navigation/fleet")["items"]
            by_identity = {row["vehicle_id"]: row for row in rows}
            for vehicle, case in zip(fleet, cases):
                identity = vehicle.vehicle_id
                row = by_identity[identity]
                source = row["navigation"].get("visual_localization") or {}
                state, pose = source.get("aiding", {}), source.get("pose", {})
                now = time.monotonic()
                gps = (
                    (state.get("gps_fix_type", -1) >= 3 and state.get("gnss_xy_active"))
                    if case in {"gps-on", "gps-loss"}
                    else (state.get("gps_fix_type") == 0 and not state.get("gnss_xy_active"))
                )
                passed = (
                    state.get("vehicle_id") == pose.get("vehicle_id") == identity
                    and state.get("controller_run_id") == pose.get("controller_run_id")
                    and 0 <= now - state.get("monotonic", 0) <= 0.5
                    and 0 <= now - pose.get("monotonic", 0) <= 0.25
                    and state.get("vision_xy_active")
                    and state.get("vision_height_active")
                    and state.get("capture_stamp_valid")
                    and state.get("source_samples_fresh")
                    and state.get("arming_state") == 1
                    and pose.get("healthy")
                    and gps
                )
                counts[identity] = counts[identity] + 1 if passed else 0
            history.append({"monotonic": time.monotonic(), "items": rows})
            if all(count >= 8 for count in counts.values()):
                return {"passed": True, "consecutive_samples": counts, "history": history}
            time.sleep(0.25)
        raise RuntimeError("Both synchronized visual sources did not qualify within 75 seconds")

    save()
    try:
        # Eight finite two-vehicle-equivalent allowances include initial/reset
        # startup generations as well as the four scored missions.
        admission = ensure_capacity(recordings, 8 * (528 * 1024**2 + 4 * 1024**2))
        write_json_atomic(args.output / "storage-admission.json", admission)
        for index, (name, cases) in enumerate(SCENARIOS):
            entry = ledger["scenarios"][name]
            entry.update(
                state="running",
                started_wall_time=time.time(),
                cases=dict(zip((v.vehicle_id for v in fleet), cases)),
            )
            save()
            stop_publishers()
            current = request("vehicles")["items"]
            if not all(row["connected"] and row["armed"] is False for row in current):
                raise RuntimeError("Reset requires both connected, disarmed vehicles")
            for vehicle in fleet:
                request("reset", {"vehicle_id": vehicle.vehicle_id})
            for monitor in monitors:
                monitor.wait(timeout=15)
            monitors.clear()
            readiness = native_ready()
            write_json_atomic(args.output / f"{name}-native-readiness.json", readiness)
            write_json_atomic(
                presentation / "presentation-context.json",
                {
                    "kind": "context",
                    "case": name,
                    "request_id": f"context-{name}",
                },
            )
            for vehicle, case, service in zip(fleet, cases, ("px4", "px4-second")):
                store = MissionStore(vehicle.output_directory(recordings))
                if (
                    not store.path("visual-comparison").exists()
                    or store.load_with_digest("visual-comparison")[1] != mission_digest
                ):
                    store.save("visual-comparison", plan)
                _, actual_digest = store.load_with_digest("visual-comparison")
                if actual_digest != mission_digest:
                    raise RuntimeError(
                        "Saved mission bytes differ from the frozen canonical mission"
                    )
                target = f"/workspace/outputs/presentation/{name}/{vehicle.vehicle_id}"
                monitors.append(
                    launch(
                        f"{name}-{vehicle.vehicle_id}-aiding",
                        [
                            "docker",
                            "compose",
                            "exec",
                            "-T",
                            service,
                            "python",
                            "-m",
                            "testing.simulation.visual_aiding_status",
                            "--vehicle",
                            vehicle.vehicle_id,
                            "--configure-gps",
                            "enabled" if case in {"gps-on", "gps-loss"} else "disabled",
                            "--output",
                            target + "-aiding.jsonl",
                        ],
                    )
                )
                publisher = f"{os.environ['COMPOSE_PROJECT_NAME']}-{name}-{vehicle.vehicle_id}"
                publishers.append(publisher)
                publisher_processes.append(
                    launch(
                        f"{name}-{vehicle.vehicle_id}-publisher",
                        [
                            "docker",
                            "compose",
                            "--profile",
                            "visual-odometry",
                            "run",
                            "--rm",
                            "--no-deps",
                            "--name",
                            publisher,
                            "visual-odometry-worker",
                            "--vehicle",
                            vehicle.vehicle_id,
                            "--flight-publisher",
                            "--record-poses",
                            "--output",
                            target + "-pose",
                        ],
                    )
                )
            write_json_atomic(args.output / f"{name}-qualification.json", qualify(cases))
            deadline = time.monotonic() + 25
            while not (presentation / f"{name}-preflight.png").exists():
                if time.monotonic() > deadline:
                    raise RuntimeError("Native presentation preflight capture did not arrive")
                time.sleep(0.25)
            capture = request("replay/start", {"target_ids": [v.vehicle_id for v in fleet]})
            if capture.get("state") != "capturing":
                raise RuntimeError("Explicit comparison capture writer was not acknowledged")
            active_capture_id = capture["capture_id"]
            entry["replay_capture_id"] = active_capture_id
            save()
            observers = []
            for vehicle, case in zip(fleet, cases):
                command = [
                    "docker",
                    "compose",
                    "exec",
                    "-T",
                    "control-panel",
                    "python",
                    "-m",
                    "testing.simulation.visual_navigation_acceptance",
                    "--vehicle",
                    vehicle.vehicle_id,
                    "--case",
                    case,
                    "--mission-name",
                    "visual-comparison",
                    "--mission-sha256",
                    mission_digest,
                    "--output",
                    f"/workspace/outputs/presentation/{name}/{vehicle.vehicle_id}.json",
                ]
                if index == 0:
                    command.append("--external-start")
                observers.append(launch(f"{name}-{vehicle.vehicle_id}-flight", command))
            if index == 0:
                deadline = time.monotonic() + 15
                while not all(
                    (recordings / "presentation" / name / f"{v.vehicle_id}.ready.json").exists()
                    for v in fleet
                ):
                    if time.monotonic() > deadline:
                        raise RuntimeError("Both saved-mission observers did not become ready")
                    time.sleep(0.1)
                write_json_atomic(
                    presentation / "quick-action-request.json",
                    {
                        "request_id": "start-both",
                        "action": "start",
                        "target_ids": [v.vehicle_id for v in fleet],
                    },
                )
            samples = []
            land_requested = False
            deadline = time.monotonic() + 400
            with (args.output / f"{name}-fleet.jsonl").open("x") as journal:
                while any(process.poll() is None for process in observers):
                    if time.monotonic() > deadline:
                        raise TimeoutError("Comparison exceeded its unchanged route/landing budget")
                    rows = request("navigation/fleet")["items"]
                    scene_state = request("scene?vehicle_id=vehicle_1")
                    poses = {p["vehicle_id"]: p["position_enu_m"] for p in scene_state["poses"]}
                    sample = {
                        "monotonic": time.monotonic(),
                        "items": rows,
                        "scene_stale": scene_state["stale"],
                        "truth": poses,
                    }
                    journal.write(json.dumps(sample) + "\n")
                    samples.append(sample)
                    target, future = (
                        ("vehicle_2", "vehicle_1") if index == 0 else ("vehicle_1", "vehicle_2")
                    )
                    if (
                        index < 2
                        and not land_requested
                        and any(
                            row["vehicle_id"] == target
                            and row["armed"]
                            and row["navigation"].get("state") == "landing"
                            for row in rows
                        )
                    ):
                        write_json_atomic(
                            presentation / "quick-action-request.json",
                            {
                                "request_id": f"land-{target.replace('_', '-')}",
                                "action": "land",
                                "target_ids": [target],
                                "change_selection_after_dispatch": [future],
                            },
                        )
                        land_requested = True
                    time.sleep(0.5)
            stopped_capture = request("replay/stop", {})
            capture_deadline = time.monotonic() + 10
            while stopped_capture["state"] == "finalizing":
                if time.monotonic() > capture_deadline:
                    raise RuntimeError("Explicit comparison capture did not finalize")
                time.sleep(0.1)
                stopped_capture = request("replay")["capture"]
            if stopped_capture["state"] != "stopped":
                raise RuntimeError("Comparison capture is incomplete; retain its failure evidence")
            request("replay/keep", {"capture_id": active_capture_id})
            entry["replay_capture_final"] = stopped_capture
            active_capture_id = None
            save()
            reports = {
                v.vehicle_id: json.loads(
                    (recordings / "presentation" / name / f"{v.vehicle_id}.json").read_text()
                )
                for v in fleet
            }
            if index < 2:
                if not land_requested:
                    raise RuntimeError(f"{name}: selected vehicle's native Land was not exercised")
                acknowledgement_path = presentation / f"land-{target.replace('_', '-')}-ack.json"
                acknowledgement_deadline = time.monotonic() + 12
                while not acknowledgement_path.exists():
                    if time.monotonic() > acknowledgement_deadline:
                        raise RuntimeError(f"{name}: native Land acknowledgement did not arrive")
                    time.sleep(0.1)
                acknowledgement = json.loads(acknowledgement_path.read_text())
                if (
                    not acknowledgement.get("accepted")
                    or set(acknowledgement.get("results", {})) != {target}
                    or acknowledgement.get("visible_selection_after_dispatch") != [future]
                ):
                    raise RuntimeError(f"{name}: native Land target/selection isolation failed")
            overlap = [
                r
                for r in samples
                if not r["scene_stale"]
                and all(item["armed"] and item["navigation"].get("active") for item in r["items"])
            ]
            separation = min(
                (math.dist(r["truth"]["vehicle_1"], r["truth"]["vehicle_2"]) for r in overlap),
                default=0,
            )
            passed = (
                all(process.returncode == 0 for process in observers)
                and all(
                    report["passed"]
                    and report["saved_mission"]["sha256"] == mission_digest
                    and report["scenario_sha256"] == ledger["scene_sha256"]
                    for report in reports.values()
                )
                and bool(overlap)
                and separation >= 2
            )
            entry.update(
                state="passed" if passed else "failed",
                passed=passed,
                overlap_samples=len(overlap),
                minimum_vehicle_separation_m=separation,
                armed_overlap_wall_seconds=overlap[-1]["monotonic"] - overlap[0]["monotonic"]
                if overlap
                else 0,
                finished_wall_time=time.time(),
                reports={
                    identity: str(recordings / "presentation" / name / f"{identity}.json")
                    for identity in reports
                },
            )
            save()
            if not passed:
                raise RuntimeError(f"{name}: saved-mission comparison failed; preserve reports")
            required_captures = ["preflight", "outbound", "landed"]
            if name == "waypoint-gps-loss":
                required_captures.extend(["gps-transition", "camera-after-gps-loss"])
            deadline = time.monotonic() + 12
            while not all(
                (presentation / f"{name}-{phase}.json").exists() for phase in required_captures
            ):
                if time.monotonic() > deadline:
                    raise RuntimeError(f"{name}: required native presentation captures are missing")
                time.sleep(0.25)
            for phase in required_captures:
                capture = json.loads((presentation / f"{name}-{phase}.json").read_text())
                geometry = capture["geometry"]
                if geometry["quick_row_top_y"] < geometry["qgc_bottom_y"] or not all(
                    geometry["native_viewers"].values()
                ):
                    raise RuntimeError(
                        f"{name}: quick row placement or native embedding unverified"
                    )
            entry["presentation_passed"] = True
            save()
            stop_publishers()
            for vehicle in fleet:
                pose_report = json.loads(
                    (
                        recordings
                        / "presentation"
                        / name
                        / f"{vehicle.vehicle_id}-pose/report.json"
                    ).read_text()
                )
                if not pose_report["passed"]:
                    raise RuntimeError(f"{name}/{vehicle.vehicle_id}: pose publisher report failed")
        for identifier in ("start-both", "land-vehicle-1", "land-vehicle-2"):
            acknowledgement = json.loads((presentation / f"{identifier}-ack.json").read_text())
            if not acknowledgement["accepted"]:
                raise RuntimeError(f"{identifier}: widget command was not acknowledged")
        ledger.update(state="passed", passed=True, clean_first_attempt=True)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        ledger.update(state="failed", passed=False, clean_first_attempt=False, error=str(error))
    finally:
        if active_capture_id is not None:
            try:
                request("replay/stop", {})
                request("replay/keep", {"capture_id": active_capture_id})
            except OSError as error:
                ledger["capture_recovery_error"] = str(error)
        if not ledger["passed"]:
            recovery_errors = []
            for vehicle in fleet:
                try:
                    request("stop", {"vehicle_id": vehicle.vehicle_id})
                except OSError as error:
                    recovery_errors.append(f"{vehicle.vehicle_id}: {error}")
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                try:
                    rows = request("vehicles")["items"]
                    if len(rows) == 2 and all(
                        row["connected"] and row["armed"] is False for row in rows
                    ):
                        break
                except OSError:
                    pass
                time.sleep(0.5)
            else:
                recovery_errors.append("Both disarmed states could not be confirmed")
            ledger["recovery_errors"] = recovery_errors
        stop_publishers()
        for monitor in monitors:
            if monitor.poll() is None:
                monitor.terminate()
        for handle in handles:
            handle.close()
        ledger["finished_wall_time"] = time.time()
        save()
    print(json.dumps({key: ledger[key] for key in ("state", "passed", "clean_first_attempt")}))
    return 0 if ledger["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
