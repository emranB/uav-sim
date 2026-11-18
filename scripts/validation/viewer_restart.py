"""Verify that restarting a viewer leaves physics and vehicle telemetry running."""

import argparse
import json
import subprocess
import time
import urllib.request
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", choices=["gazebo-viewer", "desktop"], default="gazebo-viewer")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument(
        "--unique-report", action="store_true", help="Preserve earlier restart reports"
    )
    parser.add_argument(
        "--verify-close",
        action="store_true",
        help="Verify desktop exit and hidden native views before reopening.",
    )
    parser.add_argument(
        "--verify-failure-recovery",
        action="store_true",
        help="Kill only the owned Gazebo GUI child and verify automatic recovery.",
    )
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    report = {"service": arguments.service, "passed": False, "timestamp": time.time()}

    def read(path: str) -> dict:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{arguments.port}/{path}", timeout=5
        ) as response:
            return json.load(response)

    def physics_container() -> str:
        return subprocess.check_output(
            ["docker", "compose", "ps", "-q", "gazebo"], cwd=root, text=True, timeout=10
        ).strip()

    def wait_telemetry_ready() -> dict:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            state = read("telemetry?vehicle_id=vehicle_1")
            if state["ready"]:
                return state
            time.sleep(0.5)
        raise TimeoutError("Vehicle telemetry did not become ready within 60 seconds")

    def views() -> dict:
        script = (
            "import json; from pathlib import Path; "
            "from desktop.native_window import window_ancestry; "
            "host=int(Path('/runtime/Desktop.window').read_text()); "
            "identifiers={name:int(Path('/runtime/'+name+'.window').read_text()) "
            "for name in ['Gazebo','QGroundControl']}; "
            "print(json.dumps({'ids':identifiers,'attached':all(host in window_ancestry(identifier) "
            "for identifier in identifiers.values())}))"
        )
        return json.loads(
            subprocess.check_output(
                ["docker", "compose", "exec", "-T", "desktop", "python", "-c", script],
                cwd=root,
                timeout=10,
                stderr=subprocess.DEVNULL,
            )
        )

    def wait_embedded_views() -> dict:
        deadline = time.monotonic() + 30
        last_error = "Native viewers have not attached"
        while time.monotonic() < deadline:
            try:
                current = views()
                if current["attached"]:
                    return current
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                last_error = str(error)
            time.sleep(0.5)
        raise TimeoutError(f"Native viewers did not become ready: {last_error}")

    try:
        before = read("scene")
        identifier = physics_container()
        views_before = wait_embedded_views()
        if not identifier or not before["poses"]:
            raise RuntimeError("Start the real stack and wait for vehicle readiness first")
        wait_telemetry_ready()
        if arguments.verify_close:
            if arguments.service != "desktop":
                raise ValueError("Close validation applies to the desktop service")
            desktop_id = subprocess.check_output(
                ["docker", "compose", "ps", "-q", "desktop"], cwd=root, text=True
            ).strip()
            subprocess.run(
                ["docker", "compose", "stop", "desktop"], cwd=root, timeout=30, check=True
            )
            exit_code = int(
                subprocess.check_output(
                    ["docker", "inspect", "--format", "{{.State.ExitCode}}", desktop_id], text=True
                )
            )
            hidden_script = (
                "import json,subprocess; from pathlib import Path; "
                "identifiers=[Path('/runtime/'+name+'.window').read_text().strip() for name in ['Gazebo','QGroundControl']]; "
                "hidden=all('IsUnMapped' in subprocess.check_output(['xwininfo','-id',identifier],text=True) for identifier in identifiers); "
                "print(json.dumps({'hidden':hidden}))"
            )
            hidden = json.loads(
                subprocess.check_output(
                    [
                        "docker",
                        "compose",
                        "exec",
                        "-T",
                        "gazebo-viewer",
                        "python",
                        "-c",
                        hidden_script,
                    ],
                    cwd=root,
                    text=True,
                    timeout=10,
                )
            )["hidden"]
            report["graceful_exit_code"] = exit_code
            report["closed_views_hidden"] = hidden
            if exit_code != 0 or not hidden:
                raise RuntimeError("Closing the desktop left a visible viewer or an abnormal exit")
        if arguments.verify_failure_recovery:
            if arguments.service != "gazebo-viewer" or arguments.verify_close:
                raise ValueError("Failure recovery applies only to the Gazebo viewer")
            viewer_id = subprocess.check_output(
                ["docker", "compose", "ps", "-q", "gazebo-viewer"], text=True
            ).strip()

            def restart_count() -> int:
                return int(
                    subprocess.check_output(
                        ["docker", "inspect", "--format", "{{.RestartCount}}", viewer_id], text=True
                    )
                )

            count_before = restart_count()
            injection = "\n".join(
                [
                    "import os, signal",
                    "from pathlib import Path",
                    "groups = set()",
                    "for entry in Path('/proc').iterdir():",
                    "    if not entry.name.isdigit(): continue",
                    "    try: args = (entry/'cmdline').read_bytes().split(bytes([0]))",
                    "    except (FileNotFoundError, PermissionError): continue",
                    "    parts = args[0].split() if args else []",
                    "    if parts[:2] == [b'gz', b'sim'] and b'-g' in parts:",
                    "        groups.add(os.getpgid(int(entry.name)))",
                    "if len(groups) != 1: raise RuntimeError('Expected one owned Gazebo GUI group')",
                    "os.killpg(groups.pop(), signal.SIGKILL)",
                ]
            )
            subprocess.run(
                ["docker", "compose", "exec", "-T", "gazebo-viewer", "python", "-c", injection],
                timeout=15,
                check=True,
            )
            deadline = time.monotonic() + 60
            while restart_count() <= count_before:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Gazebo GUI did not restart automatically")
                time.sleep(0.5)
            report["automatic_restart_count_before"] = count_before
            report["automatic_restart_count_after"] = restart_count()
            report["automatic_failure_recovery"] = True
        else:
            subprocess.run(
                [
                    "docker",
                    "compose",
                    "start" if arguments.verify_close else "restart",
                    arguments.service,
                ],
                cwd=root,
                timeout=45,
                check=True,
            )
        deadline = time.monotonic() + 30
        while True:
            after = read("scene")
            if (
                after["poses"]
                and not after["stale"]
                and after["poses"][0]["simulation_time_seconds"]
                > before["poses"][0]["simulation_time_seconds"]
            ):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Physics did not advance after viewer restart")
            time.sleep(1)
        report.update(
            physics_container_unchanged=physics_container() == identifier,
            telemetry_ready=wait_telemetry_ready()["ready"],
            clock_before=before["poses"][0]["simulation_time_seconds"],
            clock_after=after["poses"][0]["simulation_time_seconds"],
        )
        views_after = wait_embedded_views()
        preserved = (
            ["Gazebo", "QGroundControl"] if arguments.service == "desktop" else ["QGroundControl"]
        )
        report["independent_viewers_preserved"] = all(
            views_before["ids"][name] == views_after["ids"][name] for name in preserved
        )
        report["embedded_views_ready"] = True
        report["passed"] = (
            report["physics_container_unchanged"]
            and report["telemetry_ready"]
            and report["independent_viewers_preserved"]
        )
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        report["error"] = str(error)
    name = (
        f"{arguments.service}-failure-recovery-{time.time_ns()}.json"
        if arguments.verify_failure_recovery
        else f"{arguments.service}-restart-{time.time_ns()}.json"
        if arguments.unique_report
        else f"{arguments.service}-restart.json"
    )
    destination = root / "outputs/reports" / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({**report, "report": str(destination)}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
