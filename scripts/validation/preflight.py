"""Inspect the Windows development host without installing or reconfiguring software."""

import argparse
import importlib.util
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def run_check(name: str, command: list[str], timeout_seconds: int = 30) -> dict:
    """Capture a bounded command and its evidence in a JSON-serializable result."""
    try:
        completed = subprocess.run(
            command,
            cwd=ROOT,
            capture_output=True,
            timeout=timeout_seconds,
            check=False,
        )
        output = completed.stdout + completed.stderr
        # WSL CLI output on Windows can be UTF-16LE while Linux output is UTF-8.
        encoding = "utf-16-le" if b"\x00" in output else "utf-8"
        return {
            "name": name,
            "status": "passed" if completed.returncode == 0 else "failed",
            "command": command,
            "exit_code": completed.returncode,
            "evidence": output.decode(encoding, errors="replace").strip(),
        }
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"name": name, "status": "failed", "command": command, "reason": str(error)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--minimum-free-gib", type=float, default=20)
    parser.add_argument(
        "--probe-wslg",
        action="store_true",
        help="Use an existing image to test the WSLg X11 handshake.",
    )
    parser.add_argument("--probe-image", default="uav-sim-control-panel")
    arguments = parser.parse_args()
    timestamp = datetime.now(timezone.utc)
    checks = [
        run_check("python", [sys.executable, "--version"]),
        run_check(
            "docker-engine",
            [
                "docker",
                "info",
                "--format",
                "Server={{.ServerVersion}} OS={{.OSType}} Arch={{.Architecture}} "
                "CPUs={{.NCPU}} Memory={{.MemTotal}}",
            ],
        ),
        run_check("compose", ["docker", "compose", "version"]),
        run_check("compose-configuration", ["docker", "compose", "config", "--quiet"]),
        run_check("source-revision", ["git", "rev-parse", "HEAD"]),
    ]
    # Revision absence is evidence about provenance, not a host capability failure.
    if checks[-1]["status"] == "failed":
        checks[-1]["status"] = "warning"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(ROOT / "src")
    previous_path = os.environ.get("PYTHONPATH")
    try:
        os.environ["PYTHONPATH"] = environment["PYTHONPATH"]
        checks.append(
            run_check(
                "unit-tests",
                [sys.executable, "-m", "unittest", "discover", "-s", "src/testing/unit", "-v"],
            )
        )
    finally:
        if previous_path is None:
            os.environ.pop("PYTHONPATH", None)
        else:
            os.environ["PYTHONPATH"] = previous_path
    free_gib = shutil.disk_usage(ROOT).free / 1024**3
    checks.append(
        {
            "name": "workspace-disk-budget",
            "status": "passed" if free_gib >= arguments.minimum_free_gib else "failed",
            "free_gib": round(free_gib, 2),
            "minimum_free_gib": arguments.minimum_free_gib,
            "reason": "Planning budget for simulator downloads/builds; not an upstream minimum.",
        }
    )
    checks.append(
        {
            "name": "host-desktop-dependency",
            "status": "passed" if importlib.util.find_spec("PySide6") else "skipped",
            "reason": "Checks host installation only; container GUI dependencies are separate.",
        }
    )
    if os.name == "nt":
        checks.extend(
            [
                run_check("wsl-version", ["wsl", "--version"]),
                run_check("wsl-distributions", ["wsl", "--list", "--verbose"]),
                run_check(
                    "gpu-inventory",
                    [
                        "powershell",
                        "-NoProfile",
                        "-Command",
                        "Get-CimInstance Win32_VideoController | "
                        "Select-Object Name, DriverVersion | ConvertTo-Json",
                    ],
                ),
            ]
        )
        if arguments.probe_wslg:
            probe = (
                "import socket,struct,json; "
                "s=socket.socket(socket.AF_UNIX); s.settimeout(3); "
                "s.connect('/probe/wslg/.X11-unix/X0'); "
                "s.sendall(struct.pack('<BBHHHHH',108,0,11,0,0,0,0)); "
                "reply=s.recv(8); "
                "accepted=bool(reply) and reply[0]==1; "
                "print(json.dumps({'x11_protocol_accepted':accepted,"
                "'rendering_validated':False})); s.close(); "
                "raise SystemExit(0 if accepted else 1)"
            )
            checks.append(
                run_check(
                    "container-wslg-handshake",
                    [
                        "docker",
                        "run",
                        "--rm",
                        "--pull=never",
                        "--network",
                        "none",
                        "--mount",
                        "type=bind,source=/mnt/host/wslg,target=/probe/wslg,readonly",
                        arguments.probe_image,
                        "python",
                        "-c",
                        probe,
                    ],
                )
            )
    else:
        checks.append(
            {"name": "windows-wsl", "status": "skipped", "reason": "This host is not Windows."}
        )
    for name in (
        "qgc-rendering",
        "gazebo-rendering",
        "gpu-rendering",
        "foreign-window-embedding",
        "dds-discovery",
        "px4-connection",
    ):
        checks.append(
            {
                "name": name,
                "status": "skipped",
                "reason": "Requires the real component integration smoke test.",
            }
        )
    report = {
        "schema_version": 1,
        "timestamp_utc": timestamp.isoformat(),
        "scope": "host-preflight",
        "full_stack_validated": False,
        "checks": checks,
    }
    destination = ROOT / "outputs/reports" / timestamp.strftime("%Y%m%dT%H%M%S%fZ")
    destination.mkdir(parents=True)
    report_path = destination / "preflight.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    failed = [check["name"] for check in checks if check["status"] == "failed"]
    print(
        json.dumps(
            {"report": str(report_path), "failed_checks": failed, "full_stack_validated": False}
        )
    )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
