"""Read-only integration checks plus rejection tests for invalid local commands."""

import argparse
import copy
import hashlib
import json
import subprocess
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

from common.logging.storage import write_json_atomic


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--report", type=Path)
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    report = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "checks": []}

    def check(name: str, condition: bool, evidence: object) -> None:
        report["checks"].append({"name": name, "passed": condition, "evidence": evidence})
        print(json.dumps({"check": name, "passed": condition}), flush=True)

    def request(path: str, body: dict | None = None, origin: str | None = None) -> tuple:
        headers = {"Content-Type": "application/json"}
        if origin:
            headers["Origin"] = origin
        query = urllib.request.Request(
            f"http://127.0.0.1:{arguments.port}/{path}",
            data=json.dumps(body).encode() if body is not None else None,
            headers=headers,
        )
        try:
            with urllib.request.urlopen(query, timeout=15) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            return error.code, json.load(error)

    try:
        deadline = time.monotonic() + 45
        while True:
            try:
                status, capabilities = request("capabilities")
                if status == 200:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise TimeoutError("Simulation did not become ready within 45 seconds")
            time.sleep(1)
        check(
            "real-simulation-ready",
            status == 200 and capabilities.get("real_simulator") is True,
            capabilities,
        )
        deadline = time.monotonic() + 30
        while not capabilities.get("ros_connected") and time.monotonic() < deadline:
            time.sleep(1)
            _, capabilities = request("capabilities")
        check("ros2-position-fresh", capabilities.get("ros_connected") is True, capabilities)
        _, scene = request("scene")
        check(
            "gazebo-ground-truth",
            bool(scene.get("poses")) and not scene.get("stale") and scene.get("frame") == "ENU",
            scene,
        )
        status, error = request("mission", {"fileType": "unsupported"})
        check("invalid-mission-rejected", status == 400, error)
        plan = json.loads((root / "configs/missions/qgc-ui-validation.plan").read_text())
        for name, field, value in (
            ("non-integer-command-rejected", "command", 22.0),
            ("manual-progression-rejected", "autoContinue", False),
        ):
            invalid = copy.deepcopy(plan)
            invalid["mission"]["items"][0][field] = value
            status, error = request("mission", invalid)
            check(name, status == 400, error)
        status, error = request("pause", {}, origin="https://untrusted.invalid")
        check("browser-origin-command-rejected", status == 403, error)
        status, error = request("not-an-endpoint")
        check("unknown-endpoint", status == 404, error)
        telemetry = [
            str(path.relative_to(root))
            for path in (root / "outputs/telemetry").glob("*.jsonl")
            if path.stat().st_size > 0
        ]
        # PX4 run roots contain Linux symlinks; do not traverse them from Windows.
        recordings = list((root / "outputs/px4").glob("run-*/log/*/*.ulg"))
        recordings.extend((root / "outputs/px4/log").glob("*/*.ulg"))
        flight_logs = [
            str(path.relative_to(root)) for path in recordings if path.stat().st_size > 0
        ]
        check("persistent-telemetry", bool(telemetry), telemetry)
        check("persistent-flight-recordings", bool(flight_logs), flight_logs)
        if not arguments.headless:
            for title in ("QGroundControl", "Gazebo"):
                script = (
                    "import json,sys; from pathlib import Path; "
                    "from desktop.native_window import window_ancestry; "
                    "child=int(Path('/runtime/'+sys.argv[1]+'.window').read_text()); "
                    "host=int(Path('/runtime/Desktop.window').read_text()); "
                    "ancestors=window_ancestry(child); "
                    "print(json.dumps({'attached':host in ancestors,'ancestors':ancestors}))"
                )
                deadline = time.monotonic() + 20
                while True:
                    result = subprocess.run(
                        [
                            "docker",
                            "compose",
                            "exec",
                            "-T",
                            "desktop",
                            "python",
                            "-c",
                            script,
                            title,
                        ],
                        cwd=root,
                        capture_output=True,
                        text=True,
                        timeout=10,
                        check=False,
                    )
                    try:
                        parent = json.loads(result.stdout).get("attached", False)
                    except ValueError:
                        parent = False
                    if parent or time.monotonic() >= deadline:
                        break
                    time.sleep(1)
                check(
                    f"embedded-{title.lower()}",
                    result.returncode == 0 and bool(parent),
                    result.stdout + result.stderr,
                )
        report["image_versions"] = []
        for service in ("control-panel", "qgroundcontrol", "ros2-monitor"):
            container = subprocess.check_output(
                ["docker", "compose", "ps", "-q", service],
                text=True,
                timeout=10,
            ).strip()
            if not container:
                raise RuntimeError(f"{service}: running Compose container was not found")
            image = subprocess.check_output(
                ["docker", "inspect", "--format", "{{.Image}}", container],
                text=True,
                timeout=10,
            ).strip()
            metadata = json.loads(
                subprocess.check_output(
                    ["docker", "image", "inspect", image],
                    timeout=10,
                )
            )[0]
            report["image_versions"].append(
                {
                    "service": service,
                    **{
                        key: metadata[key]
                        for key in ("Id", "Created", "Os", "Architecture", "RepoTags")
                    },
                }
            )
        report["configuration_hashes"] = {
            str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (root / "configs").rglob("*")
            if path.is_file()
        }
        runtime_fleet = root / "outputs/fleet.json"
        if runtime_fleet.exists():
            report["runtime_fleet"] = json.loads(runtime_fleet.read_text(encoding="utf-8"))
            report["configuration_hashes"]["runtime/fleet.json"] = hashlib.sha256(
                runtime_fleet.read_bytes()
            ).hexdigest()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        check("validation-execution", False, str(error))
    report["passed"] = all(item["passed"] for item in report["checks"])
    destination = (
        arguments.report or root / "outputs/reports" / f"integration-{time.time_ns()}.json"
    )
    write_json_atomic(destination, report)
    print(
        json.dumps(
            {"passed": report["passed"], "checks": report["checks"], "report": str(destination)}
        )
    )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
