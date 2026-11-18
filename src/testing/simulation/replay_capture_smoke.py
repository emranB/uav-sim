"""Read-only native telemetry smoke through the real replay source and HTTP handlers.

The existing deployment receives GET requests only. A separate loopback test API owns
capture/export files. This checks actual idle PX4 estimates, not airborne acceptance.
"""

import argparse
import json
import math
import subprocess
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace

from common.interfaces.navigation import NavigationPose
from common.interfaces.replay import ReplayCaptureLimits
from common.logging.replay_capture import ReplayCapture
from common.logging.replay_export import ReplayExportJobs
from common.logging.replay_store import ReplayCaptureStore
from common.logging.storage import write_json_atomic
from common.simulation_manager.server import Handler
from integrations.blender.queue import FileReplayRenderer
from integrations.blender.renderer import BlenderReplayRenderer
from integrations.replay import WorkspaceReplaySource


class NativeReadOnlyApi:
    """The upstream deployment is never sent a command by this smoke observer."""

    def __init__(self, url: str) -> None:
        self.url = url
        self.get_count = 0

    def read(self, route: str) -> dict:
        with urllib.request.urlopen(self.url + "/" + route, timeout=3) as response:
            encoded = response.read(1024**2 + 1)
        if len(encoded) > 1024**2:
            raise ValueError("Native observation exceeds 1 MiB")
        self.get_count += 1
        return json.loads(encoded)


class NativeHttpVehicle:
    """Observe actual PX4 estimates; HTTP receipt time is explicitly the sampling clock."""

    def __init__(self, api: NativeReadOnlyApi, identity: str) -> None:
        self.api, self.identity = api, identity
        self.latest = None
        self.pose = None
        self.previous_stamp = None

    def snapshot(self) -> dict:
        self.latest = self.api.read(f"telemetry?vehicle_id={self.identity}")
        return self.latest

    def navigation_pose(self) -> NavigationPose:
        snapshot = self.latest
        position = snapshot["messages"]["LOCAL_POSITION_NED"]
        attitude = snapshot["messages"]["ATTITUDE"]
        if not snapshot["connected"]:
            raise RuntimeError("Native telemetry link unavailable")
        stamp = snapshot["run_id"], position["time_boot_ms"]
        if stamp != self.previous_stamp:
            self.pose = NavigationPose(
                (position["y"], position["x"], -position["z"]),
                math.pi / 2 - attitude["yaw"],
                time.monotonic(),
                snapshot["run_id"],
                snapshot["armed"],
                snapshot["flight_mode"],
                position["time_boot_ms"] / 1000,
                attitude["roll"],
                -attitude["pitch"],
            )
            self.previous_stamp = stamp
        if self.pose is None or time.monotonic() - self.pose.received_monotonic > 1:
            raise RuntimeError("Native estimated position has stopped advancing")
        return self.pose


class NativeFeatures:
    def __init__(self, api, identity):
        workspace = api.read(f"workspace?vehicle_id={identity}")
        self.configuration, self.world = workspace["vehicle"], workspace["world"]

    def versions(self) -> dict:
        return {"native_revisions": "Not exposed by the upstream API used for this read-only smoke"}


class NativeScene:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def snapshot(self) -> dict:
        return {"epoch": self.vehicle.latest.get("scene_epoch", 0), "stale": True, "poses": []}


class NativeCamera:
    def __init__(self, api, identity):
        self.api, self.identity = api, identity

    def snapshot(self) -> dict:
        return self.api.read(f"perception?vehicle_id={self.identity}").get("camera", {})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native-url", default="http://127.0.0.1:18080")
    parser.add_argument("--blender")
    parser.add_argument("--renderer-runtime", type=Path)
    args = parser.parse_args()
    if not args.blender and args.renderer_runtime is None:
        parser.error("Set an isolated renderer runtime or an explicit optional Blender executable")
    if args.native_url != "http://127.0.0.1:18080":
        parser.error("This observer is scoped to the existing local UAV API")
    if args.output.exists():
        parser.error("Choose a new smoke directory; previous evidence cannot be overwritten")
    args.output.mkdir(parents=True)
    write_json_atomic(
        args.output / "result.json",
        {
            "passed": False,
            "phase": "initial_observations",
            "airborne_acceptance_claimed": False,
        },
    )
    api = NativeReadOnlyApi(args.native_url)
    before = api.read("vehicles")["items"]
    if len(before) != 2 or not all(row["ready"] and row["armed"] is False for row in before):
        raise RuntimeError("Run the idle smoke only with both native vehicles ready and disarmed")
    services = {}
    for row in before:
        identity = row["vehicle_id"]
        vehicle = NativeHttpVehicle(api, identity)
        services[identity] = SimpleNamespace(
            vehicle=vehicle,
            features=NativeFeatures(api, identity),
            navigation=None,
            camera_status=NativeCamera(api, identity),
            scene_source=NativeScene(vehicle),
            perception=SimpleNamespace(
                snapshot=lambda vehicle_id, description=api.read(f"perception?vehicle_id={identity}"): (
                    description
                )
            ),
        )
    source = WorkspaceReplaySource(services)
    store = ReplayCaptureStore(args.output / "captures", ReplayCaptureLimits(maximum_seconds=10))
    renderer = (
        FileReplayRenderer(args.output / "replay-jobs", args.renderer_runtime)
        if args.renderer_runtime is not None
        else BlenderReplayRenderer(args.blender)
    )
    jobs = ReplayExportJobs(store, args.output / "replay-jobs", renderer)
    capture = ReplayCapture(source, store, jobs)
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    server.replay, server.restore = capture, None
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    local = f"http://127.0.0.1:{server.server_port}"

    def command(route, body):
        request = urllib.request.Request(
            local + "/" + route, json.dumps(body).encode(), {"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(request, timeout=5) as response:
            return json.load(response)

    result = {
        "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "kind": "actual idle PX4 estimates via read-only HTTP observer",
        "production_workspace_replay_source": True,
        "production_http_handlers": True,
        "existing_api_commands_sent": 0,
        "airborne_acceptance_claimed": False,
        "truth_available": False,
        "receipt_clock": "observer HTTP receipt monotonic time",
        "passed": False,
    }
    try:
        started = command("replay/start", {"target_ids": list(services)})
        if started["state"] != "capturing":
            raise RuntimeError("Replay writer did not acknowledge capture")
        time.sleep(2)
        command("replay/stop", {})
        deadline = time.monotonic() + 5
        while capture.snapshot()["state"] in {"capturing", "finalizing"}:
            if time.monotonic() > deadline:
                raise TimeoutError("Native replay interval did not finalize")
            time.sleep(0.05)
        manifest = store.manifest(started["capture_id"], verify_data=True)
        if not all(
            row["samples"] >= 5 and not row["error"] for row in manifest["vehicles"].values()
        ):
            raise RuntimeError("Actual native observations were not captured for both vehicles")
        command("replay/keep", {"capture_id": started["capture_id"]})
        command(
            "replay/render",
            {
                "capture_id": started["capture_id"],
                "start_offset_seconds": 0.4,
                "end_offset_seconds": 1.4,
            },
        )
        deadline = time.monotonic() + 120
        while jobs.snapshot()["state"] in {"queued", "running", "rendering"}:
            if time.monotonic() > deadline:
                raise TimeoutError("Small native render did not complete")
            time.sleep(0.25)
        if jobs.snapshot()["state"] != "completed":
            raise RuntimeError(jobs.snapshot().get("error"))
        movie = Path(jobs.snapshot()["path"]) / "trajectory-preview.mp4"
        if not movie.is_file() or movie.stat().st_size == 0:
            raise RuntimeError("Native trajectory frames exist but a short MP4 was not encoded")
        after = api.read("vehicles")["items"]
        if not all(
            row["ready"] and row["armed"] is False and row["recording"]["available"]
            for row in after
        ):
            raise RuntimeError("Native readiness/diagnostic recording changed during idle smoke")
        if [row["run_id"] for row in before] != [row["run_id"] for row in after]:
            raise RuntimeError("Native controller generation changed during read-only smoke")
        result.update(
            passed=True,
            capture=manifest,
            offline_job=jobs.snapshot(),
            upstream_get_requests=api.get_count,
            native_after=[
                {
                    "vehicle_id": row["vehicle_id"],
                    "run_id": row["run_id"],
                    "ready": row["ready"],
                    "armed": row["armed"],
                    "recording": row["recording"],
                }
                for row in after
            ],
        )
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        result["error"] = str(error)
    finally:
        server.shutdown()
        server.server_close()
        capture.close()
        write_json_atomic(args.output / "result.json", result)
    print(
        json.dumps(
            {"passed": result["passed"], "output": str(args.output), "error": result.get("error")}
        )
    )
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
