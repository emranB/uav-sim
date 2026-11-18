"""Admit a finite validation budget without deleting or reserving historical storage."""

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

from common.configuration.fleet import load_fleet
from common.logging.storage import ensure_capacity, write_json_atomic


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generations", type=int, required=True)
    parser.add_argument("--run-allowance-mib", type=int, default=512)
    arguments = parser.parse_args()
    if not 1 <= arguments.generations <= 100 or not 256 <= arguments.run_allowance_mib <= 2048:
        parser.error(
            "Use 1..100 two-vehicle-equivalent generations and 256..2048 MiB core allowance"
        )
    configuration = json.loads(
        subprocess.check_output(["docker", "compose", "config", "--format", "json"], text=True)
    )
    services = configuration["services"]
    camera = services.get("camera-streams", {}).get("environment", {})
    mapping = services.get("live-mapping", {}).get("environment", {})
    stereo = services.get("navigation-stereo", {}).get("environment", {})
    policy = services["control-panel"].get("environment", {})
    output_mount = next(
        mount
        for mount in services["control-panel"]["volumes"]
        if mount["target"] == "/workspace/outputs" and mount["type"] == "bind"
    )
    root = Path(output_mount["source"])
    fleet_path = root / "fleet.json"
    vehicles = sum(
        v.enabled
        for v in load_fleet(
            fleet_path if fleet_path.exists() else Path("configs/vehicles/fleet.json")
        )
    )
    width, height = (
        int(camera.get("UAV_CAMERA_WIDTH", 320)),
        int(camera.get("UAV_CAMERA_HEIGHT", 240)),
    )
    pairs = int(camera.get("UAV_RECORD_LIMIT_PAIRS", 300))
    mapping_pairs = int(mapping.get("UAV_MAPPING_INPUT_ARCHIVE_PAIRS", 0))
    batches = int(stereo.get("UAV_NAV_STEREO_ARCHIVE_BATCHES", 0))
    reserve = int(policy.get("UAV_STORAGE_RESERVE_BYTES", 2 * 1024**3))
    image_bytes = vehicles * (
        pairs * 2 * (width * height * 3 + 64)
        + mapping_pairs * 2 * 1024**2
        + batches * 12 * (320 * 240 * 3 + 64)
    )
    core_bytes = arguments.run_allowance_mib * 1024**2
    metadata_bytes = 16 * 1024**2
    required = arguments.generations * (image_bytes + core_bytes + metadata_bytes)
    filesystem = os.statvfs(root)
    capacity = {
        "available_bytes": filesystem.f_bavail * filesystem.f_frsize,
        "available_inodes": filesystem.f_favail,
    }
    report = {
        "schema_version": 1,
        "passed": False,
        "created_wall_time": time.time(),
        "generations": arguments.generations,
        "enabled_vehicles": vehicles,
        "camera_width": width,
        "camera_height": height,
        "camera_pairs_per_generation": pairs,
        "mapping_input_pairs_per_generation": mapping_pairs,
        "navigation_stereo_batches_per_generation": batches,
        "image_bytes_per_generation": image_bytes,
        "core_allowance_bytes_per_generation": core_bytes,
        "metadata_allowance_bytes_per_generation": metadata_bytes,
        "required_new_bytes": required,
        "reserve_bytes": reserve,
        "required_total_free_bytes": required + reserve,
        "additional_required_bytes": max(0, required + reserve - capacity["available_bytes"]),
        "capacity": capacity,
        "assumption": "Declared finite profile: capped raw archives plus a core telemetry/controller/log allowance per generation. Admission is not a filesystem reservation against other applications; producers and execution recheck capacity. No evidence deletion.",
    }
    try:
        report["capacity"] = ensure_capacity(root, required, reserve)
        required_inodes = (
            arguments.generations * vehicles * (pairs * 2 + mapping_pairs * 2 + batches * 13 + 128)
            + 1024
        )
        report["required_inodes"] = required_inodes
        if report["capacity"]["available_inodes"] < required_inodes:
            raise OSError("Insufficient inodes for the declared finite campaign")
        report["passed"] = True
    except OSError as error:
        report["error"] = str(error)
    destination = root / "reports" / f"storage-preflight-{time.time_ns()}.json"
    write_json_atomic(destination, report)
    write_json_atomic(
        root / "reports/storage-preflight-current.json",
        {
            "passed": report["passed"],
            "report": str(destination),
            "required_total_free_bytes": report["required_total_free_bytes"],
            "capacity": report.get("capacity"),
            "error": report.get("error"),
        },
    )
    print(
        json.dumps(
            {
                "passed": report["passed"],
                "report": str(destination),
                "required_free_gib": (required + reserve) / 1024**3,
                "available_gib": report.get("capacity", {}).get("available_bytes", 0) / 1024**3,
            }
        )
    )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
