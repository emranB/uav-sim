"""Validate real add, rename and removal without restarting existing vehicles."""

import json
import os
import time
import urllib.request
from pathlib import Path


def request(resource: str, body: dict | None = None) -> dict:
    query = urllib.request.Request(
        f"http://127.0.0.1:{int(os.getenv('BACKEND_PORT', '18080'))}/" + resource,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json", "X-Request-Source": "catalog-validation"},
    )
    with urllib.request.urlopen(query, timeout=40) as response:
        return json.load(response)


def main() -> int:
    report = {"passed": False, "checks": []}
    destination = Path("outputs/reports") / f"catalog-{time.time_ns()}.json"
    identifier = None
    try:
        before = {
            vehicle["vehicle_id"]: vehicle["run_id"] for vehicle in request("vehicles")["items"]
        }
        created = request("vehicles/add", {"label": "Cedar validation", "model": "x500"})
        identifier = created["vehicle_id"]
        deadline = time.monotonic() + 90
        while True:
            vehicle = request("telemetry?vehicle_id=" + identifier)
            if vehicle["ready"]:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Added vehicle did not become ready")
            time.sleep(1)
        report["checks"].append(
            {"name": "added-vehicle-ready", "vehicle_id": identifier, "passed": True}
        )
        request("vehicles/rename", {"vehicle_id": identifier, "label": "Cedar renamed"})
        if request("workspace?vehicle_id=" + identifier)["vehicle"]["label"] != "Cedar renamed":
            raise RuntimeError("Vehicle rename was not visible")
        report["checks"].append({"name": "rename-visible", "passed": True})
        request("vehicles/remove", {"vehicle_id": identifier})
        after = {
            vehicle["vehicle_id"]: vehicle["run_id"] for vehicle in request("vehicles")["items"]
        }
        if before != after:
            raise RuntimeError("Catalog operation changed an existing vehicle generation")
        scene = request("scene")
        if any(pose["vehicle_id"] == identifier for pose in scene["poses"]):
            raise RuntimeError("Removed vehicle remained in the application scene")
        directory = Path("outputs/vehicles") / identifier
        if not list((directory / "telemetry").glob("*.jsonl")):
            raise RuntimeError("Removing the vehicle erased or failed to record its telemetry")
        report["checks"].append(
            {"name": "removed-with-peers-and-history-preserved", "passed": True}
        )
        identifier = None
        report["passed"] = True
        return 0
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        report["error"] = str(error)
        print(str(error), flush=True)
        return 1
    finally:
        if identifier:
            try:
                request("vehicles/remove", {"vehicle_id": identifier})
            except (OSError, ValueError) as error:
                report["cleanup_error"] = str(error)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report), flush=True)
        print(f"Report: {destination.resolve()}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
