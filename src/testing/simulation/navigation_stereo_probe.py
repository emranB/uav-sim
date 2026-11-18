"""Bounded live CPU stereo prefix with isolated runtime files and no flight commands."""

import argparse
import json
import tempfile
import time
from collections.abc import Callable
from pathlib import Path
from unittest.mock import patch

from gz.transport13 import Node

from common.configuration.fleet import VehicleDefinition, load_fleet
from integrations.vision.navigation_source import FileStereoObstacles, write_atomic_json
from integrations.vision.navigation_stereo import VehicleStereoObstacles


def wait_measurement(
    worker: VehicleStereoObstacles, predicate: Callable[[dict], bool], timeout_seconds: float = 25
) -> dict:
    """Run a bounded isolated worker until the requested observable output arrives."""
    deadline = time.monotonic() + timeout_seconds
    last = {}
    while time.monotonic() < deadline:
        worker.tick()
        if worker.output.exists():
            last = json.loads(worker.output.read_text())
            if predicate(last):
                return last
        time.sleep(0.05)
    raise TimeoutError(f"No expected stereo output within {timeout_seconds}s: {last.get('error')}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exercise-missing-link", action="store_true")
    arguments = parser.parse_args()
    definition = next(vehicle for vehicle in load_fleet() if vehicle.vehicle_id == "vehicle_2")
    controller = (definition.runtime_directory / "px4.run").read_text().strip()
    destination = Path(f"outputs/reports/stereo-default-cpu-live-prefix-{time.time_ns()}.json")
    report = {
        "schema_version": 1,
        "passed": False,
        "vehicle_id": definition.vehicle_id,
        "scope": "Live measured stereo processing in the default CPU image without GPU devices",
        "gpu_devices": [str(path) for path in Path("/dev").glob("nvidia*")],
    }
    with tempfile.TemporaryDirectory() as name:
        directory = Path(name)
        (directory / "px4.run").write_text(controller)
        # Isolate the probe's writes; subscribe to the real vehicle's image topics only.
        with patch.object(VehicleDefinition, "runtime_directory", property(lambda self: directory)):
            worker = VehicleStereoObstacles(Node(), definition, "cpu")
            try:
                record = wait_measurement(worker, lambda output: output.get("ready", False))
                if worker.archive_limit:
                    archive = (
                        definition.output_directory(Path("outputs"))
                        / "perception"
                        / "navigation-stereo"
                        / record["source_session_id"]
                        / "inputs"
                    )
                    report["input_archive"] = str(archive)
                    report["archived_images"] = len(list(archive.glob("*.ppm")))
                parsed = FileStereoObstacles(definition.vehicle_id, directory).latest(controller)
                report["parsed_point_count"] = len(parsed.points)
                report["point_sampling_stride_px"] = parsed.sampling_stride_px
                report["measurement"] = record
                if arguments.exercise_missing_link:
                    write_atomic_json(
                        worker.control,
                        {"controller_run_id": controller, "settings": {"fault": "missing_left"}},
                    )
                    missing = wait_measurement(
                        worker,
                        lambda output: (
                            not output.get("ready") and "missing_left" in output.get("error", "")
                        ),
                        5,
                    )
                    assert missing["subscribed_sides"] == ["right"]
                    write_atomic_json(
                        worker.control,
                        {"controller_run_id": controller, "settings": {"fault": "none"}},
                    )
                    restored = wait_measurement(
                        worker,
                        lambda output: (
                            output.get("ready", False)
                            and output["timestamp_seconds"] > record["timestamp_seconds"]
                        ),
                        10,
                    )
                    report["link_recovery"] = {
                        "missing": missing,
                        "restored_timestamp_seconds": restored["timestamp_seconds"],
                        "source_generation_preserved": restored["source_session_id"]
                        == record["source_session_id"],
                    }
                    assert report["link_recovery"]["source_generation_preserved"]
                report["passed"] = not report["gpu_devices"] and record["backend"] == "cpu"
            except (OSError, RuntimeError, ValueError, KeyError, AssertionError) as error:
                report["error"] = str(error)
            finally:
                worker.close()
    destination.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
