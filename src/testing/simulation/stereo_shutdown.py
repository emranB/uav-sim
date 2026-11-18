"""Exercise owned stereo worker termination with isolated files and no flight commands."""

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import patch

from common.configuration.fleet import VehicleDefinition, load_fleet
from integrations.vision.navigation_source import FileStereoObstacles


def child_main(directory: Path) -> int:
    """Run the actual signal-handling worker against one temporary runtime directory."""
    from integrations.vision import navigation_stereo

    definition = next(vehicle for vehicle in load_fleet() if vehicle.vehicle_id == "vehicle_2")
    with (
        patch.object(VehicleDefinition, "runtime_directory", property(lambda self: directory)),
        patch.object(navigation_stereo, "load_fleet", return_value=(definition,)),
    ):
        return navigation_stereo.main()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--child", type=Path)
    arguments = parser.parse_args()
    if arguments.child:
        return child_main(arguments.child)
    definition = next(vehicle for vehicle in load_fleet() if vehicle.vehicle_id == "vehicle_2")
    controller = (definition.runtime_directory / "px4.run").read_text().strip()
    report = {"schema_version": 1, "passed": False, "checks": [], "scope": __doc__}
    destination = Path(f"outputs/reports/stereo-owned-shutdown-{time.time_ns()}.json")
    try:
        for termination in [signal.SIGTERM, signal.SIGKILL]:
            with tempfile.TemporaryDirectory() as name:
                directory = Path(name)
                (directory / "px4.run").write_text(controller)
                with (directory / "worker.log").open("w+") as log:
                    child = subprocess.Popen(
                        [
                            sys.executable,
                            "-m",
                            "testing.simulation.stereo_shutdown",
                            "--child",
                            name,
                        ],
                        env={
                            **os.environ,
                            "UAV_NAV_STEREO_BACKEND": "cpu",
                            "UAV_NAV_STEREO_ARCHIVE_BATCHES": "0",
                        },
                        stdout=log,
                        stderr=subprocess.STDOUT,
                    )
                    try:
                        deadline = time.monotonic() + 30
                        source = FileStereoObstacles(definition.vehicle_id, directory)
                        while True:
                            try:
                                source.latest(controller)
                                break
                            except (OSError, RuntimeError):
                                if child.poll() is not None or time.monotonic() > deadline:
                                    raise RuntimeError(
                                        "Isolated worker did not publish live evidence"
                                    )
                                time.sleep(0.05)
                        started = time.monotonic()
                        child.send_signal(termination)
                        returncode = child.wait(timeout=10)
                        elapsed = time.monotonic() - started
                        assert returncode == (0 if termination == signal.SIGTERM else -9)
                        json.loads(source.path.read_text())
                        time.sleep(2)
                        try:
                            source.latest(controller)
                        except RuntimeError as error:
                            assert "stale" in str(error).lower()
                            stale_rejected = True
                        else:
                            raise AssertionError("Terminated worker's old result remained usable")
                        log.seek(0)
                        report["checks"].append(
                            {
                                "signal": termination.name,
                                "passed": True,
                                "returncode": returncode,
                                "shutdown_wall_seconds": elapsed,
                                "last_record_complete": True,
                                "stale_result_rejected": stale_rejected,
                                "worker_log": log.read(),
                            }
                        )
                    finally:
                        if child.poll() is None:
                            child.kill()
                            child.wait(timeout=3)
        assert (definition.runtime_directory / "px4.run").read_text().strip() == controller
        report["controller_generation_preserved"] = True
        report["passed"] = True
    except (OSError, RuntimeError, ValueError, AssertionError, subprocess.SubprocessError) as error:
        report["error"] = str(error)
    destination.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
