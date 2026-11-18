"""Compile the pinned POSIX timer queue with explicit test-only OS shims."""

import argparse
import hashlib
import json
import shutil
import subprocess
import time
from pathlib import Path

from common.logging.storage import write_json_atomic

SCENARIOS = (
    "normal",
    "slow-interval",
    "fast-interval",
    "future-interval",
    "cancel-reschedule",
    "cancel",
    "one-shot",
    "delay",
)
CONTROLS = {"normal", "cancel", "one-shot", "delay"}


def execute(command: list[str], log_path: Path, timeout_seconds: float) -> dict:
    """Retain output and distinguish a process failure from a bounded timeout."""
    with log_path.open("w") as output:
        try:
            result = subprocess.run(
                command, stdout=output, stderr=subprocess.STDOUT, timeout=timeout_seconds
            )
            return {"command": command, "exit_code": result.returncode, "timed_out": False}
        except subprocess.TimeoutExpired:
            return {"command": command, "exit_code": None, "timed_out": True}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-source", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, default=Path("outputs/reports"))
    parser.add_argument("--repetitions", type=int, default=10)
    arguments = parser.parse_args()
    if not 1 <= arguments.repetitions <= 100:
        parser.error("Use 1..100 repetitions")
    repository = Path(__file__).resolve().parents[3]
    fixtures = repository / "src/testing/fixtures/px4_timer"
    patch = repository / "src/integrations/px4/patches/0001-fix-posix-periodic-timer-requeue.patch"
    source = arguments.px4_source.resolve()
    destination = arguments.output_directory.resolve() / f"px4-timer-{time.time_ns()}"
    destination.mkdir(parents=True)
    report = {
        "passed": False,
        "state": "running",
        "upstream_revision": "d6f12ad1c4f70ad3230afd7d86e971421e02fef4",
        "source": str(source),
        "patch_sha256": hashlib.sha256(patch.read_bytes()).hexdigest(),
        "scope": "Real pinned timer dispatcher and queue code; test clock, wakeup and OS shims. No PX4 process or vehicle control.",
        "commands": [],
        "cases": [],
    }
    try:
        for dependency in ("g++", "patch"):
            if shutil.which(dependency) is None:
                raise RuntimeError(f"Required native test dependency is unavailable: {dependency}")
        expected = json.loads((fixtures / "pinned-source-sha256.json").read_text())
        for name, digest in expected.items():
            if hashlib.sha256((source / name).read_bytes()).hexdigest() != digest:
                raise RuntimeError(f"Pinned source does not match: {name}")
        relative_timer = Path("platforms/posix/src/px4/common/drv_hrt.cpp")
        candidate = destination / "candidate"
        (candidate / relative_timer).parent.mkdir(parents=True)
        shutil.copy2(source / relative_timer, candidate / relative_timer)
        command = ["patch", "--batch", "--forward", "-p1", "-d", str(candidate), "-i", str(patch)]
        outcome = execute(command, destination / "apply-patch.log", 10)
        report["commands"].append(outcome)
        if outcome["exit_code"] != 0:
            raise RuntimeError("The candidate patch did not apply to the pinned timer source")
        for version, timer in (
            ("pinned", source / relative_timer),
            ("patched", candidate / relative_timer),
        ):
            binary = destination / version
            queue = source / "platforms/common/work_queue"
            command = [
                "g++",
                "-std=gnu++17",
                "-O2",
                "-pthread",
                "-D__EXPORT=",
                f'-DHRT_SOURCE="{timer}"',
                "-I" + str(fixtures / "stubs"),
                "-I" + str(source / "src"),
                "-I" + str(source / "platforms/posix/include"),
                str(fixtures / "timer_requeue.cpp"),
                *(str(queue / name) for name in ("queue.c", "sq_addafter.c", "sq_addlast.c")),
                "-o",
                str(binary),
            ]
            outcome = execute(command, destination / f"build-{version}.log", 30)
            report["commands"].append(outcome)
            if outcome["exit_code"] != 0:
                raise RuntimeError(f"Native timer regression did not compile: {version}")
            repetitions = 1 if version == "pinned" else arguments.repetitions
            for repetition in range(repetitions):
                for scenario in SCENARIOS:
                    log_path = destination / f"{version}-{repetition}-{scenario}.log"
                    outcome = execute([str(binary), scenario], log_path, 2)
                    report["cases"].append(
                        {
                            "version": version,
                            "scenario": scenario,
                            "repetition": repetition,
                            "passed": outcome["exit_code"] == 0,
                            "log": str(log_path),
                            **outcome,
                        }
                    )
        pinned = [row for row in report["cases"] if row["version"] == "pinned"]
        patched = [row for row in report["cases"] if row["version"] == "patched"]
        controls_passed = all(row["passed"] for row in pinned if row["scenario"] in CONTROLS)
        reproduced = any(not row["passed"] for row in pinned if row["scenario"] == "slow-interval")
        report.update(
            baseline_controls_passed=controls_passed,
            baseline_defect_reproduced=reproduced,
            patched_cases_passed=sum(row["passed"] for row in patched),
            patched_cases_total=len(patched),
            passed=controls_passed and reproduced and all(row["passed"] for row in patched),
            state="complete",
        )
    except (OSError, RuntimeError, ValueError) as error:
        report.update(state="failed", error=str(error))
    write_json_atomic(destination / "report.json", report)
    print(json.dumps({"passed": report["passed"], "report": str(destination / "report.json")}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
