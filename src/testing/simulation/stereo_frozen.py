"""Predeclare and retain one fixed-source twenty-flight repeatability campaign."""

import argparse
import hashlib
import json
import subprocess
import time
import urllib.request
from pathlib import Path

from common.logging.storage import write_json_atomic
from testing.simulation.stereo_acceptance import wilson


def fingerprints() -> dict[str, str]:
    """Hash application, test, declarative and deployment sources before and after the run."""
    paths = [
        *Path("src").rglob("*.py"),
        *Path("configs").rglob("*.json"),
        *Path(".").glob("compose*.yaml"),
        *Path("deploy/compose").glob("*.yaml"),
        Path("pyproject.toml"),
    ]
    return {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(paths)}


def deployment(backend: str = "cpu") -> dict:
    """Record effective service settings, allowing only the predeclared scene to change."""
    resolved = json.loads(
        subprocess.check_output(["docker", "compose", "config", "--format", "json"], text=True)
    )
    services = {}
    for name, service in resolved["services"].items():
        environment = dict(service.get("environment", {}))
        environment.pop("UAV_AVOIDANCE_SCENARIO", None)
        services[name] = {
            "environment_sha256": hashlib.sha256(
                json.dumps(environment, sort_keys=True).encode()
            ).hexdigest(),
            "image": service.get("image"),
            "cpus": service.get("cpus"),
            "mem_limit": service.get("mem_limit"),
        }
    assert (
        resolved["services"]["navigation-stereo"]["environment"]["UAV_NAV_STEREO_BACKEND"]
        == backend
    ), "Frozen deliverable requires the declared processing backend"
    containers = subprocess.check_output(["docker", "compose", "ps", "-q"], text=True).split()
    images = json.loads(subprocess.check_output(["docker", "inspect", *containers], text=True))
    return {"services": services, "image_ids": {item["Name"]: item["Image"] for item in images}}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reuse-checkpoint", type=Path)
    parser.add_argument("--backend", choices=["cpu", "vpi-cuda"], default="cpu")
    parser.add_argument("--resume-matrix", action="append", type=Path, default=[])
    arguments = parser.parse_args()
    root = Path("outputs/reports")
    seed_path = None
    if arguments.reuse_checkpoint:
        checkpoint = json.loads(arguments.reuse_checkpoint.read_text())
        benchmark = json.loads(Path(checkpoint["selected_profile"]).read_text())
        assert benchmark["passed"] and benchmark["requested_real_time_factor"] == 1.0
        assert all(
            hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
            for path, digest in benchmark["source_sha256"].items()
        ), "Application changed since the passing full-speed pair"
        current = deployment(arguments.backend)
        for service, selected in checkpoint["deployed_services"].items():
            assert selected["image_id"] in current["image_ids"].values()
            assert float(current["services"][service]["cpus"]) == selected["cpu_ceiling"]
        effective = json.loads(
            subprocess.check_output(["docker", "compose", "config", "--format", "json"], text=True)
        )
        assert float(effective["services"]["gazebo"]["environment"]["UAV_REAL_TIME_FACTOR"]) == 1.0
        cases = []
        for case in benchmark["cases"]:
            path = Path(case["report"])
            assert (
                hashlib.sha256(path.read_bytes()).hexdigest()
                == checkpoint["artifact_sha256"][str(path)]
            )
            cases.append(
                {
                    "name": "single-01",
                    "mode": case["mode"],
                    "held_out": False,
                    "status": "pass",
                    "report": str(path),
                    "source_revision": benchmark["revision"],
                    "reused": True,
                    "report_sha256": checkpoint["artifact_sha256"][str(path)],
                }
            )
        seed_path = root / f"stereo-frozen-seed-{time.time_ns()}.json"
        write_json_atomic(
            seed_path,
            {
                "revision": benchmark["revision"],
                "cases": cases,
                "checkpoint": str(arguments.reuse_checkpoint),
                "reuse_scope": "Identical application hashes, images, CPU ceilings, actual Gazebo factor, scene/quality/calibration. Backend factor metadata was synchronized after the benchmark; only the Gazebo launcher consumes this setting for physics.",
            },
        )
    destination = root / f"stereo-frozen-{time.time_ns()}.json"
    log_path = destination.with_suffix(".log")
    resources_path = destination.with_suffix(".resources.jsonl")
    report = {
        "schema_version": 1,
        "passed": False,
        "state": "predeclared",
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "source_sha256": fingerprints(),
        "deployment": deployment(arguments.backend),
        "processing_backend": arguments.backend,
        "started_wall_time": time.time(),
        "planned_trials_per_mode": 10,
        "completion_target": 0.9,
        "reused_trials": 2 if seed_path else 0,
        "reuse_seed": str(seed_path) if seed_path else None,
        "resume_matrices": [str(path) for path in arguments.resume_matrix],
        "denominator_policy": "Every completed declared slot, including diagnosed failures, remains counted once. Only unrun slots execute; no replacement.",
        "accepted_application_revision": "9e49da8" if seed_path else None,
        "budget_reason": "Use the existing ten declared configurations per mode once, with no replacement. Reuse only the explicitly verified full-speed pair when supplied; half-speed and earlier development profiles remain separate. This is the predeclared minimum mode-specific budget, not a power calculation.",
        "criteria": "Original nominal physical and perception criteria, unchanged",
        "historical_first_exposure": {"camera": "8/10", "fused": "8/10"},
        "interpretation": "Known-scene fixed-source repeatability; no new held-out claim or universal reliability bound. Unspecified PX4/Gazebo noise seeds remain unspecified.",
        "stop_rule": "Stop at the first failed flight; retain the failure and all unrun slots. No retries in this campaign.",
        "reporting_interval_seconds": 900,
        "log": str(log_path),
        "resources": str(resources_path),
    }

    def save() -> None:
        report["updated_wall_time"] = time.time()
        write_json_atomic(destination, report)
        write_json_atomic(
            root / "stereo-frozen-current-status.json",
            {
                key: value
                for key, value in report.items()
                if key not in {"source_sha256", "deployment"}
            },
        )

    save()
    try:
        with log_path.open("w") as log, resources_path.open("w") as resources:
            child = subprocess.Popen(
                ["python", "-m", "testing.simulation.stereo_acceptance", "--batch", "all"]
                + (["--resume-matrix", str(seed_path)] if seed_path else [])
                + [
                    value
                    for path in arguments.resume_matrix
                    for value in ["--resume-matrix", str(path)]
                ]
                + (["--retain-failed"] if arguments.resume_matrix else []),
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            deadline = time.monotonic() + 7200
            last_notice = time.monotonic()
            try:
                report["state"] = "running"
                save()
                while child.poll() is None:
                    if time.monotonic() > deadline:
                        raise TimeoutError("Frozen campaign exceeded its two-hour wall budget")
                    sample = {"wall_time": time.time()}
                    try:
                        with urllib.request.urlopen("http://127.0.0.1:8090", timeout=4) as response:
                            sample["space_apps_http_status"] = response.status
                        sample["available_memory_kib"] = int(
                            next(
                                line.split()[1]
                                for line in Path("/proc/meminfo").read_text().splitlines()
                                if line.startswith("MemAvailable:")
                            )
                        )
                    except (OSError, ValueError) as error:
                        sample["error"] = str(error)
                    resources.write(json.dumps(sample) + "\n")
                    resources.flush()
                    if time.monotonic() - last_notice >= 900:
                        print(
                            json.dumps({"state": "running", "report": str(destination)}), flush=True
                        )
                        last_notice = time.monotonic()
                    time.sleep(15)
            finally:
                if child.poll() is None:
                    child.terminate()
                    child.wait(timeout=15)
            report["returncode"] = child.returncode
        summary = json.loads(log_path.read_text().strip().splitlines()[-1])
        matrix_path = Path(summary["report"])
        matrix = json.loads(matrix_path.read_text())
        report["matrix"] = str(matrix_path)
        report["source_unchanged"] = fingerprints() == report["source_sha256"]
        report["deployment_unchanged"] = deployment(arguments.backend) == report["deployment"]
        report["rates"] = matrix["rates"]
        report["failed_trials"] = [case for case in matrix["cases"] if case["status"] != "pass"]
        report["not_run"] = 20 - len(matrix["cases"])
        for mode, rates in report["rates"].items():
            rates["wilson_95"] = wilson(rates["successes"], rates["attempts"])
        report["passed"] = (
            matrix["passed"]
            and report["source_unchanged"]
            and report["deployment_unchanged"]
            and report["not_run"] == 0
            and all(rates["rate"] >= 0.9 for rates in report["rates"].values())
        )
        report["state"] = "passed" if report["passed"] else "needs-diagnosis"
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        report.update(state="blocked", error=str(error))
    finally:
        save()
    print(json.dumps({"passed": report["passed"], "report": str(destination)}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
