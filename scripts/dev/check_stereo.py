"""Run isolated CPU camera checks and optional depth processing from any host."""

import argparse
import subprocess
import time
import uuid
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--depth", action="store_true", help="Build the optional depth image")
    parser.add_argument(
        "--mapping-sequence", action="store_true", help="Capture a moving test camera"
    )
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    (root / "outputs/reports").mkdir(parents=True, exist_ok=True)
    run_name = f"stereo-probe-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
    destination = f"outputs/reports/{run_name}"
    runtime = [
        "docker",
        "run",
        "--rm",
        "--network",
        "none",
        "--env",
        "PYTHONPATH=/workspace/src",
        "--env",
        "QT_QPA_PLATFORM=offscreen",
    ]
    for directory in ("src", "scripts", "outputs"):
        readonly = ",readonly" if directory != "outputs" else ""
        runtime.extend(
            [
                "--mount",
                f"type=bind,source={root / directory},target=/workspace/{directory}{readonly}",
            ]
        )
    commands = [
        [
            *runtime,
            "uav-sim-stack:development",
            "python",
            "-m",
            "gazebo.stereo_probe",
            "--output",
            f"/workspace/{destination}",
        ],
        [
            *runtime,
            "uav-sim-stack:development",
            "python",
            "scripts/dev/render_stereo_recording.py",
            f"{destination}/report.json",
            f"{destination}/playback.png",
        ],
    ]
    if arguments.mapping_sequence:
        commands[0].extend(["--mapping-sequence", "--pairs", "60", "--timeout-seconds", "90"])
    if arguments.depth:
        commands.extend(
            [
                [
                    "docker",
                    "build",
                    "-f",
                    "src/integrations/vision/Dockerfile",
                    "-t",
                    "uav-sim-perception:development",
                    ".",
                ],
                [
                    *runtime,
                    "uav-sim-perception:development",
                    "python",
                    "-m",
                    "integrations.vision.stereo_depth",
                    f"{destination}/report.json",
                    f"{destination}/depth",
                ],
            ]
        )
    for command in commands:
        try:
            result = subprocess.run(command, cwd=root, check=False, timeout=900)
        except (OSError, subprocess.TimeoutExpired) as error:
            print(f"Stereo check could not complete: {error}")
            return 1
        if result.returncode:
            print(f"Stereo check failed; inspect {root / destination}")
            return result.returncode
    print(f"Stereo evidence: {root / destination}")
    print("Scope: independent stationary cameras; no vehicle vision or recovery integration.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
