"""Run isolated recorded mapping and render its read-only result."""

import argparse
import subprocess
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--build", action="store_true")
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    source = arguments.source.resolve()
    if not source.is_file():
        parser.error("Provide an existing stereo recording report")
    destination = f"outputs/reports/mapping-{time.time_ns()}"
    (root / "outputs/reports").mkdir(parents=True, exist_ok=True)
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
        "--mount",
        f"type=bind,source={root / 'src'},target=/workspace/src,readonly",
        "--mount",
        f"type=bind,source={root / 'scripts'},target=/workspace/scripts,readonly",
        "--mount",
        f"type=bind,source={root / 'outputs'},target=/workspace/outputs",
        "--mount",
        f"type=bind,source={source.parent},target=/input,readonly",
    ]
    commands = []
    if arguments.build:
        commands.append(
            [
                "docker",
                "build",
                "-f",
                "src/integrations/vision/Dockerfile",
                "--target",
                "vision_dependencies",
                "-t",
                "uav-sim-vision-base:development",
                ".",
            ]
        )
        commands.append(
            [
                "docker",
                "build",
                "-f",
                "src/integrations/vision/Dockerfile.slam",
                "-t",
                "uav-sim-mapping:development",
                ".",
            ]
        )
    commands.extend(
        [
            [
                *runtime,
                "uav-sim-mapping:development",
                "python",
                "-m",
                "integrations.vision.mapping",
                f"/input/{source.name}",
                destination,
            ],
            [
                *runtime,
                "uav-sim-stack:development",
                "python",
                "scripts/dev/render_mapping.py",
                f"{destination}/report.json",
                f"{destination}/preview.png",
            ],
        ]
    )
    for command in commands:
        try:
            result = subprocess.run(command, cwd=root, check=False, timeout=1800)
        except (OSError, subprocess.TimeoutExpired) as error:
            print(f"Recorded mapping could not complete: {error}")
            return 1
        if result.returncode:
            print(f"Recorded mapping failed; inspect {root / destination}")
            return result.returncode
    print(f"Recorded mapping evidence: {root / destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
