"""Prepare portable cached terrain from a downloaded provincial DEM."""

import argparse
import subprocess
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--build", action="store_true")
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    source = arguments.source.resolve()
    if not source.is_file():
        parser.error("Provide the extracted provincial DEM GeoTIFF")
    if arguments.build:
        result = subprocess.run(
            [
                "docker",
                "build",
                "-f",
                "src/integrations/vision/Dockerfile.terrain",
                "-t",
                "uav-sim-terrain:development",
                ".",
            ],
            cwd=root,
            check=False,
            timeout=900,
        )
        if result.returncode:
            return result.returncode
    return subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "--network",
            "none",
            "--env",
            "PYTHONPATH=/workspace/src",
            "--mount",
            f"type=bind,source={root / 'src'},target=/workspace/src,readonly",
            "--mount",
            f"type=bind,source={root / 'configs'},target=/workspace/configs,readonly",
            "--mount",
            f"type=bind,source={root / 'outputs'},target=/workspace/outputs",
            "--mount",
            f"type=bind,source={source.parent},target=/input,readonly",
            "uav-sim-terrain:development",
            "python",
            "-m",
            "integrations.vision.terrain",
            f"/input/{source.name}",
            "outputs/terrain/waterfront",
        ],
        cwd=root,
        check=False,
        timeout=180,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
