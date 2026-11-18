"""Run panel tests in an isolated container with no simulator or network."""

import subprocess
from pathlib import Path


def main() -> int:
    source = Path(__file__).resolve().parents[2] / "src"
    return subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "--network",
            "none",
            "--env",
            "QT_QPA_PLATFORM=offscreen",
            "--env",
            "PYTHONPATH=/workspace/src",
            "--mount",
            f"type=bind,source={source},target=/workspace/src,readonly",
            "uav-sim-stack:development",
            "python",
            "-m",
            "unittest",
            "discover",
            "-s",
            "src/testing/component",
            "-v",
        ],
        check=False,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
