"""Opt-in owning-process supervision; no container engine or host-service access."""

import argparse
from pathlib import Path

from common.lifecycle.children import enable_service_subreaper, retire_service_children
from common.lifecycle.process import run_process

SERVICE_ROLES = frozenset(
    {
        "gazebo",
        "gazebo-viewer",
        "qgroundcontrol",
        "camera-streams",
        "live-mapping",
        "dds-agent",
        "ros2-monitor",
        "ros2-clock",
        "navigation-stereo",
        "visual-odometry-worker",
    }
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=sorted(SERVICE_ROLES), required=True)
    parser.add_argument("--runtime-root", type=Path, default=Path("/runtime/services"))
    parser.add_argument("--request-root", type=Path, default=Path("/runtime/service-requests"))
    parser.add_argument("command", nargs=argparse.REMAINDER)
    arguments = parser.parse_args()
    command = arguments.command
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        parser.error("Supply the owned service command after --")
    directory = arguments.runtime_root / arguments.role
    directory.mkdir(parents=True, exist_ok=True)
    enable_service_subreaper()
    while True:
        result = run_process(
            command,
            restart_file=arguments.request_root / f"{arguments.role}.restart",
            status_file=directory / "status.json",
            after_child_shutdown=retire_service_children,
        )
        if result != 75:
            return result


if __name__ == "__main__":
    raise SystemExit(main())
