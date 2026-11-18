"""Isolated software-rendered/GPU range geometry check; no vehicle command connections."""

import argparse
import json
import math
import os
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from gazebo.navigation_scene import add_navigation_obstacles, add_navigation_sensor


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--renderer", choices=["cpu", "gpu"], default="cpu")
    arguments = parser.parse_args()
    destination = (
        Path("/workspace/outputs/reports")
        / f"avoidance-{arguments.renderer}-sensor-probe-{time.time_ns()}"
    )
    destination.mkdir(parents=True, exist_ok=True)
    os.environ["GZ_PARTITION"] = "navigation-probe-" + uuid.uuid4().hex
    if arguments.renderer == "cpu":
        os.environ.update(
            LIBGL_ALWAYS_SOFTWARE="1",
            MESA_LOADER_DRIVER_OVERRIDE="llvmpipe",
            GALLIUM_DRIVER="llvmpipe",
            __GLX_VENDOR_LIBRARY_NAME="mesa",
        )
    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name="default")
    for system, name in (
        ("physics", "Physics"),
        ("sensors", "Sensors"),
        ("magnetometer", "Magnetometer"),
    ):
        plugin = ET.SubElement(
            world, "plugin", filename=f"gz-sim-{system}-system", name=f"gz::sim::systems::{name}"
        )
        if system == "sensors":
            ET.SubElement(plugin, "render_engine").text = "ogre2"
    model = ET.SubElement(world, "model", name="probe")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "0 0 4 0 0 0"
    ET.SubElement(model, "link", name="base_link")
    add_navigation_sensor(model)
    add_navigation_obstacles(world, Path("configs/navigation/single.json"))
    falling = ET.SubElement(world, "model", name="contact_positive_control")
    ET.SubElement(falling, "pose").text = "0 7 8.6 0 0 0"
    link = ET.SubElement(falling, "link", name="body")
    collision = ET.SubElement(link, "collision", name="collision")
    box = ET.SubElement(ET.SubElement(collision, "geometry"), "box")
    ET.SubElement(box, "size").text = "0.2 0.2 0.2"
    path = destination / "scene.sdf"
    ET.ElementTree(sdf).write(path)
    report = {
        "schema_version": 1,
        "renderer": arguments.renderer,
        "passed": False,
        "partition": os.environ["GZ_PARTITION"],
    }
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.transport13 import Node

    node = Node()
    scans = []
    contact_counts = []
    node.subscribe(
        Contacts,
        "/avoidance/contacts/0",
        lambda message: (
            contact_counts.append(len(message.contact))
            if message.contact and len(contact_counts) < 10
            else None
        ),
    )
    topic = "/world/default/model/probe/link/navigation_scan/sensor/navigation_lidar/scan"
    node.subscribe(LaserScan, topic, lambda scan: scans.append(scan) if len(scans) < 3 else None)
    with (destination / "gazebo.log").open("w") as log:
        process = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "2", str(path)],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 45
            while (
                (len(scans) < 3 or not contact_counts)
                and time.monotonic() < deadline
                and process.poll() is None
            ):
                time.sleep(0.1)
            if not scans:
                raise RuntimeError("Isolated renderer published no measured scan")
            scan = scans[-1]
            index = round((math.pi / 2 - scan.angle_min) / scan.angle_step)
            measured = scan.ranges[index]
            report.update(
                samples=len(scan.ranges),
                expected_range_m=6.0,
                measured_range_m=measured,
                error_m=abs(measured - 6.0),
            )
            report["contact_positive_control_messages"] = len(contact_counts)
            report["passed"] = (
                len(scan.ranges) == 720 and abs(measured - 6.0) < 0.05 and bool(contact_counts)
            )
        except (OSError, ValueError, RuntimeError) as error:
            report["error"] = str(error)
        finally:
            node.unsubscribe(topic)
            node.unsubscribe("/avoidance/contacts/0")
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
    (destination / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
