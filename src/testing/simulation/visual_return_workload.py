"""Move an isolated camera fixture through the return scene without flying PX4."""

import argparse
import math
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

from common.logging.storage import write_json_atomic
from gazebo.control_worker import execute_control, send_request

RIG_NAME = "visual_return_rig"
START = (0.0, 0.0, 0.1)
LAUNCH = (0.0, 0.0, 4.0)
ROUTE = (LAUNCH, (3.5, 1.4, 4.0), (3.5, 7.7, 4.0), (0.0, 9.0, 4.0))


def main() -> int:
    """Hold at launch, then traverse the visual workload in simulation time."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeline", required=True, type=Path)
    parser.add_argument("--return-only", action="store_true", help="Capture a short return clip")
    args = parser.parse_args()
    if os.environ.get("GZ_PARTITION") != "uav-visual-odometry" or args.timeline.exists():
        parser.error("Use the isolated project and a new timeline")
    camera_model = ET.parse("/runtime/stereo-models/x500/model.sdf").find("model")
    model = ET.Element("model", name=RIG_NAME)
    launch_position = ROUTE[-1] if args.return_only else START
    ET.SubElement(model, "pose").text = " ".join(str(v) for v in launch_position) + " 0 0 0"
    base = ET.SubElement(model, "link", name="base_link")
    ET.SubElement(base, "gravity").text = "false"
    for side in ("left", "right"):
        link = camera_model.find(f"link[@name='stereo_{side}']")
        if link is None or link.find("collision") is not None:
            raise RuntimeError("Expected a camera-only link from the configured X500")
        ET.SubElement(link, "gravity").text = "false"
        model.append(link)
        joint = ET.SubElement(model, "joint", name=f"camera_{side}", type="fixed")
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = f"stereo_{side}"
    velocity = ET.SubElement(
        model,
        "plugin",
        filename="gz-sim-velocity-control-system",
        name="gz::sim::systems::VelocityControl",
    )
    command_topic = f"/model/{RIG_NAME}/cmd_vel"
    ET.SubElement(velocity, "topic").text = command_topic
    sdf = ET.Element("sdf", version="1.9")
    sdf.append(model)
    sim_now_ns = -1

    def clock(message: Clock) -> None:
        nonlocal sim_now_ns
        sim_now_ns = message.sim.sec * 1_000_000_000 + message.sim.nsec

    subscriber = Node()
    commands = Node()
    send_request(
        commands,
        "/world/default/create",
        EntityFactory(
            sdf=ET.tostring(sdf, encoding="unicode"),
            allow_renaming=False,
        ),
    )
    try:
        if not subscriber.subscribe(Clock, "/world/default/clock", clock):
            raise RuntimeError("Cannot read the isolated simulation clock")
        publisher = commands.advertise(command_topic, Twist)
        deadline = time.monotonic() + 310
        while sim_now_ns < 0:
            if time.monotonic() >= deadline:
                raise TimeoutError("No isolated simulation clock")
            time.sleep(0.02)
        motion_start_ns = sim_now_ns + 8_000_000_000
        legs = []
        cursor = motion_start_ns
        routes = (
            ((0.34, ROUTE[-1:-3:-1]),)
            if args.return_only
            else ((0.5, (START, *ROUTE)), (0.34, ROUTE[::-1]))
        )
        for speed, path in routes:
            for first, last in zip(path, path[1:]):
                duration_ns = round(math.dist(first, last) / speed * 1e9)
                legs.append((cursor, cursor + duration_ns, first, last))
                cursor += duration_ns
        write_json_atomic(
            args.timeline,
            {
                "schema_version": 1,
                "rig": RIG_NAME,
                "motion_start_ns": motion_start_ns,
                "return_start_ns": legs[0 if args.return_only else 4][0],
                "return_end_ns": cursor,
                "scripted_route_enu_m": ROUTE,
                "scope": "Camera fixture motion only; no simulated vehicle is moved",
            },
        )
        while sim_now_ns < motion_start_ns:
            if time.monotonic() >= deadline:
                raise TimeoutError("Scripted camera launch clock stalled")
            time.sleep(0.02)
        for start_ns, end_ns, first, last in legs:
            duration_seconds = (end_ns - start_ns) / 1e9
            command = Twist()
            command.linear.x, command.linear.y, command.linear.z = (
                (b - a) / duration_seconds for a, b in zip(first, last)
            )
            while sim_now_ns < end_ns:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Scripted camera motion exceeded its wall deadline")
                if not publisher.publish(command):
                    raise RuntimeError("Could not publish scripted fixture velocity")
                time.sleep(0.04)
        publisher.publish(Twist())
        time.sleep(1)
    finally:
        subscriber.unsubscribe("/world/default/clock")
        execute_control(commands, "remove_model", {"model_name": RIG_NAME})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
