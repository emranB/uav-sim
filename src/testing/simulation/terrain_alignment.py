"""Measure terrain collision/render alignment in an isolated depth-camera world."""

import argparse
import copy
import json
import math
import os
import signal
import statistics
import struct
import subprocess
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from gazebo.terrain import add_terrain


def write_scene(destination: Path, terrain: Path, legacy_visual: bool) -> None:
    """Place invisible physical probes and downward cameras at the two camera footprints."""
    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name="terrain_alignment")
    physics = ET.SubElement(world, "physics", name="default", type="ignored")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1"
    for filename, name in [
        ("physics", "Physics"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("sensors", "Sensors"),
    ]:
        plugin = ET.SubElement(
            world, "plugin", filename=f"gz-sim-{filename}-system", name=f"gz::sim::systems::{name}"
        )
        if name == "Sensors":
            ET.SubElement(plugin, "render_engine").text = "ogre2"
    scene = ET.SubElement(world, "scene")
    ET.SubElement(scene, "ambient").text = "0.6 0.6 0.6 1"
    light = ET.SubElement(world, "light", type="directional", name="probe_sun")
    ET.SubElement(light, "direction").text = "0.2 0.1 -1"
    ET.SubElement(light, "diffuse").text = "1 1 1 1"
    add_terrain(world, terrain)
    if legacy_visual:
        visual = world.find("model/link/visual")
        visual.remove(visual.find("geometry"))
        visual.remove(visual.find("material"))
        geometry = ET.SubElement(visual, "geometry")
        heightmap = copy.deepcopy(world.find("model/link/collision/geometry/heightmap"))
        geometry.append(heightmap)
        texture = ET.SubElement(heightmap, "texture")
        ET.SubElement(texture, "size").text = "20"
        for key, name in [("diffuse", "texture.png"), ("normal", "normal.png")]:
            ET.SubElement(texture, key).text = (terrain / name).resolve().as_uri()
    for index, x in enumerate([0.4, 6.4]):
        model = ET.SubElement(world, "model", name=f"physical_probe_{index}")
        ET.SubElement(model, "pose").text = f"{x} 0.2 10 0 0 0"
        link = ET.SubElement(model, "link", name="body")
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass").text = "1"
        inertia = ET.SubElement(inertial, "inertia")
        for key in ["ixx", "iyy", "izz"]:
            ET.SubElement(inertia, key).text = "0.001"
        geometry = ET.SubElement(ET.SubElement(link, "collision", name="probe"), "geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = "0.1 0.1 0.1"
        rig = ET.SubElement(world, "model", name=f"depth_rig_{index}")
        ET.SubElement(rig, "static").text = "true"
        ET.SubElement(rig, "pose").text = f"{x} 0.2 10 0 0 0"
        sensor = ET.SubElement(
            ET.SubElement(rig, "link", name="camera"), "sensor", name="depth", type="depth_camera"
        )
        ET.SubElement(sensor, "pose").text = f"0 0 0 0 {math.pi / 2} 0"
        ET.SubElement(sensor, "topic").text = f"/terrain_alignment/depth_{index}"
        ET.SubElement(sensor, "update_rate").text = "10"
        ET.SubElement(sensor, "always_on").text = "true"
        camera = ET.SubElement(sensor, "camera")
        ET.SubElement(camera, "horizontal_fov").text = "0.5"
        image = ET.SubElement(camera, "image")
        ET.SubElement(image, "width").text = "64"
        ET.SubElement(image, "height").text = "64"
        clip = ET.SubElement(camera, "clip")
        ET.SubElement(clip, "near").text = "0.1"
        ET.SubElement(clip, "far").text = "30"
        rgb = copy.deepcopy(sensor)
        rgb.set("name", "rgb")
        rgb.set("type", "camera")
        rgb.find("topic").text = f"/terrain_alignment/rgb_{index}"
        ET.SubElement(rgb.find("camera/image"), "format").text = "R8G8B8"
        rig.find("link").append(rgb)
    ET.ElementTree(sdf).write(destination, encoding="utf-8", xml_declaration=True)


def run(directory: Path, terrain: Path, legacy_visual: bool) -> dict:
    """Compare settled physical ground height and centre optical depth, in ENU metres."""
    os.environ["GZ_PARTITION"] = "terrain-alignment-" + uuid.uuid4().hex
    os.environ["GZ_IP"] = "127.0.0.1"
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    lock = threading.Lock()
    positions = {0: [], 1: []}
    depths = {}
    rgb_samples = {}
    errors = []

    def pose_received(message: Pose_V) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nsec / 1e9
        with lock:
            for pose in message.pose:
                for index in positions:
                    if pose.name == f"physical_probe_{index}":
                        positions[index].append((stamp, pose.position.z))
                        positions[index] = positions[index][-10:]

    def depth_received(index: int, message: Image) -> None:
        if message.width != 64 or message.height != 64 or message.step != 256:
            with lock:
                errors.append("Depth camera requires 64x64 float32 output")
            return
        values = [
            struct.unpack_from("<f", message.data, (row * 64 + column) * 4)[0]
            for row in [31, 32, 33]
            for column in [31, 32, 33]
        ]
        values = [value for value in values if math.isfinite(value)]
        if values:
            with lock:
                depths[index] = sorted(values)[len(values) // 2]

    def rgb_received(index: int, message: Image) -> None:
        if message.width != 64 or message.height != 64 or message.step != 192:
            return
        pixels = [
            sum(message.data[offset : offset + 3]) / 3 for offset in range(0, len(message.data), 3)
        ]
        with lock:
            rgb_samples[index] = dict(
                mean=statistics.mean(pixels),
                stddev=statistics.pstdev(pixels),
                nonblack_fraction=sum(value > 5 for value in pixels) / len(pixels),
                pixels=message.data,
            )

    node = Node()
    topics = ["/world/terrain_alignment/pose/info"]
    if not node.subscribe(Pose_V, topics[0], pose_received):
        raise RuntimeError("Cannot observe probe poses")
    from functools import partial

    for index in positions:
        topic = f"/terrain_alignment/depth_{index}"
        if not node.subscribe(Image, topic, partial(depth_received, index)):
            raise RuntimeError("Cannot observe probe depth")
        topics.append(topic)
        rgb_topic = f"/terrain_alignment/rgb_{index}"
        if not node.subscribe(Image, rgb_topic, partial(rgb_received, index)):
            raise RuntimeError("Cannot observe probe RGB appearance")
        topics.append(rgb_topic)
    scene = directory / "scene.sdf"
    write_scene(scene, terrain, legacy_visual)
    started = time.monotonic()
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix="terrain-probe-home-") as home:
            environment = {**os.environ, "HOME": home, "LIBGL_ALWAYS_SOFTWARE": "false"}
            with (directory / "gazebo.log").open("w") as log:
                process = subprocess.Popen(
                    ["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "2", str(scene)],
                    env=environment,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                while time.monotonic() - started < 60:
                    if process.poll() is not None:
                        raise RuntimeError("Isolated terrain server exited")
                    with lock:
                        ready = (
                            all(
                                len(p) >= 5 and p[-1][0] > 4 and abs(p[-1][1] - p[-5][1]) < 0.0005
                                for p in positions.values()
                            )
                            and len(depths) == 2
                            and len(rgb_samples) == 2
                        )
                    if ready:
                        break
                    time.sleep(0.1)
                else:
                    raise TimeoutError(
                        f"Terrain probe data incomplete: {positions}, {depths}, {errors}"
                    )
                with lock:
                    cases = [
                        dict(
                            case=index,
                            physical_surface_height_m=positions[index][-1][1] - 0.05,
                            rendered_surface_height_m=10 - depths[index],
                            absolute_height_error_m=abs(
                                positions[index][-1][1] - 0.05 - (10 - depths[index])
                            ),
                        )
                        for index in positions
                    ]
                for case in cases:
                    rgb = rgb_samples[case["case"]]
                    case.update(
                        rgb_mean=rgb["mean"],
                        rgb_stddev=rgb["stddev"],
                        rgb_nonblack_fraction=rgb["nonblack_fraction"],
                    )
                    (directory / f"rgb-{case['case']}.ppm").write_bytes(
                        b"P6\n64 64\n255\n" + rgb["pixels"]
                    )
                return dict(
                    schema_version=1,
                    legacy_visual=legacy_visual,
                    cases=cases,
                    passed=all(
                        case["absolute_height_error_m"] < 0.1
                        and case["rgb_mean"] > 10
                        and case["rgb_stddev"] > 5
                        and case["rgb_nonblack_fraction"] > 0.8
                        for case in cases
                    ),
                    elapsed_wall_seconds=time.monotonic() - started,
                    partition=os.environ["GZ_PARTITION"],
                    scope="Isolated physics/depth cameras; no vehicle or live-world control",
                )
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
        for topic in topics:
            node.unsubscribe(topic)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--terrain", type=Path, default=Path("outputs/terrain/waterfront"))
    parser.add_argument("--legacy-visual", action="store_true")
    args = parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=False)
    try:
        report = run(args.directory, args.terrain, args.legacy_visual)
    except (OSError, ValueError, RuntimeError, TimeoutError) as error:
        report = dict(passed=False, error=str(error))
    (args.directory / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
