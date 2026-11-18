"""Optional observational stereo sensors for the stock X500 model."""

import os
import shutil
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from common.configuration.cameras import StereoConfiguration
from gazebo.navigation_scene import add_navigation_sensor
from gazebo.navigation_stereo_model import add_navigation_stereo


def prepare_stereo_model(source: Path, directory: Path) -> Path:
    """Preserve stock motor settings and attach two one-gram fixed camera links."""
    tree = ET.parse(source)
    maximum_rate_hz = 50 if os.environ.get("UAV_VISUAL_ODOMETRY_PROBE") == "true" else 10
    configuration = StereoConfiguration.configured(maximum_rate_hz=maximum_rate_hz)
    model = tree.find("model")
    if model is None:
        raise ValueError("X500 source has no model")
    for side, y_m in (
        ("left", configuration.baseline_m / 2),
        ("right", -configuration.baseline_m / 2),
    ):
        link_name = f"stereo_{side}"
        link = ET.SubElement(model, "link", name=link_name)
        # A shared downward pitch exposes nearby terrain while retaining parallel optics.
        ET.SubElement(link, "pose").text = (
            f"{configuration.mount_forward_m} {y_m} {configuration.mount_height_m} "
            f"0 {configuration.mount_pitch_rad} 0"
        )
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass").text = "0.001"
        inertia = ET.SubElement(inertial, "inertia")
        for key in ("ixx", "iyy", "izz"):
            ET.SubElement(inertia, key).text = "0.000001"
        sensor = ET.SubElement(link, "sensor", name="camera", type="camera")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = str(configuration.rate_hz)
        camera = ET.SubElement(sensor, "camera")
        ET.SubElement(camera, "horizontal_fov").text = str(configuration.horizontal_fov_rad)
        image = ET.SubElement(camera, "image")
        for key, value in (
            ("width", str(configuration.width)),
            ("height", str(configuration.height)),
            ("format", "R8G8B8"),
        ):
            ET.SubElement(image, key).text = value
        clip = ET.SubElement(camera, "clip")
        ET.SubElement(clip, "near").text = "0.1"
        ET.SubElement(clip, "far").text = "80"
        joint = ET.SubElement(model, "joint", name=f"{link_name}_joint", type="fixed")
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = link_name
    if os.environ.get("UAV_NAVIGATION", "false") == "true":
        add_navigation_sensor(model)
        if os.environ.get("UAV_NAVIGATION_STEREO", "false") == "true":
            add_navigation_stereo(model)
        base = source.parent.parent / "x500_base"
        copied_base = directory / "x500_base"
        shutil.copytree(base, copied_base, dirs_exist_ok=True)
        base_tree = ET.parse(copied_base / "model.sdf")
        magnetic = base_tree.find(".//sensor[@type='magnetometer']")
        if magnetic is None:
            raise ValueError("Navigation requires the stock magnetometer sensor")
        magnetic.set("name", "navigation_magnetometer_raw")
        base_tree.write(copied_base / "model.sdf", encoding="utf-8", xml_declaration=True)
    elif (directory / "x500_base").exists():
        shutil.rmtree(directory / "x500_base")
    destination = directory / "x500/model.sdf"
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f"model-{uuid.uuid4().hex}.tmp")
    try:
        tree.write(temporary, encoding="utf-8", xml_declaration=True)
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination
