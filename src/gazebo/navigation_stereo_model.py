"""Optional calibrated panoramic stereo sensors; independent of downward observation."""

import math
import xml.etree.ElementTree as ET

from common.configuration.navigation_stereo import NavigationStereoCalibration


def add_navigation_stereo(model: ET.Element) -> None:
    """Attach six parallel pinhole pairs at declared FLU mount poses, without collisions."""
    calibration = NavigationStereoCalibration()
    for rig in range(calibration.rig_count):
        yaw = calibration.yaw_rad(rig)
        for side, sign in (("left", 1), ("right", -1)):
            offset = sign * calibration.baseline_m / 2
            name = f"navigation_stereo_{rig}_{side}"
            link = ET.SubElement(model, "link", name=name)
            ET.SubElement(link, "pose").text = (
                f"{-offset * math.sin(yaw)} {offset * math.cos(yaw)} "
                f"{calibration.mount_height_m} 0 0 {yaw}"
            )
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "mass").text = "0.001"
            inertia = ET.SubElement(inertial, "inertia")
            for axis in ("ixx", "iyy", "izz"):
                ET.SubElement(inertia, axis).text = "0.000001"
            joint = ET.SubElement(model, "joint", name=name + "_joint", type="fixed")
            ET.SubElement(joint, "parent").text = "base_link"
            ET.SubElement(joint, "child").text = name
            sensor = ET.SubElement(link, "sensor", name="camera", type="camera")
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(sensor, "update_rate").text = str(calibration.rate_hz)
            camera = ET.SubElement(sensor, "camera")
            ET.SubElement(camera, "horizontal_fov").text = str(calibration.horizontal_fov_rad)
            image = ET.SubElement(camera, "image")
            for key, value in (
                ("width", calibration.width),
                ("height", calibration.height),
                ("format", "R8G8B8"),
            ):
                ET.SubElement(image, key).text = str(value)
            clip = ET.SubElement(camera, "clip")
            ET.SubElement(clip, "near").text = "0.1"
            ET.SubElement(clip, "far").text = "30"
