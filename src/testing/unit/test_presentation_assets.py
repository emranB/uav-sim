"""Bounded visual assets preserve the reference's physical and geographic contracts."""

import copy
import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from gazebo.presentation_assets import add_pad_details


class PresentationAssetTests(unittest.TestCase):
    def test_visual_details_preserve_every_existing_element_and_have_no_collisions(self) -> None:
        path = Path("src/gazebo/models/flight_reference/model.sdf")
        reference = ET.parse(path).find("model")
        before = copy.deepcopy(reference)
        add_pad_details(reference)
        self.assertEqual(reference.findall(".//collision"), [])
        added = reference.findall("link/visual")[-2:]
        self.assertEqual(len(added), 2)
        for visual in added:
            self.assertTrue(visual.get("name").startswith("presentation_"))
            self.assertEqual(visual.findtext("cast_shadows"), "false")
            reference.find("link").remove(visual)
        self.assertEqual(ET.tostring(reference), ET.tostring(before))

    def test_modified_mesh_rejected_before_use(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "changed.obj").write_text("v 0 0 0\n")
            (root / "manifest.json").write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "coordinate_frame": "ENU metres; Z up",
                        "collision_geometry": False,
                        "assets": [{"file": "changed.obj", "triangles": 1, "sha256": "wrong"}],
                    }
                )
            )
            reference = ET.fromstring('<model name="reference"><link name="visuals"/></model>')
            with self.assertRaisesRegex(ValueError, "hash changed"):
                add_pad_details(reference, root)
