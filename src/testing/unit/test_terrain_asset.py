"""Terrain loading keeps collision and display scales together and fails explicitly."""

import hashlib
import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from gazebo.terrain import add_terrain


class TerrainAssetTests(unittest.TestCase):
    def test_prepared_metadata_drives_collision_and_visual_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            metadata = {
                "schema_version": 1,
                "offset_enu_m": [1, 2, -3],
                "width_m": 500,
                "height_m": 500,
                "minimum_elevation_m": -1,
                "maximum_elevation_m": 29,
                "grid_yaw_rad": 0.01,
            }
            (directory / "terrain.json").write_text(json.dumps(metadata), encoding="utf-8")
            world = ET.fromstring('<world><model name="ground_plane"/></world>')
            with self.assertRaisesRegex(ValueError, "missing heightmap"):
                add_terrain(world, directory)
            for filename in ("heightmap.png", "texture.png", "normal.png"):
                (directory / filename).write_bytes(b"fixture, not a rendered asset")
            mesh = directory / "terrain.obj"
            mesh.write_text("o fixture\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
            manifest = dict(
                schema_version=1,
                width_m=500,
                height_m=500,
                relief_m=30,
                source_heightmap_sha256=hashlib.sha256(
                    (directory / "heightmap.png").read_bytes()
                ).hexdigest(),
                mesh_sha256=hashlib.sha256(mesh.read_bytes()).hexdigest(),
            )
            (directory / "terrain-visual.json").write_text(json.dumps(manifest))
            add_terrain(world, directory)
            self.assertIsNone(world.find("model[@name='ground_plane']"))
            collision = world.find("model/link/collision/geometry/heightmap")
            visual = world.find("model/link/visual/geometry/mesh")
            self.assertEqual(collision.findtext("size"), "500 500 30")
            self.assertEqual(world.findtext("model/pose"), "1 2 -3 0 0 0.01")
            self.assertEqual(collision.findtext("pos"), "0 0 0")
            self.assertEqual(visual.findtext("uri"), mesh.resolve().as_uri())
            self.assertEqual(world.findtext("model/link/visual/material/pbr/metal/metalness"), "0")
            self.assertEqual(world.findtext("model/link/visual/material/diffuse"), "1 1 1 1")
            mesh.write_text("changed mesh")
            unchanged = ET.fromstring('<world><model name="ground_plane"/></world>')
            with self.assertRaisesRegex(ValueError, "does not match"):
                add_terrain(unchanged, directory)
            self.assertIsNotNone(unchanged.find("model[@name='ground_plane']"))
