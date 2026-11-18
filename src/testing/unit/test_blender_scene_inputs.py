"""Offline visuals require the recorded native revision and use past observations only."""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from integrations.blender.scene_inputs import retain_aircraft_assets, write_scene_annotations


class SceneInputTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.export = self.root / "export"
        self.export.mkdir()
        self.models = self.root / "models"
        (self.models / "x500_base").mkdir(parents=True)
        (self.models / "x500_base/model.sdf").write_text("retained model fixture")
        self.revision = self.root / "models-revision"
        self.revision.write_text("model-2\n")
        (self.export / "context.json").write_text(
            json.dumps(
                {
                    "vehicles": {
                        "atlas": {
                            "configuration": {"model": "x500"},
                            "native_revisions": {"models-revision": "model-1"},
                        }
                    }
                }
            )
        )

    def test_model_revision_mismatch_rejects_before_asset_copy(self) -> None:
        with self.assertRaisesRegex(ValueError, "do not match"):
            retain_aircraft_assets(self.export, self.models, self.revision)
        self.assertFalse((self.export / "scene-assets").exists())

    def test_matched_assets_are_copied_with_hashes_and_without_thumbnails(self) -> None:
        self.revision.write_text("model-1\n")
        thumbs = self.models / "x500_base/thumbnails"
        thumbs.mkdir()
        (thumbs / "large.png").write_bytes(b"not a render input")
        output = retain_aircraft_assets(self.export, self.models, self.revision)
        manifest = json.loads((output / "asset-manifest.json").read_text())
        self.assertEqual(manifest["models_revision"], "model-1")
        self.assertIn("x500_base/model.sdf", manifest["files_sha256"])
        self.assertFalse((output / "x500_base/thumbnails").exists())

    def test_labels_remain_unknown_until_an_observation_and_escape_ass_commands(self) -> None:
        sample = SimpleNamespace(
            elapsed_wall_seconds=0.1,
            observation={
                "mission": {"state": "navigating", "leg": "outbound"},
                "aiding": {
                    "gps_requested_enabled": False,
                    "gnss_xy_active": False,
                    "vision_xy_active": True,
                    "gps_loss_injected": True,
                    "gps_fix_type": 0,
                },
                "evaluation_truth": {"simulation_time_seconds": 21},
            },
        )
        replay = SimpleNamespace(
            duration_seconds=0.2,
            samples={"atlas": []},
            at=lambda identity, elapsed: sample if elapsed >= 0.1 else None,
            label=lambda identity: "Atlas{\\pos(0,0)}",
        )
        path = self.root / "annotations.ass"
        write_scene_annotations(replay, path, 0, 2, 10, "Saved mission")
        content = path.read_text()
        self.assertIn("GPS request UNKNOWN", content)
        self.assertIn("GPS request OFF | injected loss ON | fix 0", content)
        self.assertIn("Measured GNSS XY OFF | vision XY ON", content)
        self.assertNotIn("Atlas{\\pos", content)
        self.assertIn("0:00:00.10,0:00:00.20", content)
