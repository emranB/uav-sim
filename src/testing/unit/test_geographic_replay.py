"""Geographic replay reads declared bounded assets and retains their exact bytes."""

import hashlib
import json

import pytest

from integrations.replay import WorkspaceReplaySource


def test_geographic_meshes_are_retained_with_exact_hashes(tmp_path, monkeypatch) -> None:
    terrain = tmp_path / "terrain"
    buildings = tmp_path / "buildings"
    terrain.mkdir()
    buildings.mkdir()
    content = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
    (terrain / "terrain.obj").write_text(content)
    (buildings / "one.obj").write_text(content)
    (buildings / "buildings.json").write_text(json.dumps({"buildings": [{"file": "one.obj"}]}))
    monkeypatch.setenv("UAV_TERRAIN_DIR", str(terrain))
    monkeypatch.setenv("UAV_WATERFRONT_BUILDINGS_DIR", str(buildings))
    result = WorkspaceReplaySource({}).geographic_assets()
    assert set(result) == {"terrain.obj", "one.obj"}
    assert result["one.obj"]["contents"] == content
    assert result["one.obj"]["sha256"] == hashlib.sha256(content.encode()).hexdigest()


def test_geographic_manifest_cannot_escape_asset_directory(tmp_path, monkeypatch) -> None:
    (tmp_path / "buildings.json").write_text(
        json.dumps({"buildings": [{"file": "../private.obj"}]})
    )
    monkeypatch.delenv("UAV_TERRAIN_DIR", raising=False)
    monkeypatch.setenv("UAV_WATERFRONT_BUILDINGS_DIR", str(tmp_path))
    with pytest.raises(ValueError, match="plain OBJ"):
        WorkspaceReplaySource({}).geographic_assets()
