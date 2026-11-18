"""Independent camera renders share progress and terminate together on failure."""

import json
import threading
import time
from unittest.mock import patch

import pytest

from integrations.blender.renderer import BlenderReplayRenderer


@pytest.mark.parametrize("fail_atlas", [False, True])
def test_parallel_cameras_complete_or_stop_their_sibling(tmp_path, monkeypatch, fail_atlas):
    monkeypatch.setenv("UAV_REPLAY_PARALLEL_CAMERAS", "true")
    renderer = BlenderReplayRenderer()
    barrier = threading.Barrier(2)
    entered = set()
    progress_values = []
    sibling_stopped = threading.Event()

    def run(command, directory, cancelled, check, progress, log_name):
        if log_name != "blender.log":
            return
        entered.add(directory.name)
        barrier.wait(timeout=3)
        progress(0.5)
        if fail_atlas:
            if directory.name == "atlas":
                raise RuntimeError("Atlas renderer failed")
            try:
                for _ in range(300):
                    check()
                    time.sleep(0.01)
            finally:
                sibling_stopped.set()
            pytest.fail("Sibling camera did not receive failure cancellation")
        (directory / "render-manifest.json").write_text(json.dumps({"passed": True}))

    with (
        patch("common.logging.replay_timeline.RecordedReplay"),
        patch("integrations.blender.dual_replay.mission_interval", return_value=(0, 1)),
        patch("integrations.blender.dual_replay.write_labels"),
        patch.object(renderer, "availability", return_value={"executable": "blender"}),
        patch.object(renderer, "_run", side_effect=run),
    ):
        if fail_atlas:
            with pytest.raises(RuntimeError, match="Atlas renderer failed"):
                renderer._render_dual_replay(
                    tmp_path, {}, threading.Event(), lambda: None, progress_values.append
                )
            assert sibling_stopped.is_set()
        else:
            renderer._render_dual_replay(
                tmp_path, {}, threading.Event(), lambda: None, progress_values.append
            )
            manifest = json.loads((tmp_path / "render-manifest.json").read_text())
            assert set(manifest["aircraft_views"]) == {"vehicle_1", "vehicle_2"}
            assert manifest["parallel_aircraft_views"]
            assert progress_values == sorted(progress_values)
            assert progress_values[-1] == 1
    assert entered == {"atlas", "boreal"}
