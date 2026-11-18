"""Qualify bounded stereo recording, both live mapping inputs and native viewer readiness."""

import json
import os
import time
from pathlib import Path

from common.logging.storage import write_json_atomic
from testing.simulation.navigation_acceptance import fleet, request

pair_limit = int(os.environ["UAV_RECORD_LIMIT_PAIRS"])
deadline = time.monotonic() + 100
observations = {}
viewer = {}
while time.monotonic() < deadline:
    try:
        for vehicle_id in fleet():
            camera = request(f"perception?vehicle_id={vehicle_id}")["camera"]
            mapping = request(f"perception/map?vehicle_id={vehicle_id}")
            manifest = Path("outputs") / camera.get("recording", "__unavailable__")
            data = json.loads(manifest.read_text()) if manifest.is_file() else {}
            observations[vehicle_id] = {
                "camera_ready": camera.get("ready"),
                "archive_pair_count": len(data.get("pairs", [])),
                "mapping_stale": mapping.get("stale"),
                "processed_frames": mapping.get("processed_frames"),
                "mapping_error": mapping.get("error"),
                "manifest": str(manifest),
            }
        viewer = request("scene")["viewer"]
        if (
            len(observations) == 2
            and all(
                row["camera_ready"]
                and 1 <= row["archive_pair_count"] <= pair_limit
                and row["mapping_stale"] is False
                and (row["processed_frames"] or 0) >= 3
                for row in observations.values()
            )
            and viewer["available"]
        ):
            passed = True
            break
    except (OSError, ValueError, KeyError):
        pass
    time.sleep(0.5)
else:
    passed = False
result = {"passed": passed, "vehicles": observations, "viewer": viewer}
write_json_atomic(Path("outputs/reports/platform-observation-qualification.json"), result)
print(json.dumps(result))
raise SystemExit(0 if passed else 1)
