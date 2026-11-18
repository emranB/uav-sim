"""Record waypoint-flight contacts and tracking evidence without commanding flight."""

import json
import math
import sys
import time
import urllib.request
from pathlib import Path

from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node

from common.logging.storage import write_atomic, write_json_atomic
from testing.simulation.navigation import clearance
from testing.simulation.tracking import TrackingContinuity

vehicle_id, destination_name, stop_name, strict = sys.argv[1:]
strict = strict == "true"
destination = Path(destination_name)
stop = Path(stop_name)
scenario = json.loads(Path("configs/navigation/stereo/single-01.json").read_text())
node = Node()
contact_count = 0
contact_details = []


def contacts(message: Contacts) -> None:
    global contact_count
    if message.contact:
        contact_count += 1
        if len(contact_details) < 8:
            contact_details.append(str(message)[:1500])


topics = []
for index in range(len(scenario["obstacles"])):
    topic = "/avoidance/contacts/" + str(index)
    if not node.subscribe(Contacts, topic, contacts):
        raise RuntimeError("Contact subscription failed")
    topics.append(topic)
deadline = time.monotonic() + 20
while not all(node.topic_info(topic)[0] for topic in topics):
    if time.monotonic() > deadline:
        raise RuntimeError("Physical contact publishers unavailable")
    time.sleep(0.1)


def request(resource: str) -> dict:
    with urllib.request.urlopen("http://127.0.0.1:8080/" + resource, timeout=2) as response:
        return json.load(response)


write_atomic(destination.with_suffix(".ready"), b"ready")
rows = []
continuity = TrackingContinuity()
airborne = 0
started = time.monotonic()
error = None
fields = [
    "ready",
    "stale",
    "position_valid",
    "tracking_state",
    "session_id",
    "map_id",
    "map_generation",
    "processed_frames",
    "tracked_frames",
    "tracking_loss_events",
    "input_gap_events",
    "maximum_pose_speed_m_s",
    "simulation_timestamp_ns",
    "surface_backend",
    "error",
]
try:
    with destination.with_suffix(".samples.jsonl").open("w") as stream:
        while not stop.exists() and time.monotonic() - started < 240:
            tick = time.monotonic()
            row = {"wall_time": time.time(), "monotonic": tick}
            try:
                scene = request("scene")
                pose = next(p for p in scene["poses"] if p["vehicle_id"] == vehicle_id)
                telemetry = request("telemetry?vehicle_id=" + vehicle_id)
                mapping = request("perception/map?vehicle_id=" + vehicle_id)
                row.update(
                    peer_distance_m=min(
                        math.dist(p["position_enu_m"], pose["position_enu_m"])
                        for p in scene["poses"]
                        if p["vehicle_id"] != vehicle_id
                    ),
                    epoch=scene["epoch"],
                    scene_stale=scene["stale"],
                    pose=pose,
                    armed=telemetry["armed"],
                    connected=telemetry["connected"],
                    recording=telemetry.get("recording"),
                    mapping={k: mapping.get(k) for k in fields},
                    clearance_m=min(
                        clearance(pose["position_enu_m"], box) for box in scenario["obstacles"]
                    ),
                )
                if telemetry["armed"] and pose["position_enu_m"][2] > 3:
                    airborne += 1
                    if strict:
                        continuity.observe(mapping)
            except (OSError, ValueError, KeyError, StopIteration) as failure:
                row["error"] = str(failure)
            rows.append(row)
            stream.write(json.dumps(row, allow_nan=False) + "\n")
            stream.flush()
            time.sleep(max(0, 0.5 - (time.monotonic() - tick)))
except (OSError, ValueError) as failure:
    error = str(failure)
finally:
    for topic in topics:
        node.unsubscribe(topic)
    points = [r for r in rows if "pose" in r]
    air = [r for r in points if r.get("armed") and r["pose"]["position_enu_m"][2] > 3]
    rtf = None
    groups = {}
    for row in air:
        groups.setdefault(row["epoch"], []).append(row)
    clock_groups = [group for group in groups.values() if len(group) >= 3]
    if clock_groups:
        group = max(clock_groups, key=lambda group: group[-1]["monotonic"] - group[0]["monotonic"])
        rtf = (
            group[-1]["pose"]["simulation_time_seconds"]
            - group[0]["pose"]["simulation_time_seconds"]
        ) / (group[-1]["monotonic"] - group[0]["monotonic"])
    summary = {
        "schema_version": 1,
        "scope": "Read-only actual physics contacts, sampled clearance and existing observer continuity; never a command input",
        "vehicle_id": vehicle_id,
        "strict_observer_check": strict,
        "contacts": contact_count,
        "contact_details": contact_details,
        "minimum_clearance_m": min((r["clearance_m"] for r in points), default=None),
        "minimum_sampled_peer_distance_m": min(
            (r["peer_distance_m"] for r in points), default=None
        ),
        "airborne_samples": airborne,
        "airborne_tracking": (
            continuity.result()
            if strict
            else {
                "not_gated": "Intentional pause/lifecycle fault; raw state and stale frames remain explicit"
            }
        ),
        "airborne_achieved_rtf": rtf,
        "api_error_samples": sum("error" in r for r in rows),
        "error": error,
        "samples": str(destination.with_suffix(".samples.jsonl")),
        "rows": len(rows),
    }
    write_json_atomic(destination, summary)
    print(json.dumps(summary), flush=True)
