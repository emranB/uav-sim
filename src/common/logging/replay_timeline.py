"""Indexed recorded observations for offline playback; no simulation or control access."""

import bisect
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from common.logging.replay_export import verify_replay_export


@dataclass(frozen=True)
class ReplaySample:
    elapsed_wall_seconds: float
    observation: dict
    segment: int


class RecordedReplay:
    """Use observed sample times without extrapolation or joining reset/gap boundaries."""

    def __init__(self, directory: Path) -> None:
        self.manifest = verify_replay_export(directory)
        self.context = json.loads((directory / "context.json").read_text())
        self.started_monotonic = self.manifest["start_monotonic"]
        self.duration_seconds = self.manifest["end_monotonic"] - self.started_monotonic
        self.samples: dict[str, list[ReplaySample]] = {
            identity: [] for identity in self.manifest["vehicle_ids"]
        }
        previous, segments = {}, {}
        count = 0
        with (directory / "observations.jsonl").open("rb") as stream:
            while encoded := stream.readline(65537):
                if len(encoded) > 65536:
                    raise ValueError("Replay observation line exceeds 64 KiB")
                count += 1
                if count > 160000:
                    raise ValueError("Replay timeline exceeds 160000 observations")
                record = json.loads(encoded)
                observation = record["observation"]
                identity = observation["vehicle_id"]
                if identity not in self.samples or identity != record["vehicle_id"]:
                    raise ValueError("Replay timeline vehicle differs from its sealed selection")
                aiding = observation.get("aiding") or {}
                key = (
                    observation["controller_generation"],
                    observation["world_epoch"],
                    aiding.get("map_generation"),
                    aiding.get("xy_reset_counter"),
                    aiding.get("z_reset_counter"),
                    aiding.get("heading_reset_counter"),
                )
                if record.get("segment_changed") or previous.get(identity) != key:
                    segments[identity] = segments.get(identity, 0) + 1
                previous[identity] = key
                # Observation receipt is after the source snapshot. Sensor stamps are
                # retained separately; no later aiding record appears before receipt.
                elapsed = record["observed_monotonic"] - self.started_monotonic
                self.samples[identity].append(
                    ReplaySample(elapsed, observation, segments[identity])
                )
        self.samples = {
            identity: sorted(rows, key=lambda row: row.elapsed_wall_seconds)
            for identity, rows in self.samples.items()
        }
        self._times = {
            identity: [row.elapsed_wall_seconds for row in rows]
            for identity, rows in self.samples.items()
        }
        self.bindings = {}
        self.missions = {}
        for identity, rows in self.samples.items():
            for sample in rows:
                binding = sample.observation.get("frame_binding")
                mission = sample.observation.get("mission")
                if binding:
                    self.bindings.setdefault(identity, binding)
                if mission and mission.get("planned_goal_offset_enu_m") is not None:
                    self.missions.setdefault(identity, mission)
        self.shared_frame = bool(self.samples) and all(
            identity in self.bindings
            and len({row.observation["controller_generation"] for row in rows}) == 1
            and len({row.observation["world_epoch"] for row in rows}) == 1
            and len({(row.observation.get("aiding") or {}).get("map_generation") for row in rows})
            <= 1
            and len(
                {
                    json.dumps(row.observation["frame_binding"], sort_keys=True)
                    for row in rows
                    if row.observation.get("frame_binding")
                }
            )
            == 1
            and len({(row.observation.get("aiding") or {}).get("xy_reset_counter") for row in rows})
            <= 1
            for identity, rows in self.samples.items()
        )

    def label(self, identity: str) -> str:
        return (self.context.get("vehicles", {}).get(identity, {}).get("configuration") or {}).get(
            "label", identity
        )

    def at(self, identity: str, elapsed_wall_seconds: float) -> ReplaySample | None:
        """Return the latest observation at or before the frame, never a future sample."""
        index = bisect.bisect_right(self._times[identity], elapsed_wall_seconds) - 1
        return self.samples[identity][index] if index >= 0 else None

    def position(self, identity: str, observation: dict) -> tuple[float, float, float] | None:
        pose = observation.get("pose")
        if not pose:
            return None
        position = tuple(pose["position_enu_m"])
        if len(position) != 3 or any(
            type(value) not in {int, float} or not math.isfinite(value) for value in position
        ):
            raise ValueError("Replay position must be three finite estimated ENU metres")
        if self.shared_frame:
            binding = self.bindings[identity]
            position = tuple(
                value - home + launch
                for value, home, launch in zip(
                    position,
                    binding["estimated_home_enu_m"],
                    binding["scene_launch_enu_m"],
                )
            )
        return position

    @staticmethod
    def scene_position(observation: dict) -> tuple[float, float, float] | None:
        """Return recorded simulator ENU position, kept distinct from estimated pose."""
        scene = observation.get("evaluation_truth")
        if not scene:
            return None
        if scene.get("vehicle_id") != observation["vehicle_id"]:
            raise ValueError("Recorded scene pose belongs to another vehicle")
        position = tuple(scene["position_enu_m"])
        if len(position) != 3 or any(
            type(value) not in {int, float} or not math.isfinite(value) for value in position
        ):
            raise ValueError("Recorded scene position must be three finite ENU metres")
        return position

    @property
    def scene_available(self) -> bool:
        """Common scene playback requires actual poses and receipt clocks for every identity."""
        return bool(self.samples) and all(
            any(
                self.scene_position(row.observation) is not None
                and row.observation.get("evaluation_truth_received_monotonic") is not None
                for row in rows
            )
            and len({row.observation["world_epoch"] for row in rows}) == 1
            for rows in self.samples.values()
        )

    def planned_markers(
        self, identity: str, *, scene_frame: bool = False
    ) -> tuple[tuple, tuple] | None:
        """Planned home/waypoint markers are intent; they never become observed paths."""
        if identity not in self.bindings or identity not in self.missions:
            return None
        binding, mission = self.bindings[identity], self.missions[identity]
        home = tuple(
            binding["scene_launch_enu_m"]
            if self.shared_frame or scene_frame
            else binding["estimated_home_enu_m"]
        )
        goal = (
            home[0] + mission["planned_goal_offset_enu_m"][0],
            home[1] + mission["planned_goal_offset_enu_m"][1],
            home[2],
        )
        return home, goal

    def obstacle_footprints(
        self, *, scene_frame: bool = False
    ) -> list[tuple[str, tuple[tuple[float, float], ...]]]:
        """Project retained horizontal box collisions from the actual SDF into ENU."""
        retained = self.context.get("retained_world_sdf")
        if not (self.shared_frame or scene_frame) or not retained:
            return []
        world = ET.fromstring(retained["contents"]).find("world")
        if world is None:
            return []
        result = []
        for model in world.findall("model"):
            if not model.get("name", "").startswith("avoidance_obstacle_"):
                continue
            pose = [float(value) for value in model.findtext("pose", "0 0 0 0 0 0").split()]
            if len(pose) != 6 or any(abs(value) > 1e-8 for value in pose[3:5]):
                continue
            size = model.findtext("link/collision/geometry/box/size")
            if size is None:
                continue
            width, depth, _ = [float(value) for value in size.split()]
            cosine, sine = math.cos(pose[5]), math.sin(pose[5])
            corners = tuple(
                (pose[0] + x * cosine - y * sine, pose[1] + x * sine + y * cosine)
                for x, y in (
                    (-width / 2, -depth / 2),
                    (width / 2, -depth / 2),
                    (width / 2, depth / 2),
                    (-width / 2, depth / 2),
                )
            )
            result.append((model.get("name"), corners))
        return result
