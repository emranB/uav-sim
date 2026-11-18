"""Bounded log retrieval and persistent recording inventory for diagnostics panels."""

import json
import math
import re
import time
from collections.abc import Iterator
from datetime import datetime
from pathlib import Path


class RecordingCatalog:
    SOURCES = (
        "px4",
        "gazebo",
        "gazebo-viewer",
        "qgroundcontrol",
        "application",
        "desktop",
        "environment",
        "camera-streams",
    )

    def __init__(self, directory: Path, shared_directory: Path | None = None) -> None:
        self.directory = directory
        self.shared_directory = shared_directory or directory

    def logs(self, source: str, query: str = "") -> dict:
        if source not in self.SOURCES:
            raise ValueError("Unknown log source")
        shared = source in {
            "gazebo",
            "gazebo-viewer",
            "qgroundcontrol",
            "application",
            "desktop",
            "camera-streams",
        }
        directory = self.shared_directory if shared else self.directory
        path = directory / "logs" / f"{source}.jsonl"
        if not path.exists():
            return {"source": source, "records": [], "tail_only": True}
        with path.open("rb") as stream:
            stream.seek(0, 2)
            offset = max(0, stream.tell() - 524288)
            stream.seek(offset)
            if offset:
                stream.readline()
            lines = stream.read().decode("utf-8", errors="replace").splitlines()
        records = []
        for line in lines:
            if query.casefold() not in line.casefold():
                continue
            try:
                records.append(json.loads(line))
            except ValueError:
                records.append({"source": source, "message": line})
        return {"source": source, "records": records[-1000:], "tail_only": True}

    def list(self) -> dict:
        telemetry = sorted((self.directory / "telemetry").glob("*.jsonl"))[-200:]
        flight_logs = sorted((self.directory / "px4").glob("run-*/log/*/*.ulg"))[-200:]
        return {
            "telemetry": [{"name": p.name, "bytes": p.stat().st_size} for p in telemetry],
            "flight_logs": [
                {"name": str(p.relative_to(self.directory)), "bytes": p.stat().st_size}
                for p in flight_logs
            ],
        }

    @staticmethod
    def _position_records(path: Path, size: int, deadline: float) -> Iterator[dict]:
        """Read large host-mounted recordings in blocks, with bounded line and memory use."""
        pending = b""
        with path.open("rb") as stream:
            remaining = size
            while remaining > 0:
                if time.monotonic() > deadline:
                    raise TimeoutError("Recording analysis timed out; choose a smaller recording")
                block = stream.read(min(remaining, 4 * 1024 * 1024))
                if not block:
                    break
                remaining -= len(block)
                lines = (pending + block).split(b"\n")
                pending = lines.pop()
                if len(pending) > 65536:
                    raise ValueError("Telemetry record exceeds 64 KiB")
                for line in lines:
                    if len(line) > 65536:
                        raise ValueError("Telemetry record exceeds 64 KiB")
                    if b'"GLOBAL_POSITION_INT"' not in line:
                        continue
                    try:
                        yield json.loads(line)
                    except ValueError:
                        continue
            if pending and b'"GLOBAL_POSITION_INT"' in pending:
                try:
                    yield json.loads(pending)
                except ValueError:
                    pass

    def statistics(self, name: str, deadline_monotonic: float | None = None) -> dict:
        if not isinstance(name, str) or not re.fullmatch(
            r"telemetry-\d+(?:-\d{4,}-(?:idle|flight))?\.jsonl", name
        ):
            raise ValueError("Choose a telemetry recording from the inventory")
        folder = (self.directory / "telemetry").resolve()
        path = (folder / name).resolve()
        if not path.is_relative_to(folder):
            raise ValueError("Recording is outside the telemetry directory")
        size = path.stat().st_size
        if size > 128 * 1024 * 1024:
            raise ValueError("Recording comparison supports files up to 128 MiB")
        result = {
            "name": name,
            "position_samples": 0,
            "maximum_altitude_m": 0.0,
            "maximum_speed_m_s": 0.0,
            "maximum_displacement_m": 0.0,
        }
        first_position = None
        first_time = None
        last_time = None
        deadline = deadline_monotonic or time.monotonic() + 25
        for record in self._position_records(path, size, deadline):
            try:
                position = record["message"]
                if position.get("mavpackettype") != "GLOBAL_POSITION_INT":
                    continue
                latitude, longitude = position["lat"] / 1e7, position["lon"] / 1e7
                timestamp = datetime.fromisoformat(record["timestamp_utc"]).timestamp()
                if first_position is None:
                    first_position = (latitude, longitude)
                    first_time = timestamp
                last_time = timestamp
                displacement = math.hypot(
                    (latitude - first_position[0]) * 111320,
                    (longitude - first_position[1]) * 111320 * math.cos(math.radians(latitude)),
                )
                result["position_samples"] += 1
                result["maximum_altitude_m"] = max(
                    result["maximum_altitude_m"], position["relative_alt"] / 1000
                )
                result["maximum_speed_m_s"] = max(
                    result["maximum_speed_m_s"],
                    math.hypot(position["vx"], position["vy"]) / 100,
                )
                result["maximum_displacement_m"] = max(
                    result["maximum_displacement_m"], displacement
                )
            except (ValueError, KeyError, TypeError):
                continue
        if not result["position_samples"]:
            raise ValueError("Recording has no valid geographic position samples")
        result["duration_utc_seconds"] = max(0, last_time - first_time)
        return result

    def compare(self, left: str, right: str) -> dict:
        deadline = time.monotonic() + 25
        first = self.statistics(left, deadline)
        second = self.statistics(right, deadline)
        result = {
            "left": first,
            "right": second,
            "delta": {
                key: second[key] - first[key]
                for key in ("maximum_altitude_m", "maximum_speed_m_s", "maximum_displacement_m")
            },
            "scope": "Whole-recording GPS statistics; UTC duration includes paused and idle time",
        }
        reports = self.directory / "reports"
        reports.mkdir(parents=True, exist_ok=True)
        name = f"comparison-{time.time_ns()}.json"
        (reports / name).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        return {"accepted": True, "comparison": result, "report_name": name}
