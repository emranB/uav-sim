"""Geographic reference shared by the simulator and its desktop map."""

import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class WorldLocation:
    name: str
    latitude_deg: float
    longitude_deg: float
    elevation_m: float

    @classmethod
    def configured(cls) -> "WorldLocation":
        """All launchers use the same explicit, portable geographic configuration."""
        return cls.load(Path(os.environ.get("UAV_WORLD_LOCATION", "configs/worlds/default.json")))

    @classmethod
    def load(cls, path: Path) -> "WorldLocation":
        configuration = json.loads(path.read_text(encoding="utf-8"))
        if configuration.get("schema_version") != 1:
            raise ValueError("World location requires schema version 1")
        location = cls(
            configuration["name"],
            float(configuration["latitude_deg"]),
            float(configuration["longitude_deg"]),
            float(configuration["elevation_m"]),
        )
        if not location.name or not -90 <= location.latitude_deg <= 90:
            raise ValueError("World location needs a name and valid latitude")
        if not -180 <= location.longitude_deg <= 180 or not math.isfinite(location.elevation_m):
            raise ValueError("World longitude and elevation must be finite and valid")
        return location

    def as_dict(self) -> dict:
        return asdict(self)
