"""Validated, persistent desktop preferences independent of simulation state."""

import json
from pathlib import Path

DEFAULT_PREFERENCES = {"poll_interval_ms": 500, "log_rows": 1000, "map_width_percent": 80}


class DesktopPreferences:
    def __init__(self, path: Path) -> None:
        self.path = path

    @staticmethod
    def validate(values: dict) -> dict:
        result = dict(DEFAULT_PREFERENCES)
        for name, minimum, maximum in (
            ("poll_interval_ms", 250, 2000),
            ("log_rows", 100, 5000),
            ("map_width_percent", 60, 85),
        ):
            value = values.get(name, result[name])
            if type(value) is not int or not minimum <= value <= maximum:
                raise ValueError(f"{name} must be an integer in {minimum}..{maximum}")
            result[name] = value
        return result

    def load(self) -> dict:
        if not self.path.exists():
            return dict(DEFAULT_PREFERENCES)
        return self.validate(json.loads(self.path.read_text(encoding="utf-8")))

    def save(self, values: dict) -> dict:
        result = self.validate(values)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.path)
        return result
