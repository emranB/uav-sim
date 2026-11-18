"""Time-indexed replay symbols and progressive trails from recorded observations only."""

import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ReplayViewport:
    """Uniform ENU-metre to image-pixel projection; north is up."""

    bounds: tuple[float, float, float, float]
    rectangle: tuple[float, float, float, float]

    def project(self, point: tuple) -> tuple[float, float]:
        xmin, xmax, ymin, ymax = self.bounds
        left, top, width, height = self.rectangle
        scale = min(width / (xmax - xmin), height / (ymax - ymin))
        return (
            left + width / 2 + (point[0] - (xmin + xmax) / 2) * scale,
            top + height / 2 - (point[1] - (ymin + ymax) / 2) * scale,
        )


def clean_label(value: object) -> str:
    """Keep retained names out of ASS control syntax."""
    return (
        str(value).replace("\\", "/").replace("{", "(").replace("}", ")").replace("\n", " ")[:140]
    )


def ass_time(seconds: float, *, ceiling: bool = False) -> str:
    count = max(0, math.ceil(seconds * 100 - 1e-8) if ceiling else round(seconds * 100))
    return f"{count // 360000}:{count // 6000 % 60:02}:{count // 100 % 60:02}.{count % 100:02}"


def fresh_received(received: object, monotonic_seconds: float, maximum_age_seconds=0.3) -> bool:
    return (
        type(received) in {int, float} and 0 <= monotonic_seconds - received <= maximum_age_seconds
    )


def scene_yaw(observation: dict) -> float:
    x, y, z, w = observation["evaluation_truth"]["orientation_xyzw"]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def estimated_relative_position(replay, identity: str, observation: dict) -> tuple | None:
    pose = observation.get("pose")
    if not pose:
        return None
    point = tuple(pose["position_enu_m"])
    home = replay.bindings.get(identity, {}).get("estimated_home_enu_m", (0, 0, 0))
    return tuple(value - origin for value, origin in zip(point, home))


class ReplayOverlay:
    """Stream vector events; trails appear at their observed receipt time, never in advance."""

    def __init__(self, path: Path, width: int, height: int) -> None:
        self.stream = path.open("w", encoding="utf-8")
        self.stream.write(
            f"[Script Info]\nScriptType: v4.00+\nPlayResX: {width}\nPlayResY: {height}\n"
        )
        self.stream.write(
            "[V4+ Styles]\nFormat: Name,Fontname,Fontsize,PrimaryColour,SecondaryColour,OutlineColour,BackColour,Bold,Italic,Underline,StrikeOut,ScaleX,ScaleY,Spacing,Angle,BorderStyle,Outline,Shadow,Alignment,MarginL,MarginR,MarginV,Encoding\n"
        )
        self.stream.write(
            "Style: Data,DejaVu Sans,20,&H00F2E7D3,&H000000FF,&H0020150D,&H0020150D,0,0,0,0,100,100,0,0,1,1,0,7,0,0,0,1\n"
        )
        self.stream.write(
            "[Events]\nFormat: Layer,Start,End,Style,Name,MarginL,MarginR,MarginV,Effect,Text\n"
        )

    def event(self, start: float, end: float, body: str, layer: int = 2) -> None:
        if end > start:
            self.stream.write(
                f"Dialogue: {layer},{ass_time(start, ceiling=True)},{ass_time(end, ceiling=True)},Data,,0,0,0,,{body}\n"
            )

    def label(self, start, end, x, y, text, size=20) -> None:
        self.event(start, end, f"{{\\pos({x:.1f},{y:.1f})\\fs{size}}}" + clean_label(text), 5)

    def polygon(self, start, end, points, colour, layer=2) -> None:
        if not points:
            return
        body = "m " + " l ".join(f"{x:.1f} {y:.1f}" for x, y in points)
        self.event(start, end, f"{{\\pos(0,0)\\bord0\\1c&H{colour}&\\p1}}{body}{{\\p0}}", layer)

    def segment(self, start, end, first, second, colour, width=2.2, layer=2) -> None:
        dx, dy = second[0] - first[0], second[1] - first[1]
        distance = math.hypot(dx, dy)
        if distance < 0.2:
            return
        nx, ny = -dy / distance * width / 2, dx / distance * width / 2
        self.polygon(
            start,
            end,
            [
                (first[0] + nx, first[1] + ny),
                (second[0] + nx, second[1] + ny),
                (second[0] - nx, second[1] - ny),
                (first[0] - nx, first[1] - ny),
            ],
            colour,
            layer,
        )

    def quadrotor(self, start, end, centre, yaw, colour, letter, radius=18) -> None:
        cosine, sine = math.cos(yaw), math.sin(yaw)

        def rotated(u, v):
            return centre[0] + cosine * u - sine * v, centre[1] - sine * u - cosine * v

        for u, v in ((-radius, -radius), (-radius, radius), (radius, -radius), (radius, radius)):
            self.segment(start, end, centre, rotated(u, v), colour, 4, 4)
            rotor = [
                rotated(u + 6 * math.cos(a * math.tau / 16), v + 6 * math.sin(a * math.tau / 16))
                for a in range(16)
            ]
            self.polygon(start, end, rotor, colour, 4)
        self.polygon(
            start, end, [rotated(11, 0), rotated(0, 8), rotated(-9, 0), rotated(0, -8)], "302015", 4
        )
        self.label(start, end, centre[0] - 5, centre[1] - 9, letter, 15)

    def close(self) -> None:
        self.stream.close()


def progressive_trail(overlay, replay, identity, viewport, *, scene, colour) -> int:
    """Keep actual discontinuities and independent clocks; never join an estimator reset."""
    previous = None
    segments = 0
    for row in replay.samples[identity]:
        observed = replay.started_monotonic + row.elapsed_wall_seconds
        pose = row.observation.get("evaluation_truth" if scene else "pose")
        received = (
            row.observation.get("evaluation_truth_received_monotonic")
            if scene
            else pose.get("received_monotonic")
            if pose
            else None
        )
        point = (
            replay.scene_position(row.observation)
            if scene
            else estimated_relative_position(replay, identity, row.observation)
        )
        if point is None or not fresh_received(received, observed):
            previous = None
            continue
        key = (
            (row.observation["controller_generation"], row.observation["world_epoch"])
            if scene
            else row.segment
        )
        current = (row.elapsed_wall_seconds, key, viewport.project(point))
        if previous and key == previous[1] and current[0] - previous[0] <= 0.5:
            overlay.segment(
                current[0], replay.duration_seconds, previous[2], current[2], "504840", 1.8, 1
            )
            overlay.segment(
                current[0],
                min(replay.duration_seconds, current[0] + 8),
                previous[2],
                current[2],
                colour,
                2.8,
                2,
            )
            segments += 1
        previous = current
    return segments
