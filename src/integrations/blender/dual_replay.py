"""Evidence-based labels and composition for the two-aircraft replay player."""

import math
from pathlib import Path

from common.logging.replay_timeline import RecordedReplay
from integrations.blender.overlay import ReplayOverlay, fresh_received


def mission_interval(replay: RecordedReplay) -> tuple[float, float]:
    """Retain takeoff through landing plus context; omit preflight UI setup from replay."""
    active_states = {
        "taking_off",
        "navigating",
        "waiting_for_fresh_observation",
        "blocked",
        "landing",
    }
    times = [
        sample.elapsed_wall_seconds
        for rows in replay.samples.values()
        for sample in rows
        if (sample.observation.get("mission") or {}).get("state") in active_states
    ]
    if not times:
        raise ValueError("Saved capture contains no observed mission interval")
    return max(0.0, min(times) - 3), min(replay.duration_seconds, max(times) + 5)


def aiding_labels(observation: dict, now: float) -> tuple[str, bool | None]:
    """Keep requested GPS, native fix/fusion and visual evidence distinct and fresh."""
    aiding = observation.get("aiding") or {}
    if not fresh_received(aiding.get("received_monotonic"), now, 0.5):
        return "GPS fix: UNKNOWN | Fusion: UNKNOWN | Vision: UNKNOWN | IMU: UNKNOWN", None
    fix = aiding.get("gps_fix_type")
    fix_label = (
        "ACTIVE"
        if isinstance(fix, int) and fix >= 3
        else "NO FIX"
        if fix is not None
        else "UNKNOWN"
    )

    def flag(value: object) -> str:
        return "ACTIVE" if value is True else "INACTIVE" if value is False else "UNKNOWN"

    imu = next((row for row in observation.get("systems", []) if row.get("system") == "IMU"), {})
    imu_label = (
        "HEALTHY"
        if imu.get("healthy") is True
        and fresh_received(imu.get("received_monotonic"), now, imu.get("maximum_age_seconds", 1))
        else "UNKNOWN"
    )
    denied = aiding.get("gps_requested_enabled") is False
    return (
        f"GPS: {fix_label} | Fusion: {flag(aiding.get('gnss_xy_active'))} | "
        f"Vision: {flag(aiding.get('vision_xy_active'))} | IMU: {imu_label}"
    ), denied


def write_labels(
    replay: RecordedReplay, path: Path, start: float, end: float, fps: int = 10
) -> None:
    """Write recorded sensor status for each frame without recolouring source views."""
    overlay = ReplayOverlay(path, 800, 1050)
    count = math.ceil((end - start) * fps - 1e-8)
    previous = {identity: None for identity in replay.samples}
    lost_at = {}
    for frame in range(count):
        offset, elapsed = frame / fps, start + frame / fps
        for index, identity in enumerate(("vehicle_1", "vehicle_2")):
            top = index * 510
            sample = replay.at(identity, elapsed)
            observation = (
                sample.observation
                if sample and elapsed - sample.elapsed_wall_seconds <= 0.3
                else {}
            )
            text, denied = aiding_labels(observation, replay.started_monotonic + elapsed)
            if denied and previous[identity] is False:
                lost_at[identity] = offset
            if denied is not None:
                previous[identity] = denied
            title = "ATLAS" if index == 0 else "BOREAL"
            colour = "&H00B1D939&" if index == 0 else "&H00FFA562&"
            overlay.event(
                offset,
                offset + 1 / fps,
                rf"{{\pos(14,{top + 4})\fs21\b1\1c{colour}}}{title} · RECORDED REPLAY",
            )
            overlay.event(offset, offset + 1 / fps, rf"{{\pos(14,{top + 32})\fs16}}" + text)
            if denied:
                pulse = identity in lost_at and offset - lost_at[identity] < 3
                if pulse:
                    y = top + 60
                    overlay.event(
                        offset,
                        offset + 1 / fps,
                        r"{\p1\bord0\1c&H0055C5FF&}"
                        + f"m 0 {y} l 800 {y} 800 {y + 3} 0 {y + 3} "
                        + f"m 0 {y + 447} l 800 {y + 447} 800 {y + 450} 0 {y + 450} "
                        + f"m 0 {y} l 3 {y} 3 {y + 450} 0 {y + 450} "
                        + f"m 797 {y} l 800 {y} 800 {y + 450} 797 {y + 450}",
                        layer=1,
                    )
                overlay.event(
                    offset,
                    offset + 1 / fps,
                    rf"{{\pos(14,{top + 474})\fs18\1c&H0055C5FF&}}"
                    + (
                        "GPS INPUT DISABLED" if pulse else "GPS DISABLED · recorded sensor evidence"
                    ),
                )
        overlay.event(
            offset,
            offset + 1 / fps,
            r"{\pos(14,1023)\fs12}"
            + "BLENDER SIMULATION REPLAY · recorded poses · synthetic materials · not camera footage",
        )
        overlay.event(
            offset,
            offset + 1 / fps,
            r"{\pos(14,1040)\fs10}"
            + f"Terrain: Service Nova Scotia · Buildings: Halifax Regional Municipality · capture +{elapsed:.1f}s",
        )
    overlay.close()


def composite_command(directory: Path, duration: float) -> list[str]:
    """Keep each 800x450 camera unscaled inside the 800x1050 player layout."""
    graph = "[0:v][1:v]overlay=0:60[a];[a][2:v]overlay=0:570,ass=dual-labels.ass[out]"
    return [
        "ffmpeg",
        "-nostdin",
        "-v",
        "error",
        "-n",
        "-f",
        "lavfi",
        "-i",
        f"color=c=0x0b1522:s=800x1050:r=10:d={duration}",
        "-i",
        str(directory / "atlas/mission-replay.mp4"),
        "-i",
        str(directory / "boreal/mission-replay.mp4"),
        "-filter_complex_threads",
        "2",
        "-filter_complex",
        graph,
        "-map",
        "[out]",
        "-t",
        str(duration),
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-crf",
        "18",
        "-threads",
        "2",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(directory / "mission-replay.mp4"),
    ]
