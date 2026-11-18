"""Blender schematic scene plus independent time-indexed simulator/estimate overlays."""

import json
import math
import sys
from pathlib import Path


def main() -> None:
    import bpy

    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from common.logging.replay_timeline import RecordedReplay
    from integrations.blender.overlay import (
        ReplayOverlay,
        ReplayViewport,
        estimated_relative_position,
        fresh_received,
        progressive_trail,
        scene_yaw,
    )

    directory = Path(sys.argv[sys.argv.index("--") + 1]).resolve()
    replay = RecordedReplay(directory)
    width, height, fps = 1600, 900, 10
    frame_count = max(1, math.ceil(replay.duration_seconds * fps))
    if frame_count > 18000:
        raise ValueError("Replay exceeds its 18000-frame video limit")
    identities = list(replay.samples)
    if len(identities) > 2:
        raise ValueError("This readable comparison layout supports at most two selected vehicles")
    scene_mode = replay.scene_available
    colours = [(0.18, 0.82, 0.63), (0.30, 0.62, 1)]
    ass_colours = ["A1D12E", "FF9E4D"]
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = 16
    scene.cycles.use_denoising = False
    scene.render.threads_mode, scene.render.threads = "FIXED", 2
    scene.render.resolution_x, scene.render.resolution_y = width, height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.view_settings.view_transform = "Standard"
    scene.world.use_nodes = True
    scene.world.node_tree.nodes["Background"].inputs[0].default_value = (0.025, 0.04, 0.065, 1)

    def material(name, colour):
        result = bpy.data.materials.new(name)
        result.use_nodes = True
        nodes = result.node_tree.nodes
        nodes.clear()
        emission = nodes.new("ShaderNodeEmission")
        emission.inputs["Color"].default_value = (*colour, 1)
        output = nodes.new("ShaderNodeOutputMaterial")
        result.node_tree.links.new(emission.outputs[0], output.inputs[0])
        return result

    white = material("labels", (0.82, 0.88, 0.95))
    grid = material("metre grid", (0.14, 0.20, 0.27))
    boundary = material("pane edges", (0.30, 0.40, 0.50))
    obstacle = material("recorded box footprints", (0.30, 0.37, 0.45))
    target = material("planned waypoint", (0.96, 0.70, 0.25))
    vehicle_materials = {
        identity: material(identity, colours[index]) for index, identity in enumerate(identities)
    }

    def line(points, colour, thickness=1):
        if len(points) < 2:
            return
        curve = bpy.data.curves.new("recorded geometry", "CURVE")
        curve.dimensions, curve.bevel_depth, curve.bevel_resolution = "3D", thickness / 2, 1
        spline = curve.splines.new("POLY")
        spline.points.add(len(points) - 1)
        for point, (x, y) in zip(spline.points, points):
            point.co = (x, height - y, 0.2, 1)
        instance = bpy.data.objects.new("recorded geometry", curve)
        scene.collection.objects.link(instance)
        instance.data.materials.append(colour)

    def text(label, x, y, pixels=20, colour=white):
        curve = bpy.data.curves.new("annotation", "FONT")
        curve.body, curve.size = label, pixels / 0.72
        instance = bpy.data.objects.new("annotation", curve)
        scene.collection.objects.link(instance)
        instance.location = (x, height - y, 0.4)
        instance.data.materials.append(colour)

    def bounds(points, margin=1):
        if not points:
            return -2, 4, -2, 3
        return (
            min(p[0] for p in points) - margin,
            max(p[0] for p in points) + margin,
            min(p[1] for p in points) - margin,
            max(p[1] for p in points) + margin,
        )

    def axes(viewport):
        xmin, xmax, ymin, ymax = viewport.bounds
        left, top, pane_width, pane_height = viewport.rectangle
        line(
            [
                (left, top),
                (left + pane_width, top),
                (left + pane_width, top + pane_height),
                (left, top + pane_height),
                (left, top),
            ],
            boundary,
        )
        for east in range(math.ceil(xmin), math.floor(xmax) + 1):
            line([viewport.project((east, ymin)), viewport.project((east, ymax))], grid, 0.7)
        for north in range(math.ceil(ymin), math.floor(ymax) + 1):
            line([viewport.project((xmin, north)), viewport.project((xmax, north))], grid, 0.7)

    def planned_symbols(identity, viewport, markers, letter, size=9):
        if not markers:
            return
        home, goal = [viewport.project(point) for point in markers]
        ring = [
            (
                home[0] + size * math.cos(a * math.tau / 32),
                home[1] + size * math.sin(a * math.tau / 32),
            )
            for a in range(33)
        ]
        line(ring, vehicle_materials[identity], 2)
        line(
            [
                (goal[0], goal[1] - size),
                (goal[0] + size, goal[1]),
                (goal[0], goal[1] + size),
                (goal[0] - size, goal[1]),
                (goal[0], goal[1] - size),
            ],
            target,
            2,
        )
        text("HOME " + letter, home[0] + 13, home[1] + 22, 16)
        text("WP1 " + letter + " (planned)", goal[0] + 13, goal[1] - 13, 16)

    planned_scene = {
        identity: replay.planned_markers(identity, scene_frame=True) for identity in identities
    }
    scene_points = [
        replay.scene_position(row.observation) for rows in replay.samples.values() for row in rows
    ]
    scene_points = [point for point in scene_points if point is not None]
    for markers in planned_scene.values():
        if markers:
            scene_points.extend(markers)
    footprints = replay.obstacle_footprints(scene_frame=scene_mode)
    if scene_mode and scene_points:
        xmin, xmax, ymin, ymax = bounds(scene_points, 4)
        for _, corners in footprints:
            span_x = max(p[0] for p in corners) - min(p[0] for p in corners)
            span_y = max(p[1] for p in corners) - min(p[1] for p in corners)
            centre = (sum(p[0] for p in corners) / 4, sum(p[1] for p in corners) / 4)
            if max(span_x, span_y) <= 6 and xmin <= centre[0] <= xmax and ymin <= centre[1] <= ymax:
                scene_points.extend(corners)
    main_view = ReplayViewport(bounds(scene_points, 1.5), (32, 205, 990, 475))
    if scene_mode:
        axes(main_view)
    visible_footprints = 0
    for obstacle_name, corners in footprints:
        if not scene_mode:
            continue
        xmin, xmax, ymin, ymax = main_view.bounds
        if any(not xmin <= p[0] <= xmax or not ymin <= p[1] <= ymax for p in corners):
            continue
        projected = [main_view.project(point) for point in (*corners, corners[0])]
        line(projected, obstacle, 6)
        visible_footprints += 1
        text(
            obstacle_name.replace("avoidance_obstacle_", "Obstacle "),
            projected[0][0],
            projected[0][1] - 10,
            16,
        )
    estimate_views = {}
    estimate_markers = {}
    for index, identity in enumerate(identities):
        letter = chr(65 + index)
        points = [
            estimated_relative_position(replay, identity, row.observation)
            for row in replay.samples[identity]
        ]
        points = [point for point in points if point is not None]
        mission = replay.missions.get(identity)
        markers = ((0, 0, 0), (*mission["planned_goal_offset_enu_m"], 0)) if mission else None
        if markers:
            points.extend(markers)
        viewport = ReplayViewport(bounds(points, 0.8), (1080, 205 + index * 260, 475, 215))
        estimate_views[identity], estimate_markers[identity] = viewport, markers
        axes(viewport)
        text(
            letter + "  Estimated local ENU",
            1080,
            185 + index * 260,
            21,
            vehicle_materials[identity],
        )
        planned_symbols(identity, viewport, markers, letter, 6)
        if scene_mode:
            planned_symbols(identity, main_view, planned_scene[identity], letter)
    text(replay.manifest.get("title", "Recorded mission replay"), 32, 49, 32)
    text("SCHEMATIC RECONSTRUCTION OF RECORDED DATA  |  NOT CAMERA FOOTAGE", 32, 91, 21)
    text(
        "RECORDED SIMULATOR SCENE" if scene_mode else "COMMON SCENE POSES NOT RETAINED", 32, 165, 25
    )
    text(
        "World ENU top view; retained obstacle footprints; metre grid"
        if scene_mode
        else "Use the independent estimated-pose panels at right; no scene alignment inferred",
        32,
        190,
        17,
    )
    text(
        "Recorded home offsets; resets and gaps remain.",
        1080,
        710,
        15,
    )
    text(
        "Trails appear with the observations; bright tails show the last 8 s. HOME / WP1 are planned targets.",
        32,
        881,
        17,
    )
    if scene_mode:
        line([(65, 645), (105, 645), (99, 641), (105, 645), (99, 649)], white, 1.5)
        line([(65, 645), (65, 605), (61, 611), (65, 605), (69, 611)], white, 1.5)
        text("E", 112, 650, 15)
        text("N", 59, 596, 15)
    for index, identity in enumerate(identities):
        text("EST H", 724 + index * 800, 723, 13)
        text("0..5 m", 724 + index * 800, 853, 12)
    camera_data = bpy.data.cameras.new("schematic camera")
    camera = bpy.data.objects.new("schematic camera", camera_data)
    scene.collection.objects.link(camera)
    camera.location = (width / 2, height / 2, 1000)
    camera_data.type, camera_data.ortho_scale = "ORTHO", width
    camera_data.clip_end = 2000
    scene.camera = camera
    scene.render.filepath = str(directory / "background.png")
    bpy.ops.render.render(write_still=True)

    overlay = ReplayOverlay(directory / "trajectory-overlay.ass", width, height)
    trail_counts = {}
    try:
        for index, identity in enumerate(identities):
            count = progressive_trail(
                overlay,
                replay,
                identity,
                estimate_views[identity],
                scene=False,
                colour=ass_colours[index],
            )
            if scene_mode:
                count += progressive_trail(
                    overlay, replay, identity, main_view, scene=True, colour=ass_colours[index]
                )
            trail_counts[identity] = count
        for frame in range(frame_count):
            elapsed = frame / fps
            end = min((frame + 1) / fps, replay.duration_seconds)
            monotonic = replay.started_monotonic + elapsed
            overlay.label(
                elapsed,
                end,
                32,
                112,
                f"CAPTURE TIME  {elapsed:6.1f} / {replay.duration_seconds:.1f} s",
                24,
            )
            for index, identity in enumerate(identities):
                letter, colour = chr(65 + index), ass_colours[index]
                sample = replay.at(identity, elapsed)
                observation = sample.observation if sample else {}
                pose, aiding = observation.get("pose"), observation.get("aiding") or {}
                mission = observation.get("mission") or {}
                pose_fresh = pose is not None and fresh_received(
                    pose.get("received_monotonic"), monotonic
                )
                scene_fresh = (
                    scene_mode
                    and observation.get("evaluation_truth") is not None
                    and fresh_received(
                        observation.get("evaluation_truth_received_monotonic"), monotonic
                    )
                )
                if pose_fresh:
                    point = estimated_relative_position(replay, identity, observation)
                    overlay.quadrotor(
                        elapsed,
                        end,
                        estimate_views[identity].project(point),
                        pose["yaw_enu_rad"],
                        colour,
                        letter,
                        10,
                    )
                if scene_fresh:
                    point = replay.scene_position(observation)
                    overlay.quadrotor(
                        elapsed,
                        end,
                        main_view.project(point),
                        scene_yaw(observation),
                        colour,
                        letter,
                        18,
                    )
                source_fresh = (
                    fresh_received(aiding.get("received_monotonic"), monotonic, 0.5)
                    and aiding.get("source_samples_fresh") is not False
                )

                def switch(value):
                    return "ON" if value is True else "OFF" if value is False else "?"

                def flag(value):
                    return "1" if value is True else "0" if value is False else "?"

                state = mission.get("state", "not captured")
                if mission.get("leg"):
                    state += " / " + mission["leg"]
                lines = [
                    f"{letter}  {replay.label(identity)} ({identity})  |  {state}",
                    f"GPS request {switch(aiding.get('gps_requested_enabled'))} | injected loss "
                    + (
                        "YES"
                        if aiding.get("gps_loss_injected") is True
                        else "NO"
                        if aiding.get("gps_loss_injected") is False
                        else "?"
                    )
                    + f" | fix {aiding.get('gps_fix_type', '?')} / {aiding.get('satellites_used', '?')} sats",
                ]
                if source_fresh:
                    lines.append(
                        f"Measured: GNSS XY {switch(aiding.get('gnss_xy_active'))} / VEL {switch(aiding.get('gnss_velocity_active'))}"
                        + f" | VISION XY {switch(aiding.get('vision_xy_active'))}"
                    )
                    lines.append(
                        "Height aiding GNSS / VISION / BARO: "
                        + "/".join(
                            flag(aiding.get(key))
                            for key in (
                                "gnss_height_active",
                                "vision_height_active",
                                "barometric_height_active",
                            )
                        )
                        + "  (1 active, 0 inactive)"
                    )
                else:
                    lines.extend(
                        [
                            "Measured aiding stale / unknown",
                            "No healthy-state inference from requests",
                        ]
                    )
                simulation = pose.get("simulation_timestamp_seconds") if pose else None
                lines.append(
                    (
                        f"Native sim {simulation:.2f} s | segment {sample.segment}"
                        if simulation is not None and pose_fresh
                        else "Estimated pose stale / unavailable"
                    )
                )
                x = 32 + index * 800
                for line_index, label in enumerate(lines):
                    overlay.label(
                        elapsed, end, x, 735 + line_index * 25, label, 19 if line_index else 21
                    )
                estimated = (
                    estimated_relative_position(replay, identity, observation)
                    if pose_fresh
                    else None
                )
                height_m = estimated[2] if estimated else None
                if height_m is not None:
                    gauge_x = x + 704
                    fill = max(0, min(1, height_m / 5)) * 75
                    overlay.polygon(
                        elapsed,
                        end,
                        [(gauge_x, 758), (gauge_x + 12, 758), (gauge_x + 12, 833), (gauge_x, 833)],
                        "504840",
                        3,
                    )
                    overlay.polygon(
                        elapsed,
                        end,
                        [
                            (gauge_x, 833 - fill),
                            (gauge_x + 12, 833 - fill),
                            (gauge_x + 12, 833),
                            (gauge_x, 833),
                        ],
                        colour,
                        4,
                    )
                    overlay.label(elapsed, end, gauge_x - 12, 736, f"{height_m:.1f} m", 18)
                if frame % 100 == 0:
                    progress = directory / "render-progress.json"
                    temporary = progress.with_suffix(".tmp")
                    temporary.write_text(
                        json.dumps({"completed_frames": frame + 1, "total_frames": frame_count})
                    )
                    temporary.replace(progress)
    finally:
        overlay.close()
    (directory / "render-manifest.json").write_text(
        json.dumps(
            {
                "blender_version": bpy.app.version_string,
                "blender_cpu_background_frames": 1,
                "frames": frame_count,
                "fps": fps,
                "width": width,
                "height": height,
                "requested_interval_seconds": replay.duration_seconds,
                "kind": "schematic recorded simulator poses and independent local estimates",
                "camera_video_recorded": False,
                "scene_geometry_reconstructed": bool(footprints) and scene_mode,
                "scene_representation": "retained SDF collision footprints in world ENU; textures omitted",
                "recorded_scene_pose_playback": scene_mode,
                "retained_obstacle_count": len(footprints),
                "visible_obstacle_count": visible_footprints,
                "shared_launch_binding": replay.shared_frame,
                "estimated_vehicle_ids": identities,
                "planned_markers_complete": all(estimate_markers.values()),
                "progressive_trail_segments": trail_counts,
                "bright_tail_seconds": 8,
                "vehicle_symbol": "rotated quadrotor schematic; main 48 px rotor span; inset 32 px",
                "frame_time_quantization_seconds": 1 / fps,
                "temporal_method": "latest actual observation; no extrapolation; hide poses after 0.3 wall seconds",
                "height_gauge": "estimated height above recorded home, 0..5 m display; value remains explicit",
                "render_method": "Blender CPU schematic plus FFmpeg/libass observed pose, progressive trails and status",
                "source_mission_hashes": {
                    key: value.get("sha256") for key, value in replay.missions.items()
                },
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
