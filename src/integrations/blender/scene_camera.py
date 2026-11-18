"""Qualify a small set of chase views against the retained scene and sealed poses."""

import math

from common.logging.replay_timeline import RecordedReplay
from integrations.blender.camera_policy import ChaseCameraState, advance_chase_camera
from integrations.blender.overlay import fresh_received


def plan_chase_camera(
    replay: RecordedReplay,
    identity: str,
    start_seconds: float,
    frame_count: int,
    fps: int,
    world_meshes: list,
    width: int,
    height: int,
    outline_points: list[tuple],
) -> tuple[list[ChaseCameraState | None], dict]:
    """Require solid-scene clearance; retain and separately report thin pad guide crossings."""
    from mathutils import Quaternion, Vector
    from mathutils.bvhtree import BVHTree

    reference_names = {"pad_trim", "pad_labels"}

    def geometry(meshes: list) -> tuple:
        vertices, polygons, owners = [], [], []
        for mesh in meshes:
            base = len(vertices)
            vertices.extend(mesh.matrix_world @ vertex.co for vertex in mesh.data.vertices)
            polygons.extend(
                tuple(base + index for index in polygon.vertices) for polygon in mesh.data.polygons
            )
            owners.extend(mesh.name for polygon in mesh.data.polygons)
        return BVHTree.FromPolygons(vertices, polygons, epsilon=0.01), owners

    retained_world, owners = geometry(world_meshes)
    solids, solid_owners = geometry(
        [mesh for mesh in world_meshes if mesh.name.split(".", 1)[0] not in reference_names]
    )
    recorded = []
    for frame in range(frame_count):
        elapsed = start_seconds + frame / fps
        sample = replay.at(identity, elapsed)
        valid = bool(
            sample
            and elapsed - sample.elapsed_wall_seconds <= 0.3
            and fresh_received(
                sample.observation.get("evaluation_truth_received_monotonic"),
                replay.started_monotonic + elapsed,
            )
        )
        if not valid:
            recorded.append(None)
            continue
        truth = sample.observation["evaluation_truth"]
        x, y, z, w = truth["orientation_xyzw"]
        heading = math.atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))
        recorded.append((tuple(truth["position_enu_m"]), heading, Quaternion((w, x, y, z))))
    tested = []
    # The reviewed behind/side view is first. Alternatives are only for a real obstacle.
    presets = (
        (5.5, 4, -3),
        (5.5, 6, -3),
        (5.5, 8, -3),
        (5.5, 12, -3),
        (3, 16, -2),
        (5.5, 4, 3),
        (5.5, 8, 3),
        (5.5, 12, 3),
        (3, 16, 2),
        (3, 12, 6),
        (3, 16, 6),
        (1, 20, 0),
        (3, 4, -3),
        (1, 4, -3),
        (-5.5, 4, 0),
        (-5.5, 6, 0),
        (-5.5, 4, 3),
        (-5.5, 6, 3),
        (-5.5, 12, 3),
        (-5.5, 16, 3),
        (-3, 4, 3),
        (0, 8, 0),
    )
    for back_m, height_m, side_m in presets:
        previous = None
        states, solid_frames, guide_frames, center_frames, margins = [], [], [], [], []
        intersections = {}
        for frame, observation in enumerate(recorded):
            if observation is None:
                states.append(previous)
                continue
            position, heading, orientation = observation
            previous = advance_chase_camera(
                position, heading, previous, side_m, height_m, fps, back_m
            )
            states.append(previous)
            origin = Vector(previous.position)
            forward = (Vector(previous.target) - origin).normalized()
            solid_blocked, guide_crossing = False, False
            for ray_index, local in enumerate(outline_points):
                endpoint = Vector(position) + orientation @ Vector(local)
                # Orthographic pixels cast parallel rays, not rays from the camera centre.
                # Matching the actual projection prevents a clear perspective ray from
                # falsely certifying an aircraft hidden by a foreground building.
                depth = (endpoint - origin).dot(forward)
                ray_origin = endpoint - forward * depth
                ray = forward
                distance = max(0, depth - 0.2)
                hit = retained_world.ray_cast(ray_origin, ray, distance)
                if hit[0] is not None:
                    name = owners[hit[2]]
                    intersections[name] = intersections.get(name, 0) + 1
                    if ray_index == 0:
                        center_frames.append(frame)
                    guide_crossing = guide_crossing or name.split(".", 1)[0] in reference_names
                # A thin reference must not hide a solid obstacle from the clearance check.
                opaque = solids.ray_cast(ray_origin, ray, distance)
                solid_blocked = solid_blocked or opaque[0] is not None
            if solid_blocked:
                solid_frames.append(frame)
            if guide_crossing:
                guide_frames.append(frame)
            forward = (Vector(previous.target) - origin).normalized()
            right = forward.cross(Vector((0, 0, 1))).normalized()
            up = right.cross(forward).normalized()
            relative = Vector(position) - origin
            pixel_x = width / 2 + relative.dot(right) * width / 9.5
            pixel_y = height / 2 - relative.dot(up) * width / 9.5
            margins.append(min(pixel_x, pixel_y, width - pixel_x, height - pixel_y))
        result = {
            "back_m": back_m,
            "side_m": side_m,
            "height_m": height_m,
            "solid_occluded_frames": solid_frames,
            "guide_intersection_frames": guide_frames,
            "center_intersection_frames": sorted(set(center_frames)),
            "minimum_root_margin_px": min(margins),
            "intersected_visuals": intersections,
        }
        tested.append(result)
        if not solid_frames and min(margins) >= 40:
            return states, {
                "schema_version": 1,
                "selected": result,
                "tested_candidates": tested,
                "world_meshes": len(world_meshes),
                "timeline_frames": frame_count,
                "rays_per_pose": len(outline_points),
                "aircraft_outline_points_local_m": outline_points,
                "guide_crossings_require_pixel_review": bool(guide_frames),
                "selection": (
                    "Front three-quarter follow"
                    if back_m < 0
                    else "Overhead follow"
                    if back_m == 0 and side_m == 0
                    else "Behind/side chase"
                )
                + "; constant offset with solid geometry clearance",
                "geometry_note": "Pad reference meshes remain fully rendered and their crossings are reported",
                "pose_scope": "Camera only; aircraft and world geometry unchanged",
            }
    raise RuntimeError(f"No clear chase view for {identity}; retained geometry unchanged; {tested}")
