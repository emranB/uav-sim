"""Offline 3D reconstruction from sealed replay observations and retained scene inputs."""

import bisect
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def main() -> None:
    """Render a bounded sealed interval using explicit offline device settings."""
    import bpy
    import gpu
    from mathutils import Euler, Matrix, Vector

    args = json.loads(Path(sys.argv[sys.argv.index("--") + 1]).read_text())
    sys.path.insert(0, args["source_python"])
    from common.logging.replay_timeline import RecordedReplay
    from integrations.blender.overlay import fresh_received
    from integrations.blender.scene_geometry import recorded_trail_curve

    replay = RecordedReplay(Path(args["export_path"]))
    if len(replay.samples) > 2:
        raise ValueError("This readable 3D scene supports at most two selected aircraft")
    assert replay.scene_available
    out = Path(args["output"])
    out.mkdir(exist_ok=True, parents=True)
    assets = Path(args["assets"])
    start = float(args.get("start_seconds", 0))
    end = min(float(args.get("end_seconds", replay.duration_seconds)), replay.duration_seconds)
    fps = int(args.get("fps", 10))
    count = math.ceil((end - start) * fps - 1e-8)
    assert 0 < count <= 18000
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.render.engine = args.get("engine", "CYCLES")
    scene.eevee.taa_render_samples = int(args.get("samples", 16))
    scene.cycles.samples = int(args.get("samples", 16))
    scene.cycles.use_denoising = False
    scene.render.use_persistent_data = True
    scene.render.resolution_x = int(args.get("width", 1280))
    scene.render.resolution_y = int(args.get("height", 720))
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    # Frames are transient pipe inputs; video encoding supplies final compression.
    scene.render.image_settings.compression = 0
    scene.render.threads_mode = "FIXED"
    scene.render.threads = 4
    scene.view_settings.view_transform = "AgX"
    scene.world.color = (0.22, 0.25, 0.3)
    compute = args.get("compute", "CPU")
    if compute != "CPU":
        prefs = bpy.context.preferences.addons["cycles"].preferences
        prefs.compute_device_type = compute
        prefs.get_devices()
        for dev in prefs.devices:
            dev.use = dev.type == compute
        if not any((d.use for d in prefs.devices)):
            raise RuntimeError("Requested offline render device unavailable")
    scene.cycles.device = "CPU" if compute == "CPU" else "GPU"
    scene.render.fps = fps
    scene.frame_start = 1
    scene.frame_end = count
    materials = {}

    def mat(name, colour, emission=False):
        key = (name, tuple(colour), emission)
        if key in materials:
            return materials[key]
        m = bpy.data.materials.new(name)
        m.diffuse_color = (*colour[:3], 1)
        m.use_nodes = True
        n = m.node_tree.nodes.get("Principled BSDF")
        n.inputs["Base Color"].default_value = (*colour[:3], 1)
        n.inputs["Roughness"].default_value = 0.66
        if emission:
            m.node_tree.nodes.remove(n)
            n = m.node_tree.nodes.new("ShaderNodeEmission")
            n.inputs["Color"].default_value = (*colour[:3], 1)
            m.node_tree.links.new(
                n.outputs[0], m.node_tree.nodes.get("Material Output").inputs["Surface"]
            )
        materials[key] = m
        return m

    colours = [(0.08, 0.8, 0.57), (0.18, 0.48, 1)]
    colormats = [
        mat("Atlas annotation", colours[0], True),
        mat("Boreal annotation", colours[1], True),
    ]
    gold = mat("planned marker", (0.95, 0.64, 0.13), True)

    def transform(text):
        v = [float(n) for n in (text or "0 0 0 0 0 0").split()]
        assert len(v) == 6
        return Matrix.Translation(v[:3]) @ Euler(v[3:], "XYZ").to_matrix().to_4x4()

    def primitive(name, g):
        kind = g[0].tag
        n = g[0]
        if kind == "box":
            bpy.ops.mesh.primitive_cube_add(size=1)
            o = bpy.context.object
            o.scale = [float(v) for v in n.findtext("size").split()]
        elif kind == "plane":
            bpy.ops.mesh.primitive_plane_add(size=1)
            o = bpy.context.object
            o.scale = (*[float(v) for v in n.findtext("size").split()], 1)
        elif kind == "cylinder":
            bpy.ops.mesh.primitive_cylinder_add(
                vertices=48, radius=float(n.findtext("radius")), depth=float(n.findtext("length"))
            )
            o = bpy.context.object
        elif kind == "sphere":
            bpy.ops.mesh.primitive_uv_sphere_add(
                segments=16, ring_count=8, radius=float(n.findtext("radius"))
            )
            o = bpy.context.object
        else:
            return None
        o.name = name
        bpy.context.view_layer.update()
        return o

    meshcache = {}

    def import_mesh(path):
        path = path.resolve()
        if path in meshcache:
            copies = []
            for old, baseline in meshcache[path]:
                o = old.copy()
                o.data = old.data
                o.parent = None
                o.matrix_world = baseline.copy()
                scene.collection.objects.link(o)
                copies.append(o)
            return copies
        before = set(bpy.data.objects)
        if path.suffix == ".dae":
            bpy.ops.wm.collada_import(filepath=str(path))
        elif path.suffix == ".stl":
            bpy.ops.wm.stl_import(filepath=str(path))
        elif path.suffix == ".obj":
            # Prepared geographic and presentation meshes are already ENU / Z-up.
            bpy.ops.wm.obj_import(filepath=str(path), forward_axis="Y", up_axis="Z")
        else:
            raise ValueError(f"Unsupported retained mesh {path}")
        imported = list(set(bpy.data.objects) - before)
        bpy.context.view_layer.update()
        objs = [o for o in imported if o.type == "MESH"]
        assert objs
        for o in imported:
            if o.type != "MESH":
                bpy.data.objects.remove(o, do_unlink=True)
        baselines = [(o, o.matrix_world.copy()) for o in objs]
        for o, baseline in baselines:
            o.parent = None
            o.matrix_world = baseline
        meshcache[path] = baselines
        return objs

    for name, contents in replay.context.get("retained_presentation_assets", {}).items():
        if name.endswith(".obj"):
            (assets / name).write_text(contents)

    def visual(visual, base, parent=None, model=False):
        geom = visual.find("geometry")
        assert geom is not None
        matrix = base @ transform(visual.findtext("pose"))
        if geom[0].tag == "mesh":
            uri = geom.findtext("mesh/uri")
            path = (
                assets / "x500_base" / uri.split("x500_base/", 1)[1]
                if uri.startswith("model://x500_base/")
                else assets / Path(uri).name
            )
            assert path.is_file(), path
            objs = import_mesh(path)
            scale = [float(n) for n in geom.findtext("mesh/scale", "1 1 1").split()]
            matrix = matrix @ Matrix.Diagonal((*scale, 1))
        else:
            objs = [primitive(visual.get("name", "visual"), geom)]
        colour = [float(n) for n in visual.findtext("material/diffuse", "0.24 0.29 0.34 1").split()]
        material = mat("retained visual " + str(colour), colour)
        albedo = visual.findtext("material/pbr/metal/albedo_map")
        if albedo and (assets / Path(albedo).name).is_file():
            material = material.copy()
            image = bpy.data.images.load(str(assets / Path(albedo).name), check_existing=True)
            node = material.node_tree.nodes.new("ShaderNodeTexImage")
            node.image = image
            material.node_tree.links.new(
                node.outputs["Color"],
                material.node_tree.nodes.get("Principled BSDF").inputs["Base Color"],
            )
        for o in objs:
            assert o is not None
            local = o.matrix_world.copy()
            o.parent = parent
            o.matrix_local = matrix @ local
            if o.type == "MESH" and (not model or not o.data.materials):
                o.data.materials.clear()
                o.data.materials.append(material)

    world = ET.fromstring(replay.context["retained_world_sdf"]["contents"]).find("world")
    world_visuals = 0
    for model in world.findall("model"):
        for link in model.findall("link"):
            for v in link.findall("visual"):
                visual(v, transform(model.findtext("pose")) @ transform(link.findtext("pose")))
                world_visuals += 1
    light = world.find("light")
    ld = bpy.data.lights.new("retained sun direction", "SUN")
    ld.energy = 2.1
    lo = bpy.data.objects.new("retained sun", ld)
    scene.collection.objects.link(lo)
    direction = Vector([float(x) for x in light.findtext("direction").split()])
    lo.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    ld.angle = 0.12
    ld = bpy.data.lights.new("presentation fill", "AREA")
    ld.energy = 1100
    ld.shape = "DISK"
    ld.size = 12
    lo = bpy.data.objects.new("presentation fill", ld)
    scene.collection.objects.link(lo)
    lo.location = (3, -4, 12)
    camdata = bpy.data.cameras.new("replay camera")
    camera = bpy.data.objects.new("replay camera", camdata)
    scene.collection.objects.link(camera)
    camera.location = (10, -8, 18)
    target = Vector((4, 1.2, 1.8))
    camera.rotation_euler = (target - camera.location).to_track_quat("-Z", "Y").to_euler()
    camdata.type = "ORTHO"
    camdata.ortho_scale = 16
    camdata.lens = 40
    scene.camera = camera
    # Frame the complete recorded volume inside the unobscured video area. This is
    # offline camera composition; future samples never become visible flight poses.
    bpy.context.view_layer.update()
    framing_points = [
        replay.scene_position(sample.observation)
        for rows in replay.samples.values()
        for sample in rows
    ]
    framing_points = [Vector(point) for point in framing_points if point is not None]
    for identity in replay.samples:
        markers = replay.planned_markers(identity, scene_frame=True)
        if markers is None:
            continue
        home, goal = markers
        framing_points.append(Vector((goal[0], goal[1], 0.1)))
        framing_points.extend(
            Vector((home[0] + x, home[1] + y, 0.1)) for x in (-1.85, 1.85) for y in (-1.85, 1.85)
        )
    local_points = [camera.matrix_world.inverted() @ point for point in framing_points]
    x_min, x_max = min(p.x for p in local_points), max(p.x for p in local_points)
    y_min, y_max = min(p.y for p in local_points), max(p.y for p in local_points)
    camdata.ortho_scale = max(
        14, (x_max - x_min + 1.6) / 0.9, (y_max - y_min + 1.6) * (16 / 9) / 0.64
    )
    view_height = camdata.ortho_scale * 9 / 16
    camera.location += camera.rotation_euler.to_matrix() @ Vector(
        ((x_min + x_max) / 2, (y_min + y_max) / 2 - 0.0125 * view_height, 0)
    )
    bpy.context.view_layer.update()
    camera_fit = {
        "framing_points": len(framing_points),
        "source_local_bounds": [x_min, x_max, y_min, y_max],
        "safe_vertical_fraction": 0.64,
        "margin_m": 0.8,
        "method": "Complete recorded positions and retained launch-pad extents",
    }

    static_world_meshes = [item for item in bpy.data.objects if item.type == "MESH"]
    vehicles = {}
    trails = {}
    roots = {}
    vehicle_sdf = ET.parse(assets / "x500_base/model.sdf").find("model")
    for index, identity in enumerate(replay.samples):
        root = bpy.data.objects.new(identity, None)
        scene.collection.objects.link(root)
        root.rotation_mode = "QUATERNION"
        roots[identity] = root
        for link in vehicle_sdf.findall("link"):
            for v in link.findall("visual"):
                visual(
                    v,
                    transform(vehicle_sdf.findtext("pose")) @ transform(link.findtext("pose")),
                    root,
                    True,
                )
        meshes = [o for o in bpy.data.objects if o.parent == root]
        vehicles[identity] = meshes
        bpy.ops.mesh.primitive_torus_add(
            major_radius=0.4, minor_radius=0.018, major_segments=32, minor_segments=6
        )
        halo = bpy.context.object
        halo.name = identity + " identity annotation"
        halo.parent = root
        halo.location = (0, 0, 0.1)
        halo.data.materials.append(colormats[index])
        meshes.append(halo)
        markers = replay.planned_markers(identity, scene_frame=True)
        marker_specs = []
        if markers is not None:
            home, goal = markers
            marker_specs = [("HOME", home, colormats[index]), ("WP1 planned", goal, gold)]
        for title, p, material in marker_specs:
            bpy.ops.mesh.primitive_torus_add(
                major_radius=0.45, minor_radius=0.025, major_segments=32, minor_segments=6
            )
            o = bpy.context.object
            o.name = identity + " " + title
            o.location = (p[0], p[1], 0.07)
            o.data.materials.append(material)
        segments = []
        previous = None
        for sample in replay.samples[identity]:
            o = sample.observation
            p = replay.scene_position(o)
            stamp = o.get("evaluation_truth_received_monotonic")
            key = (o["world_epoch"], o["controller_generation"])
            valid = p is not None and fresh_received(
                stamp, replay.started_monotonic + sample.elapsed_wall_seconds
            )
            if not valid:
                previous = None
                continue
            if (
                previous is None
                or key != previous[0]
                or sample.elapsed_wall_seconds - previous[1] > 0.5
            ):
                segments.append([])
            segments[-1].append((sample.elapsed_wall_seconds, p))
            previous = (key, sample.elapsed_wall_seconds)
        entries = []
        for si, segment in enumerate(segments):
            if len(segment) < 2:
                continue
            curve = recorded_trail_curve(
                identity + " observed trail", [position for _, position in segment]
            )
            lengths = [0.0]
            for j, (stamp, p) in enumerate(segment):
                if j:
                    lengths.append(lengths[-1] + (Vector(p) - Vector(segment[j - 1][1])).length)
            ob = bpy.data.objects.new(identity + " progressive trail " + str(si), curve)
            scene.collection.objects.link(ob)
            curve.materials.append(colormats[index])
            entries.append((ob, [s[0] for s in segment], lengths))
        trails[identity] = entries
    from bpy_extras.object_utils import world_to_camera_view

    from common.logging.storage import write_json_atomic
    from integrations.blender.scene_inputs import write_scene_annotations

    bpy.context.view_layer.update()

    def project(point: tuple) -> tuple[float, float]:
        projected = world_to_camera_view(scene, camera, Vector(point))
        return projected.x * 1280, (1 - projected.y) * 720

    focus_identity = args.get("focus_vehicle_id")
    if focus_identity is not None and focus_identity not in replay.samples:
        raise ValueError("Focus camera requires an explicitly captured vehicle")
    camera_states, camera_plan = [], None
    if focus_identity is not None:
        from integrations.blender.scene_camera import plan_chase_camera

        points = [
            mesh.matrix_world @ vertex.co
            for mesh in vehicles[focus_identity][:-1]
            for vertex in mesh.data.vertices
        ]
        outline = [(0, 0, 0.265)] + [
            tuple(select(points, key=lambda point: point[axis]))
            for axis in (0, 1)
            for select in (min, max)
        ]
        camera_states, camera_plan = plan_chase_camera(
            replay,
            focus_identity,
            start,
            count,
            fps,
            static_world_meshes,
            scene.render.resolution_x,
            scene.render.resolution_y,
            outline,
        )
        write_json_atomic(out / "camera-qualification.json", camera_plan)
    probe_frames = set(args.get("probe_frame_indices", []))
    if focus_identity is None:
        write_scene_annotations(
            replay, out / "scene-labels.ass", start, count, fps, args["title"], project
        )
    encode_log = (out / "scene-encode.log").open("xb")
    encoder = subprocess.Popen(
        [
            "ffmpeg",
            "-nostdin",
            "-v",
            "error",
            "-n",
            "-f",
            "image2pipe",
            "-vcodec",
            "png",
            "-framerate",
            str(fps),
            "-i",
            "pipe:0",
            *(["-vf", "ass=scene-labels.ass"] if focus_identity is None else []),
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "20",
            "-threads",
            "2",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(out / "scene.partial.mp4"),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=encode_log,
        cwd=out,
    )
    try:
        frame_evidence = []
        focus_pixels = []
        for frame in range(1, count + 1):
            elapsed = start + (frame - 1) / fps
            for index, identity in enumerate(replay.samples):
                sample = replay.at(identity, elapsed)
                o = sample.observation if sample else {}
                truth = o.get("evaluation_truth") or {}
                received = o.get("evaluation_truth_received_monotonic")
                visible = bool(
                    sample
                    and elapsed - sample.elapsed_wall_seconds <= 0.3
                    and fresh_received(received, replay.started_monotonic + elapsed)
                )
                root = roots[identity]
                for mesh in vehicles[identity]:
                    mesh.hide_render = not visible
                if visible:
                    root.location = truth["position_enu_m"]
                    (x, y, z, w) = truth["orientation_xyzw"]
                    norm = math.sqrt(x * x + y * y + z * z + w * w)
                    assert abs(norm - 1) < 0.01
                    root.rotation_quaternion = (w, x, y, z)
                for ob, times, lengths in trails[identity]:
                    j = bisect.bisect_right(times, elapsed) - 1
                    ob.hide_render = j < 1
                    ob.data.bevel_factor_end = (
                        lengths[max(0, j)] / lengths[-1] if lengths[-1] > 0 else 0
                    )
            if focus_identity is not None:
                focus = roots[focus_identity]
                state = camera_states[frame - 1]
                if state is not None:
                    camera.location = state.position
                    camera.rotation_euler = (
                        (Vector(state.target) - camera.location).to_track_quat("-Z", "Y").to_euler()
                    )
                    camdata.ortho_scale = 9.5
                    bpy.context.view_layer.update()
                pixel = world_to_camera_view(scene, camera, focus.location)
                focus_pixels.append(
                    [pixel.x * scene.render.resolution_x, (1 - pixel.y) * scene.render.resolution_y]
                )
            if probe_frames and frame not in probe_frames:
                continue
            scene.frame_set(frame)
            scene.render.filepath = str(out / "render-frame.png")
            bpy.ops.render.render(write_still=True)
            encoded = (out / "render-frame.png").read_bytes()
            if len(encoded) > 8 * 1024**2:
                raise ValueError("Rendered frame exceeds 8 MiB")
            encoder.stdin.write(encoded)
            if frame == 1 or (probe_frames and frame == min(probe_frames)):
                (out / "preview.png").write_bytes(encoded)
            if probe_frames:
                (out / f"probe-frame-{frame:05d}.png").write_bytes(encoded)
            (out / "render-frame.png").unlink()
            write_json_atomic(
                out / "render-progress.json", {"completed_frames": frame, "total_frames": count}
            )
            if frame in (1, count) or frame % 10 == 0:
                print("REPLAY_FRAME", frame, count, elapsed, flush=True)
            if frame in (1, count):
                frame_evidence.append({"frame": frame, "capture_wall_seconds": elapsed})
        encoder.stdin.close()
        if encoder.wait(timeout=30):
            raise RuntimeError("Scene video encoder failed; inspect scene-encode.log")
        (out / "scene.partial.mp4").rename(
            out / ("camera-probe.mp4" if probe_frames else "mission-replay.mp4")
        )
    finally:
        if encoder.poll() is None:
            encoder.terminate()
            encoder.wait(timeout=5)
        encode_log.close()
    graphics_renderer, graphics_vendor, graphics_query_error = None, None, None
    try:
        graphics_renderer = gpu.platform.renderer_get()
        graphics_vendor = gpu.platform.vendor_get()
    except SystemError as error:
        # Cycles CPU and headless GPU contexts may not expose drawing diagnostics.
        # This optional diagnostic must not turn a decoded reconstruction into a failure.
        graphics_query_error = str(error)
    (out / "render-manifest.json").write_text(
        json.dumps(
            {
                "passed": True,
                "kind": "Selected camera qualification frames"
                if probe_frames
                else "animated 3D Blender scene reconstruction",
                "probe_frame_indices": sorted(probe_frames),
                "camera_qualification": camera_plan,
                "recorded_scene_poses": True,
                "recorded_scene_quaternions": True,
                "world_visuals_imported": world_visuals,
                "frames": len(probe_frames) if probe_frames else count,
                "timeline_frames": count,
                "fps": fps,
                "start_seconds": start,
                "end_seconds": end,
                "duration_seconds": (len(probe_frames) if probe_frames else count) / fps,
                "device": compute,
                "engine": scene.render.engine,
                "graphics_renderer": graphics_renderer,
                "graphics_query_error": graphics_query_error,
                "camera_world_matrix": [list(row) for row in camera.matrix_world],
                "camera_orthographic_scale": camdata.ortho_scale,
                "camera_fit": camera_fit,
                "width": scene.render.resolution_x,
                "height": scene.render.resolution_y,
                "lighting": "Retained sun direction with explicit area fill; original world geometry/materials",
                "graphics_vendor": graphics_vendor,
                "samples": int(args.get("samples", 16)),
                "requested_interval_seconds": end - start,
                "blender_version": bpy.app.version_string,
                "scene_assets_manifest": "scene-assets/asset-manifest.json",
                "source_export": args["export_path"],
                "source_revision": replay.context["source_revision"],
                "frame_evidence": frame_evidence,
                "rotor_animation": False,
                "focus_vehicle_id": focus_identity,
                "focus_pixels": focus_pixels,
                "camera_follow": "Recorded quaternion heading; 0.8 rad/s yaw bound, 5 m/s translation bound; 0.45 s presentation smoothing",
                "rotor_note": "Rotor angular velocity was not captured; blades remain static rather than inventing motion",
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
