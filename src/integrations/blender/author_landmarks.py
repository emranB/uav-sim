"""Author small visual-only ENU launch-pad trim assets; run with optional Blender."""

import hashlib
import json
import math
import sys
from pathlib import Path


def main() -> None:
    import bpy
    from mathutils import Vector

    destination = Path(sys.argv[sys.argv.index("--") + 1])
    destination.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    vertices, faces = [], []
    segments = 96
    for index in range(segments):
        angle = 2 * math.pi * index / segments
        for radius in (1.63, 1.66):
            vertices.append((radius * math.cos(angle), radius * math.sin(angle), 0.030))
    for index in range(segments):
        a, b = 2 * index, 2 * ((index + 1) % segments)
        faces.append((a, a + 1, b + 1, b))
    mesh = bpy.data.meshes.new("pad trim")
    mesh.from_pydata(vertices, [], faces)
    trim = bpy.data.objects.new("pad trim", mesh)
    bpy.context.scene.collection.objects.link(trim)
    label_vertices, label_faces = [], []

    def stroke(x1, y1, x2, y2, width=0.018):
        length = math.hypot(x2 - x1, y2 - y1)
        nx, ny = -(y2 - y1) / length * width / 2, (x2 - x1) / length * width / 2
        offset = len(label_vertices)
        label_vertices.extend(
            [
                (x1 + nx, y1 + ny, 0.032),
                (x1 - nx, y1 - ny, 0.032),
                (x2 - nx, y2 - ny, 0.032),
                (x2 + nx, y2 + ny, 0.032),
            ]
        )
        label_faces.append(tuple(offset + index for index in range(4)))

    # Original geometric E/N strokes avoid external font geometry in distributed meshes.
    stroke(1.49, -0.10, 1.49, 0.08)
    for y in (-0.10, -0.01, 0.08):
        stroke(1.49, y, 1.60, y)
    stroke(-0.07, 1.45, -0.07, 1.63)
    stroke(0.05, 1.45, 0.05, 1.63)
    stroke(-0.07, 1.63, 0.05, 1.45)
    label_mesh = bpy.data.meshes.new("direction strokes")
    label_mesh.from_pydata(label_vertices, [], label_faces)
    label_object = bpy.data.objects.new("direction strokes", label_mesh)
    bpy.context.scene.collection.objects.link(label_object)
    labels = [label_object]

    def export(name, objects, colour):
        positions, triangles = [], []
        for item in objects:
            item.data.calc_loop_triangles()
            offset = len(positions)
            positions.extend(
                tuple(item.matrix_world @ Vector(vertex.co)) for vertex in item.data.vertices
            )
            triangles.extend(
                tuple(index + offset + 1 for index in face.vertices)
                for face in item.data.loop_triangles
            )
        if len(triangles) > 5000:
            raise ValueError("Landmark mesh exceeds its triangle budget")
        lines = ["v " + " ".join(f"{value:.8f}" for value in point) for point in positions]
        lines.extend("f " + " ".join(str(index) for index in face) for face in triangles)
        path = destination / f"{name}.obj"
        path.write_text("\n".join(lines) + "\n")
        return {
            "file": path.name,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "vertices": len(positions),
            "triangles": len(triangles),
            "bytes": path.stat().st_size,
            "colour_rgba": colour,
        }

    assets = [
        export("pad_trim", [trim], [0.65, 0.88, 0.82, 1]),
        export("pad_labels", labels, [0.88, 0.96, 1, 1]),
    ]
    (destination / "manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "coordinate_frame": "ENU metres; Z up",
                "blender_version": bpy.app.version_string,
                "source_recipe": "src/integrations/blender/author_landmarks.py",
                "license": "CC0-1.0",
                "collision_geometry": False,
                "assets": assets,
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
