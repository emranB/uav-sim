"""Run with Blender to verify visible geometry stops at the last observed position."""

import json
import sys
from pathlib import Path


def main() -> None:
    """Compare production arc-length clipping against an intentionally wrong control."""
    import bpy

    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from integrations.blender.scene_geometry import recorded_trail_curve

    result = {}
    for mapping in ("RESOLUTION", "production"):
        curve = recorded_trail_curve(mapping, [(0, 0, 0), (1, 0, 0), (10, 0, 0), (11, 0, 0)])
        if mapping == "RESOLUTION":
            curve.bevel_factor_mapping_end = "RESOLUTION"
        curve.bevel_factor_end = 10 / 11
        instance = bpy.data.objects.new(mapping, curve)
        bpy.context.scene.collection.objects.link(instance)
        bpy.context.view_layer.update()
        evaluated = instance.evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh = evaluated.to_mesh()
        maximum_x = max(vertex.co.x for vertex in mesh.vertices)
        evaluated.to_mesh_clear()
        result[mapping] = {
            "maximum_x_m": maximum_x,
            "latest_observed_x_m": 10,
            "no_future_vertices": maximum_x <= 10.013,
        }
    assert not result["RESOLUTION"]["no_future_vertices"], result
    assert result["production"]["no_future_vertices"], result
    destination = Path(sys.argv[sys.argv.index("--") + 1])
    destination.write_text(json.dumps({"passed": True, "geometry": result}, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
