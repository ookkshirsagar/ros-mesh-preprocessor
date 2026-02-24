"""
urdf_export.py - Generate visual + collision mesh pairs ready for URDF / MoveIt.

Usage (programmatic):
    from optimizer.urdf_export import export_urdf_pair

    export_urdf_pair(
        input_path="robot_cell.stl",
        output_dir="output/",
        visual_triangles=20_000,
        collision_triangles=3_000,
    )

This produces:
    output/robot_cell_visual.stl
    output/robot_cell_collision.stl
    output/robot_cell_urdf_snippet.xml   ← drop-in <geometry> tags
"""

import xml.dom.minidom as minidom
from pathlib import Path

from optimizer.core import clean_mesh, load_mesh, save_mesh, simplify_mesh
from optimizer.metrics import build_report, print_report, save_report


def export_urdf_pair(
    input_path: str,
    output_dir: str,
    visual_triangles: int,
    collision_triangles: int,
    save_reports: bool = True,
    compute_hausdorff: bool = True,
) -> dict:
    """
    Create a visual mesh and a collision mesh from a single source mesh.

    Returns a dict with paths and metrics for both outputs.
    """
    stem = Path(input_path).stem
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    visual_path = str(out / f"{stem}_visual.stl")
    collision_path = str(out / f"{stem}_collision.stl")

    print(f"[urdf_export] Loading '{input_path}' …")
    mesh = load_mesh(input_path)
    mesh = clean_mesh(mesh)

    print(f"[urdf_export] Generating visual mesh  ({visual_triangles:,} triangles) …")
    visual_mesh, vis_time = simplify_mesh(mesh, visual_triangles)
    save_mesh(visual_mesh, visual_path)

    print(f"[urdf_export] Generating collision mesh ({collision_triangles:,} triangles) …")
    collision_mesh, col_time = simplify_mesh(mesh, collision_triangles)
    save_mesh(collision_mesh, collision_path)

    vis_report = build_report(
        input_path, visual_path, mesh, visual_mesh,
        vis_time, compute_hausdorff=compute_hausdorff,
    )
    col_report = build_report(
        input_path, collision_path, mesh, collision_mesh,
        col_time, compute_hausdorff=compute_hausdorff,
    )

    print("\n[Visual Mesh]")
    print_report(vis_report)
    print("[Collision Mesh]")
    print_report(col_report)

    if save_reports:
        save_report(vis_report, str(out / f"{stem}_visual_report.json"))
        save_report(col_report, str(out / f"{stem}_collision_report.json"))

    _write_urdf_snippet(stem, visual_path, collision_path, out)

    return {"visual": vis_report, "collision": col_report}


def _write_urdf_snippet(
    stem: str,
    visual_path: str,
    collision_path: str,
    out: Path,
) -> None:
    """Write a ready-to-paste URDF <link> geometry snippet."""
    snippet = f"""<link name="{stem}">

  <visual>
    <geometry>
      <mesh filename="package://YOUR_PACKAGE/meshes/{Path(visual_path).name}" scale="1 1 1"/>
    </geometry>
  </visual>

  <collision>
    <geometry>
      <mesh filename="package://YOUR_PACKAGE/meshes/{Path(collision_path).name}" scale="1 1 1"/>
    </geometry>
  </collision>

</link>"""

    snippet_path = out / f"{stem}_urdf_snippet.xml"
    snippet_path.write_text(snippet)
    print(f"[urdf_export] URDF snippet saved → {snippet_path}")
