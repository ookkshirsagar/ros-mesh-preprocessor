"""
examples/moveit_cell_demo/run_demo.py

Demonstrates a full industrial robot cell preprocessing pipeline:

1. Load a high-polygon robot cell STL           (raw mesh)
2. Clean it (dedupe / degenerate triangle removal)
3. Generate a visual mesh (20 000 triangles), for RViz rendering
4. Generate a collision mesh (3 000 triangles), for MoveIt planning
5. Output a URDF <link> snippet
6. Run a MoveIt load check against the generated snippet + meshes
7. Print a benchmarking report with Hausdorff distance

Usage:
    python examples/moveit_cell_demo/run_demo.py

Note:
    The demo creates a synthetic sphere mesh if no real STL is present.
    Replace CELL_MESH_PATH to point at your actual robot cell model.
"""

import sys
from pathlib import Path

import open3d as o3d

# --- Path configuration ---------------------------------------------------
DEMO_DIR = Path(__file__).parent
OUTPUT_DIR = DEMO_DIR / "output"
CELL_MESH_PATH = DEMO_DIR / "robot_cell.stl"  # ← replace with your file

# Import here so the demo is self-contained even when run from this dir
sys.path.insert(0, str(DEMO_DIR.parent.parent))


# ---------------------------------------------------------------------------
# Generate a synthetic mesh if no real file is present
# ---------------------------------------------------------------------------


def _create_synthetic_cell(path: Path) -> None:
    """Create a high-polygon sphere as a stand-in for a robot cell model."""
    print("[demo] No robot_cell.stl found, generating synthetic mesh …")
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
    for _ in range(5):  # subdivide to ~80k triangles
        sphere = sphere.subdivide_midpoint(1)
    sphere.compute_triangle_normals()  # required for a valid STL write
    o3d.io.write_triangle_mesh(str(path), sphere)
    print(f"[demo] Synthetic mesh saved → {path}  ({len(sphere.triangles):,} triangles)")


# ---------------------------------------------------------------------------
# Main demo
# ---------------------------------------------------------------------------


def main():
    from optimizer.moveit_check import print_load_check, run_load_check
    from optimizer.urdf_export import export_urdf_pair

    if not CELL_MESH_PATH.exists():
        _create_synthetic_cell(CELL_MESH_PATH)

    print("\n=== MoveIt Cell Demo: ros-mesh-preprocessor ===\n")
    print(
        "[demo] Pipeline: raw mesh -> clean -> visual/collision decimation -> URDF -> load check\n"
    )

    export_urdf_pair(
        input_path=str(CELL_MESH_PATH),
        output_dir=str(OUTPUT_DIR),
        visual_triangles=20_000,
        collision_triangles=3_000,
        compute_hausdorff=True,
    )

    stem = CELL_MESH_PATH.stem
    snippet_path = OUTPUT_DIR / f"{stem}_urdf_snippet.xml"
    load_check = run_load_check(str(snippet_path), collision_triangle_budget=5_000)
    print_load_check(load_check)

    print("=== Demo complete. Check examples/moveit_cell_demo/output/ ===")
    return 0 if load_check.ok else 1


if __name__ == "__main__":
    sys.exit(main())
