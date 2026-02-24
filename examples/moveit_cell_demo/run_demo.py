"""
examples/moveit_cell_demo/run_demo.py

Demonstrates a full industrial robot cell preprocessing pipeline:

1. Load a high-polygon robot cell STL
2. Generate a visual mesh (20 000 triangles)   — for RViz rendering
3. Generate a collision mesh (3 000 triangles) — for MoveIt planning
4. Output a URDF snippet
5. Print a benchmarking report with Hausdorff distance

Usage:
    python examples/moveit_cell_demo/run_demo.py

Note:
    The demo creates a synthetic sphere mesh if no real STL is present.
    Replace CELL_MESH_PATH to point at your actual robot cell model.
"""

from pathlib import Path
import open3d as o3d

# --- Path configuration ---------------------------------------------------
DEMO_DIR = Path(__file__).parent
OUTPUT_DIR = DEMO_DIR / "output"
CELL_MESH_PATH = DEMO_DIR / "robot_cell.stl"  # ← replace with your file


# ---------------------------------------------------------------------------
# Generate a synthetic mesh if no real file is present
# ---------------------------------------------------------------------------

def _create_synthetic_cell(path: Path) -> None:
    """Create a high-polygon sphere as a stand-in for a robot cell model."""
    print("[demo] No robot_cell.stl found — generating synthetic mesh …")
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
    for _ in range(5):  # subdivide to ~80k triangles
        sphere = sphere.subdivide_midpoint(1)
    o3d.io.write_triangle_mesh(str(path), sphere)
    print(f"[demo] Synthetic mesh saved → {path}  ({len(sphere.triangles):,} triangles)")


# ---------------------------------------------------------------------------
# Main demo
# ---------------------------------------------------------------------------

def main():
    if not CELL_MESH_PATH.exists():
        _create_synthetic_cell(CELL_MESH_PATH)

    # Import here so the demo is self-contained even when run from this dir
    import sys
    sys.path.insert(0, str(DEMO_DIR.parent.parent))

    from optimizer.urdf_export import export_urdf_pair

    print("\n=== MoveIt Cell Demo — ros-mesh-preprocessor ===\n")
    export_urdf_pair(
        input_path=str(CELL_MESH_PATH),
        output_dir=str(OUTPUT_DIR),
        visual_triangles=20_000,
        collision_triangles=3_000,
        compute_hausdorff=True,
    )
    print("\n=== Demo complete. Check examples/moveit_cell_demo/output/ ===")


if __name__ == "__main__":
    main()
