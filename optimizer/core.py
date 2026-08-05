"""
core.py - Mesh loading, simplification, and cleaning logic.
Supports: STL, OBJ, PLY, OFF, GLB/GLTF
"""

import time
from pathlib import Path

import open3d as o3d

SUPPORTED_FORMATS = {".stl", ".obj", ".ply", ".off", ".glb", ".gltf"}


def load_mesh(path: str) -> o3d.geometry.TriangleMesh:
    """Load a mesh from any supported format."""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Input file not found: '{path}'.")
    if p.suffix.lower() not in SUPPORTED_FORMATS:
        raise ValueError(
            f"Unsupported format: '{p.suffix}'. " f"Supported: {', '.join(SUPPORTED_FORMATS)}"
        )
    mesh = o3d.io.read_triangle_mesh(str(p))
    if len(mesh.triangles) == 0:
        raise RuntimeError(f"No triangles found in '{path}'. File may be empty or corrupt.")
    return mesh


def clean_mesh(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    """Remove duplicated vertices, degenerate triangles, and unreferenced geometry."""
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    return mesh


def simplify_mesh(
    mesh: o3d.geometry.TriangleMesh,
    target_triangles: int,
) -> tuple[o3d.geometry.TriangleMesh, float]:
    """
    Simplify a mesh using Quadric Error Metrics (QEM).

    Returns:
        (simplified_mesh, processing_time_sec)
    """
    t0 = time.perf_counter()
    # open3d renamed this method to simplify_quadric_decimation in 0.19;
    # support both so the tool works across the >=0.18 range we advertise.
    if hasattr(mesh, "simplify_quadric_decimation"):
        simplified = mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangles)
    else:
        simplified = mesh.simplify_quadric_error_metrics(
            target_number_of_triangles=target_triangles
        )
    elapsed = time.perf_counter() - t0
    simplified = clean_mesh(simplified)
    return simplified, elapsed


def save_mesh(mesh: o3d.geometry.TriangleMesh, path: str) -> None:
    """Save a mesh; output format is inferred from the file extension."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    # STL export silently fails (writes a 0-byte file) without triangle
    # normals. simplify_quadric_decimation doesn't carry normals through,
    # so recompute them unconditionally before every write.
    mesh.compute_triangle_normals()
    success = o3d.io.write_triangle_mesh(str(p), mesh)
    if not success:
        raise RuntimeError(f"Failed to write mesh to '{path}'.")


def resolve_output_path(path: str, format_override: str | None) -> str:
    """
    Apply an explicit --format override to an output path, replacing its
    extension. Validates the requested format is supported.
    """
    if format_override is None:
        return path
    fmt = format_override.lower().lstrip(".")
    suffix = f".{fmt}"
    if suffix not in SUPPORTED_FORMATS:
        raise ValueError(
            f"Unsupported --format '{format_override}'. "
            f"Supported: {', '.join(sorted(s.lstrip('.') for s in SUPPORTED_FORMATS))}"
        )
    return str(Path(path).with_suffix(suffix))


def resolve_target_triangles(
    mesh: o3d.geometry.TriangleMesh,
    target_triangles: int | None,
    reduction_percent: float | None,
) -> int:
    """
    Resolve target triangle count from either an absolute value or a percentage.
    Exactly one of the two arguments must be provided.
    """
    original = len(mesh.triangles)
    if target_triangles is not None and reduction_percent is not None:
        raise ValueError("Provide --triangles OR --reduction, not both.")
    if target_triangles is not None:
        return int(target_triangles)
    if reduction_percent is not None:
        if not (0 < reduction_percent < 100):
            raise ValueError("--reduction must be between 0 and 100 (exclusive).")
        keep = 1.0 - (reduction_percent / 100.0)
        return max(1, int(original * keep))
    raise ValueError("Provide --triangles or --reduction.")
