"""
core.py - Mesh loading, simplification, and cleaning logic.
Supports: STL, OBJ, PLY, OFF, GLB/GLTF
"""

import time
import open3d as o3d
import numpy as np
from pathlib import Path


SUPPORTED_FORMATS = {".stl", ".obj", ".ply", ".off", ".glb", ".gltf"}


def load_mesh(path: str) -> o3d.geometry.TriangleMesh:
    """Load a mesh from any supported format."""
    p = Path(path)
    if p.suffix.lower() not in SUPPORTED_FORMATS:
        raise ValueError(
            f"Unsupported format: '{p.suffix}'. "
            f"Supported: {', '.join(SUPPORTED_FORMATS)}"
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
    o3d.io.write_triangle_mesh(str(p), mesh)


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
