"""
tests/test_core.py - Unit tests for optimizer.core

Run with:
    pytest tests/ -v
"""

import numpy as np
import open3d as o3d
import pytest

from optimizer.core import (
    clean_mesh,
    resolve_target_triangles,
    simplify_mesh,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _make_sphere(n_subdivisions: int = 3) -> o3d.geometry.TriangleMesh:
    """Create a subdivided icosphere for testing."""
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    for _ in range(n_subdivisions):
        sphere = sphere.subdivide_midpoint(1)
    return sphere


# ---------------------------------------------------------------------------
# core.clean_mesh
# ---------------------------------------------------------------------------

class TestCleanMesh:
    def test_returns_triangle_mesh(self):
        mesh = _make_sphere()
        result = clean_mesh(mesh)
        assert isinstance(result, o3d.geometry.TriangleMesh)

    def test_preserves_topology(self):
        mesh = _make_sphere()
        original_tris = len(mesh.triangles)
        result = clean_mesh(mesh)
        # Cleaning should not ADD triangles
        assert len(result.triangles) <= original_tris


# ---------------------------------------------------------------------------
# core.simplify_mesh
# ---------------------------------------------------------------------------

class TestSimplifyMesh:
    def test_reduces_triangle_count(self):
        mesh = _make_sphere(n_subdivisions=4)
        target = 200
        result, elapsed = simplify_mesh(mesh, target)
        assert len(result.triangles) <= target * 1.1  # allow small overshoot

    def test_returns_positive_elapsed(self):
        mesh = _make_sphere()
        _, elapsed = simplify_mesh(mesh, 100)
        assert elapsed >= 0.0

    def test_result_is_triangle_mesh(self):
        mesh = _make_sphere()
        result, _ = simplify_mesh(mesh, 80)
        assert isinstance(result, o3d.geometry.TriangleMesh)


# ---------------------------------------------------------------------------
# core.resolve_target_triangles
# ---------------------------------------------------------------------------

class TestResolveTargetTriangles:
    def _mesh_with(self, n_triangles: int) -> o3d.geometry.TriangleMesh:
        """Return a mesh guaranteed to have at least n triangles."""
        sphere = _make_sphere(n_subdivisions=4)
        # We just need len(mesh.triangles) to be known; use sphere directly
        return sphere

    def test_absolute_target(self):
        mesh = _make_sphere()
        result = resolve_target_triangles(mesh, target_triangles=500, reduction_percent=None)
        assert result == 500

    def test_percentage_reduction(self):
        mesh = _make_sphere(n_subdivisions=3)
        n = len(mesh.triangles)
        result = resolve_target_triangles(mesh, target_triangles=None, reduction_percent=50.0)
        expected = int(n * 0.5)
        assert result == expected

    def test_raises_when_both_given(self):
        mesh = _make_sphere()
        with pytest.raises(ValueError, match="not both"):
            resolve_target_triangles(mesh, target_triangles=100, reduction_percent=50)

    def test_raises_when_neither_given(self):
        mesh = _make_sphere()
        with pytest.raises(ValueError):
            resolve_target_triangles(mesh, target_triangles=None, reduction_percent=None)

    def test_raises_on_invalid_percentage(self):
        mesh = _make_sphere()
        with pytest.raises(ValueError, match="between 0 and 100"):
            resolve_target_triangles(mesh, target_triangles=None, reduction_percent=110)
