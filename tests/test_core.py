"""
tests/test_core.py - Unit tests for optimizer.core

Run with:
    pytest tests/ -v
"""

import open3d as o3d
import pytest

from optimizer.core import (
    clean_mesh,
    load_mesh,
    resolve_output_path,
    resolve_target_triangles,
    save_mesh,
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
    sphere.compute_triangle_normals()
    return sphere


@pytest.fixture
def sphere_stl(tmp_path):
    """Write a small sphere mesh to an .stl file and return its path."""
    mesh = _make_sphere(n_subdivisions=2)
    path = tmp_path / "sphere.stl"
    o3d.io.write_triangle_mesh(str(path), mesh)
    return path


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


# ---------------------------------------------------------------------------
# core.load_mesh / core.save_mesh
# ---------------------------------------------------------------------------


class TestLoadMesh:
    def test_round_trip_preserves_triangle_count(self, sphere_stl):
        mesh = load_mesh(str(sphere_stl))
        original = _make_sphere(n_subdivisions=2)
        assert len(mesh.triangles) == len(original.triangles)

    def test_raises_on_missing_file(self, tmp_path):
        missing = tmp_path / "does_not_exist.stl"
        with pytest.raises(FileNotFoundError, match="not found"):
            load_mesh(str(missing))

    def test_raises_on_unsupported_extension(self, tmp_path):
        bogus = tmp_path / "model.xyz"
        bogus.write_text("not a mesh")
        with pytest.raises(ValueError, match="Unsupported format"):
            load_mesh(str(bogus))

    def test_raises_on_empty_or_corrupt_file(self, tmp_path):
        empty = tmp_path / "empty.stl"
        empty.write_text("")
        with pytest.raises(RuntimeError, match="No triangles found"):
            load_mesh(str(empty))


class TestSaveMesh:
    def test_creates_parent_directories(self, tmp_path):
        mesh = _make_sphere(n_subdivisions=1)
        out_path = tmp_path / "nested" / "dir" / "out.stl"
        save_mesh(mesh, str(out_path))
        assert out_path.exists()

    def test_saved_file_reloads_with_same_triangle_count(self, tmp_path):
        mesh = _make_sphere(n_subdivisions=1)
        out_path = tmp_path / "out.obj"
        save_mesh(mesh, str(out_path))
        reloaded = load_mesh(str(out_path))
        assert len(reloaded.triangles) == len(mesh.triangles)

    @pytest.mark.parametrize("ext", ["stl", "obj", "ply", "off"])
    def test_round_trips_across_supported_formats(self, tmp_path, ext):
        mesh = _make_sphere(n_subdivisions=1)
        out_path = tmp_path / f"out.{ext}"
        save_mesh(mesh, str(out_path))
        reloaded = load_mesh(str(out_path))
        assert len(reloaded.triangles) > 0


class TestResolveOutputPath:
    def test_no_override_returns_path_unchanged(self):
        assert resolve_output_path("model_small.stl", None) == "model_small.stl"

    def test_override_replaces_extension(self):
        assert resolve_output_path("model_small.stl", "obj") == "model_small.obj"

    def test_override_accepts_leading_dot(self):
        assert resolve_output_path("model_small.stl", ".ply") == "model_small.ply"

    def test_raises_on_unsupported_format(self):
        with pytest.raises(ValueError, match="Unsupported --format"):
            resolve_output_path("model_small.stl", "fbx")


# ---------------------------------------------------------------------------
# Regression: triangle count reduction should track the requested target
# ---------------------------------------------------------------------------


class TestSimplifyMeshRegression:
    """
    Locks in the expected relationship between a requested target triangle
    count and the actual result, across a range of reduction levels, so a
    future dependency upgrade (e.g. open3d's QEM implementation changing)
    is caught instead of silently degrading mesh quality.
    """

    @pytest.mark.parametrize("target", [50, 200, 1000, 5000])
    def test_result_never_exceeds_target_by_much(self, target):
        mesh = _make_sphere(n_subdivisions=4)  # ~20k triangles
        result, _ = simplify_mesh(mesh, target)
        assert len(result.triangles) <= target * 1.1

    def test_does_not_increase_triangle_count(self):
        mesh = _make_sphere(n_subdivisions=4)
        original = len(mesh.triangles)
        result, _ = simplify_mesh(mesh, target_triangles=original // 2)
        assert len(result.triangles) < original

    def test_simplified_mesh_has_no_degenerate_triangles_after_cleaning(self):
        mesh = _make_sphere(n_subdivisions=4)
        result, _ = simplify_mesh(mesh, target_triangles=500)
        cleaned = clean_mesh(result)
        assert len(cleaned.triangles) == len(result.triangles)
