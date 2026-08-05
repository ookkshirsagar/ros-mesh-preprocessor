"""
tests/test_moveit_check.py - Unit tests for optimizer.moveit_check

Run with:
    pytest tests/ -v
"""

import open3d as o3d
import pytest

from optimizer.moveit_check import run_load_check
from optimizer.urdf_export import export_urdf_pair


@pytest.fixture
def exported_pair(tmp_path):
    """Run a real export_urdf_pair and return (snippet_path, out_dir)."""
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    for _ in range(3):
        sphere = sphere.subdivide_midpoint(1)
    sphere.compute_triangle_normals()
    cell_path = tmp_path / "cell.stl"
    o3d.io.write_triangle_mesh(str(cell_path), sphere)

    out_dir = tmp_path / "output"
    export_urdf_pair(
        input_path=str(cell_path),
        output_dir=str(out_dir),
        visual_triangles=500,
        collision_triangles=100,
        compute_hausdorff=False,
    )
    return out_dir / "cell_urdf_snippet.xml"


class TestRunLoadCheck:
    def test_passes_for_a_valid_export(self, exported_pair):
        report = run_load_check(str(exported_pair))
        assert report.ok
        assert all(c.passed for c in report.checks)

    def test_fails_on_oversized_collision_mesh(self, exported_pair):
        # The collision mesh has ~100 triangles; a budget of 10 should fail
        # exactly (and only) the triangle-budget check.
        report = run_load_check(str(exported_pair), collision_triangle_budget=10)
        assert not report.ok
        failed_names = [c.name for c in report.checks if not c.passed]
        assert any("real-time budget" in name for name in failed_names)

    def test_fails_on_missing_snippet(self, tmp_path):
        report = run_load_check(str(tmp_path / "does_not_exist.xml"))
        assert not report.ok
        assert report.checks[0].name == "URDF snippet is well-formed XML"
        assert not report.checks[0].passed

    def test_fails_on_malformed_xml(self, tmp_path):
        bad_snippet = tmp_path / "bad.xml"
        bad_snippet.write_text("<link><visual>")  # unclosed tags
        report = run_load_check(str(bad_snippet))
        assert not report.ok

    def test_fails_when_referenced_mesh_file_is_missing(self, tmp_path):
        snippet = tmp_path / "cell_urdf_snippet.xml"
        snippet.write_text(
            '<link name="cell">'
            "<visual><geometry>"
            '<mesh filename="package://pkg/meshes/cell_visual.stl" scale="1 1 1"/>'
            "</geometry></visual>"
            "<collision><geometry>"
            '<mesh filename="package://pkg/meshes/cell_collision.stl" scale="1 1 1"/>'
            "</geometry></collision>"
            "</link>"
        )
        report = run_load_check(str(snippet))
        assert not report.ok
        failed_names = [c.name for c in report.checks if not c.passed]
        assert any("exists on disk" in name for name in failed_names)
