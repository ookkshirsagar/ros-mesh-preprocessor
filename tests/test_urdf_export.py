"""
tests/test_urdf_export.py - Unit tests for optimizer.urdf_export

Run with:
    pytest tests/ -v
"""

import xml.etree.ElementTree as ET

import open3d as o3d
import pytest

from optimizer.urdf_export import export_urdf_pair


@pytest.fixture
def cell_stl(tmp_path):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    for _ in range(3):
        sphere = sphere.subdivide_midpoint(1)
    sphere.compute_triangle_normals()
    path = tmp_path / "cell.stl"
    o3d.io.write_triangle_mesh(str(path), sphere)
    return path


class TestExportUrdfPair:
    def test_writes_visual_and_collision_meshes(self, cell_stl, tmp_path):
        out_dir = tmp_path / "output"
        export_urdf_pair(
            input_path=str(cell_stl),
            output_dir=str(out_dir),
            visual_triangles=500,
            collision_triangles=100,
            compute_hausdorff=False,
        )
        assert (out_dir / "cell_visual.stl").exists()
        assert (out_dir / "cell_collision.stl").exists()

    def test_collision_mesh_is_not_larger_than_visual(self, cell_stl, tmp_path):
        out_dir = tmp_path / "output"
        export_urdf_pair(
            input_path=str(cell_stl),
            output_dir=str(out_dir),
            visual_triangles=500,
            collision_triangles=100,
            compute_hausdorff=False,
        )
        visual = o3d.io.read_triangle_mesh(str(out_dir / "cell_visual.stl"))
        collision = o3d.io.read_triangle_mesh(str(out_dir / "cell_collision.stl"))
        assert len(collision.triangles) <= len(visual.triangles)

    def test_urdf_snippet_is_well_formed_xml(self, cell_stl, tmp_path):
        out_dir = tmp_path / "output"
        export_urdf_pair(
            input_path=str(cell_stl),
            output_dir=str(out_dir),
            visual_triangles=500,
            collision_triangles=100,
            compute_hausdorff=False,
        )
        snippet_path = out_dir / "cell_urdf_snippet.xml"
        # Fails with ParseError if the emitted XML is malformed.
        root = ET.fromstring(snippet_path.read_text())
        assert root.tag == "link"
        assert root.attrib["name"] == "cell"

    def test_snippet_references_the_generated_mesh_filenames(self, cell_stl, tmp_path):
        out_dir = tmp_path / "output"
        export_urdf_pair(
            input_path=str(cell_stl),
            output_dir=str(out_dir),
            visual_triangles=500,
            collision_triangles=100,
            compute_hausdorff=False,
        )
        snippet_text = (out_dir / "cell_urdf_snippet.xml").read_text()
        assert "cell_visual.stl" in snippet_text
        assert "cell_collision.stl" in snippet_text

    def test_reports_are_saved_when_requested(self, cell_stl, tmp_path):
        out_dir = tmp_path / "output"
        export_urdf_pair(
            input_path=str(cell_stl),
            output_dir=str(out_dir),
            visual_triangles=500,
            collision_triangles=100,
            compute_hausdorff=False,
            save_reports=True,
        )
        assert (out_dir / "cell_visual_report.json").exists()
        assert (out_dir / "cell_collision_report.json").exists()
