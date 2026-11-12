"""
tests/test_gui.py - Tests for the GUI's processing functions (optimizer.gui).

The processing layer is plain Python and runs without gradio; the app-building
test is skipped when the `gui` extra is not installed.

Run with:
    pytest tests/ -v
"""

import zipfile
from pathlib import Path

import open3d as o3d
import pytest

from optimizer.gui import GuiError, _fmt_stats, optimize_mesh_file, optimize_urdf_zip


def _sphere_file(path: Path, subdivisions: int = 1) -> Path:
    m = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    for _ in range(subdivisions):
        m = m.subdivide_midpoint(1)
    m.compute_triangle_normals()
    o3d.io.write_triangle_mesh(str(path), m)
    return path


@pytest.fixture
def robot_zip(tmp_path):
    pkg = tmp_path / "src" / "tiny_description"
    (pkg / "meshes").mkdir(parents=True)
    (pkg / "urdf").mkdir()
    (pkg / "package.xml").write_text("<package><name>tiny_description</name></package>")
    _sphere_file(pkg / "meshes" / "body.stl")
    (pkg / "urdf" / "tiny.urdf").write_text(
        '<robot name="tiny"><link name="body">'
        '<visual><geometry><mesh filename="package://tiny_description/meshes/body.stl"/>'
        "</geometry></visual>"
        '<collision><geometry><mesh filename="package://tiny_description/meshes/body.stl"/>'
        "</geometry></collision></link></robot>"
    )
    archive = tmp_path / "tiny.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        for f in pkg.rglob("*"):
            zf.write(f, f.relative_to(tmp_path / "src"))
    return archive


class TestOptimizeMeshFile:
    def test_decimates_and_writes_previews(self, tmp_path):
        src = _sphere_file(tmp_path / "ball.stl")  # 6,080 triangles
        job = optimize_mesh_file(str(src), 500, "obj")
        assert job.stats["original_triangles"] == 6080
        assert 0 < job.stats["optimized_triangles"] <= 500
        assert job.download.endswith(".obj") and Path(job.download).stat().st_size > 0
        assert Path(job.original_preview).suffix == ".glb"
        assert Path(job.optimized_preview).stat().st_size > 0
        assert "hausdorff_rms" in job.stats
        assert job.preview_note is None

    def test_under_budget_is_not_decimated(self, tmp_path):
        src = _sphere_file(tmp_path / "ball.stl", subdivisions=0)  # 1,520 triangles
        job = optimize_mesh_file(str(src), 5000)
        assert job.stats["optimized_triangles"] == job.stats["original_triangles"]
        assert "hausdorff_rms" not in job.stats
        assert "Already within budget" in _fmt_stats(job.stats, None)

    def test_stats_table_mentions_relative_deviation(self, tmp_path):
        job = optimize_mesh_file(str(_sphere_file(tmp_path / "b.stl")), 300)
        text = _fmt_stats(job.stats, "note here")
        assert "% of part size" in text and "note here" in text

    def test_rejects_unsupported_format(self, tmp_path):
        bad = tmp_path / "part.step"
        bad.write_text("not a mesh")
        with pytest.raises(GuiError, match="aren't supported"):
            optimize_mesh_file(str(bad), 500)

    def test_rejects_tiny_budget(self, tmp_path):
        with pytest.raises(GuiError, match="at least 4"):
            optimize_mesh_file(str(_sphere_file(tmp_path / "b.stl")), 2)


class TestOptimizeUrdfZip:
    def test_round_trip(self, robot_zip):
        archive, rows, summary = optimize_urdf_zip(str(robot_zip), 300)
        assert "tiny" in summary and "collision" in summary
        coll = [r for r in rows if r[0] == "collision"]
        assert coll and coll[0][4] <= 300 and coll[0][5] == "simplified"
        with zipfile.ZipFile(archive) as zf:
            names = zf.namelist()
        assert "tiny.urdf" in names and "meshes/collision/body.stl" in names

    def test_not_a_zip(self, tmp_path):
        f = tmp_path / "x.zip"
        f.write_text("nope")
        with pytest.raises(GuiError, match="Upload a .zip"):
            optimize_urdf_zip(str(f), 300)

    def test_zip_slip_rejected(self, tmp_path):
        archive = tmp_path / "evil.zip"
        with zipfile.ZipFile(archive, "w") as zf:
            zf.writestr("../escape.urdf", "<robot name='x'/>")
        with pytest.raises(GuiError, match="unsafe path"):
            optimize_urdf_zip(str(archive), 300)

    def test_size_limit(self, robot_zip):
        with pytest.raises(GuiError, match="expands to more than"):
            optimize_urdf_zip(str(robot_zip), 300, max_total_bytes=1000)

    def test_xacro_only(self, tmp_path):
        archive = tmp_path / "x.zip"
        with zipfile.ZipFile(archive, "w") as zf:
            zf.writestr("pkg/urdf/robot.urdf.xacro", "<robot/>")
        with pytest.raises(GuiError, match="xacro"):
            optimize_urdf_zip(str(archive), 300)

    def test_no_urdf(self, tmp_path):
        archive = tmp_path / "x.zip"
        with zipfile.ZipFile(archive, "w") as zf:
            zf.writestr("readme.txt", "hi")
        with pytest.raises(GuiError, match="No .urdf"):
            optimize_urdf_zip(str(archive), 300)


def test_build_app():
    pytest.importorskip("gradio")
    from optimizer.gui import build_app

    app = build_app(max_upload_mb=50)
    assert app is not None
