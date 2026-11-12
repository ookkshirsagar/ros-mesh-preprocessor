"""
tests/test_urdf_robot.py - Unit tests for optimizer.urdf_robot (whole-robot URDF mode).

Builds a throwaway ROS-style description package:

    my_robot_description/
    ├── package.xml
    ├── urdf/robot.urdf        package://, relative and file:// mesh URIs
    └── meshes/
        ├── base.stl           dense  (needs decimation)
        ├── arm.stl            dense, shared by two links
        ├── tool.stl           already under budget
        └── cover.dae          unsupported format (must be skipped, URI untouched)

Run with:
    pytest tests/ -v
"""

import json
import xml.etree.ElementTree as ET

import open3d as o3d
import pytest

from optimizer.cli import main
from optimizer.urdf_robot import UrdfError, optimize_urdf, resolve_mesh_uri

PKG = "my_robot_description"


def _sphere(path, subdivisions):
    m = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    for _ in range(subdivisions):
        m = m.subdivide_midpoint(1)
    m.compute_triangle_normals()
    o3d.io.write_triangle_mesh(str(path), m)
    return len(m.triangles)


@pytest.fixture
def robot_pkg(tmp_path):
    pkg = tmp_path / "ws" / PKG
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    (pkg / "package.xml").write_text(f"<package><name>{PKG}</name></package>")
    _sphere(pkg / "meshes" / "base.stl", 1)  # 6,080 triangles
    _sphere(pkg / "meshes" / "arm.stl", 1)
    _sphere(pkg / "meshes" / "tool.stl", 0)  # 1,520 triangles
    (pkg / "meshes" / "cover.dae").write_text("<COLLADA/>")
    tool_abs = (pkg / "meshes" / "tool.stl").as_posix()
    urdf = f"""<?xml version="1.0"?>
<robot name="test_bot">
  <!-- a comment that must survive -->
  <link name="world"/>
  <link name="base_link">
    <visual><geometry><mesh filename="package://{PKG}/meshes/base.stl"/></geometry></visual>
    <collision><geometry><mesh filename="package://{PKG}/meshes/base.stl"/></geometry></collision>
  </link>
  <link name="arm_1">
    <visual><geometry><mesh filename="package://{PKG}/meshes/cover.dae"/></geometry></visual>
    <collision>
      <geometry><mesh filename="../meshes/arm.stl" scale="0.001 0.001 0.001"/></geometry>
    </collision>
  </link>
  <link name="arm_2">
    <collision><geometry><mesh filename="package://{PKG}/meshes/arm.stl"/></geometry></collision>
  </link>
  <link name="tool">
    <collision><geometry><mesh filename="file://{tool_abs}"/></geometry></collision>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="j0" type="fixed"><parent link="world"/><child link="base_link"/></joint>
</robot>
"""
    path = pkg / "urdf" / "robot.urdf"
    path.write_text(urdf)
    return path


def _mesh_uris(urdf_path, role):
    root = ET.parse(urdf_path).getroot()
    return {
        link.attrib["name"]: [m.attrib["filename"] for m in link.findall(f"{role}/geometry/mesh")]
        for link in root.iter("link")
    }


class TestResolveMeshUri:
    def test_package_uri_found_from_urdf_location(self, robot_pkg):
        p = resolve_mesh_uri(f"package://{PKG}/meshes/base.stl", robot_pkg.parent, {})
        assert p.resolve() == (robot_pkg.parent.parent / "meshes" / "base.stl").resolve()

    def test_package_path_override(self, robot_pkg, tmp_path):
        other = tmp_path / "elsewhere"
        p = resolve_mesh_uri("package://foo/m.stl", robot_pkg.parent, {"foo": other})
        assert p == other / "m.stl"

    def test_ament_prefix_path(self, tmp_path, monkeypatch):
        (tmp_path / "install" / "share" / "bar").mkdir(parents=True)
        monkeypatch.setenv("AMENT_PREFIX_PATH", str(tmp_path / "install"))
        p = resolve_mesh_uri("package://bar/m.stl", tmp_path / "nowhere", {})
        assert p == tmp_path / "install" / "share" / "bar" / "m.stl"

    def test_unknown_package_raises(self, tmp_path, monkeypatch):
        monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)
        with pytest.raises(UrdfError, match="--package-path"):
            resolve_mesh_uri("package://nope/m.stl", tmp_path, {})

    def test_relative_and_file_uris(self, tmp_path):
        assert resolve_mesh_uri("m.stl", tmp_path, {}) == tmp_path / "m.stl"
        assert resolve_mesh_uri("file:///abs/m.stl", tmp_path, {}).as_posix() == "/abs/m.stl"

    def test_unsupported_scheme_raises(self, tmp_path):
        with pytest.raises(UrdfError, match="scheme"):
            resolve_mesh_uri("http://example.com/m.stl", tmp_path, {})


class TestOptimizeUrdf:
    def test_collision_meshes_within_budget_and_rewritten(self, robot_pkg, tmp_path):
        out = tmp_path / "out"
        summary = optimize_urdf(str(robot_pkg), str(out), collision_triangles=500)
        new_urdf = out / "test_bot.urdf"
        assert new_urdf.exists()

        coll = _mesh_uris(new_urdf, "collision")
        assert coll["base_link"] == ["meshes/collision/base.stl"]
        # arm.stl is referenced twice (relative + package://) -> processed once, both rewritten
        assert coll["arm_1"] == coll["arm_2"] == ["meshes/collision/arm.stl"]
        for uris in coll.values():
            for uri in uris:
                m = o3d.io.read_triangle_mesh(str(out / uri))
                assert 0 < len(m.triangles) <= 500

        arm = next(m for m in summary.meshes if m.source_path.endswith("arm.stl"))
        assert sorted(arm.links) == ["arm_1", "arm_2"]
        assert arm.status == "simplified" and arm.output_triangles <= 500

    def test_under_budget_mesh_is_copied_not_simplified(self, robot_pkg, tmp_path):
        summary = optimize_urdf(str(robot_pkg), str(tmp_path / "out"), collision_triangles=500)
        tool = next(m for m in summary.meshes if m.source_path.endswith("tool.stl"))
        assert tool.status == "simplified"  # 1,520 > 500
        summary = optimize_urdf(str(robot_pkg), str(tmp_path / "out2"), collision_triangles=2000)
        tool = next(m for m in summary.meshes if m.source_path.endswith("tool.stl"))
        assert tool.status == "copied"
        assert tool.output_triangles == tool.original_triangles

    def test_visuals_untouched_without_visual_budget(self, robot_pkg, tmp_path):
        out = tmp_path / "out"
        optimize_urdf(str(robot_pkg), str(out), collision_triangles=500)
        vis = _mesh_uris(out / "test_bot.urdf", "visual")
        assert vis["base_link"] == [f"package://{PKG}/meshes/base.stl"]
        assert not (out / "meshes" / "visual").exists()

    def test_visual_budget_and_unsupported_format_skipped(self, robot_pkg, tmp_path):
        out = tmp_path / "out"
        summary = optimize_urdf(
            str(robot_pkg), str(out), collision_triangles=500, visual_triangles=2000
        )
        vis = _mesh_uris(out / "test_bot.urdf", "visual")
        assert vis["base_link"] == ["meshes/visual/base.stl"]
        assert vis["arm_1"] == [f"package://{PKG}/meshes/cover.dae"]  # .dae left alone
        dae = next(m for m in summary.meshes if m.source_path.endswith(".dae"))
        assert dae.status == "skipped" and "not a supported" in dae.reason

    def test_mesh_uri_prefix(self, robot_pkg, tmp_path):
        out = tmp_path / "out"
        optimize_urdf(
            str(robot_pkg),
            str(out),
            collision_triangles=500,
            mesh_uri_prefix="package://opt_pkg/meshes/",
        )
        coll = _mesh_uris(out / "test_bot.urdf", "collision")
        assert coll["base_link"] == ["package://opt_pkg/meshes/collision/base.stl"]

    def test_everything_else_passes_through(self, robot_pkg, tmp_path):
        out = tmp_path / "out"
        optimize_urdf(str(robot_pkg), str(out), collision_triangles=500)
        text = (out / "test_bot.urdf").read_text()
        root = ET.fromstring(text.encode())
        assert "a comment that must survive" in text
        assert root.find("joint/parent").attrib["link"] == "world"
        assert root.find("link[@name='tool']/collision/geometry/box") is not None
        arm_mesh = root.find("link[@name='arm_1']/collision/geometry/mesh")
        assert arm_mesh.attrib["scale"] == "0.001 0.001 0.001"

    def test_report_written(self, robot_pkg, tmp_path):
        out = tmp_path / "out"
        optimize_urdf(str(robot_pkg), str(out), collision_triangles=500, compute_hausdorff=False)
        report = json.loads((out / "urdf_report.json").read_text())
        assert report["robot"] == "test_bot"
        assert report["collision_triangles_after"] < report["collision_triangles_before"]

    def test_dry_run_writes_nothing(self, robot_pkg, tmp_path):
        out = tmp_path / "out"
        summary = optimize_urdf(str(robot_pkg), str(out), collision_triangles=500, dry_run=True)
        assert not out.exists()
        assert summary.output_urdf is None
        base = next(
            m
            for m in summary.meshes
            if m.role == "collision" and m.source_path.endswith("base.stl")
        )
        assert base.original_triangles == 6080 and base.output_triangles == 500

    def test_missing_mesh_raises(self, robot_pkg, tmp_path):
        (robot_pkg.parent.parent / "meshes" / "base.stl").unlink()
        with pytest.raises(UrdfError, match="not found"):
            optimize_urdf(str(robot_pkg), str(tmp_path / "out"), collision_triangles=500)

    def test_unexpanded_xacro_rejected(self, tmp_path):
        p = tmp_path / "robot.urdf.xacro"
        p.write_text(
            '<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro">'
            '<xacro:macro name="m"/></robot>'
        )
        with pytest.raises(UrdfError, match="xacro"):
            optimize_urdf(str(p), str(tmp_path / "out"), collision_triangles=500)

    def test_not_a_robot_rejected(self, tmp_path):
        p = tmp_path / "x.urdf"
        p.write_text("<link name='a'/>")
        with pytest.raises(UrdfError, match="expected <robot>"):
            optimize_urdf(str(p), str(tmp_path / "out"), collision_triangles=500)


class TestRobotCli:
    def _run(self, monkeypatch, argv):
        monkeypatch.setattr("sys.argv", ["urdf-mesh-optimizer"] + argv)
        try:
            main()
        except SystemExit as exc:
            return exc.code
        return 0

    def test_success(self, monkeypatch, robot_pkg, tmp_path):
        out = tmp_path / "out"
        argv = ["--urdf", str(robot_pkg), "--output", str(out), "--collision", "500"]
        assert self._run(monkeypatch, argv + ["--skip-hausdorff"]) == 0
        assert (out / "test_bot.urdf").exists()

    def test_requires_collision(self, monkeypatch, robot_pkg, tmp_path):
        argv = ["--urdf", str(robot_pkg), "--output", str(tmp_path / "out")]
        assert self._run(monkeypatch, argv) == 1

    def test_requires_output(self, monkeypatch, robot_pkg):
        assert self._run(monkeypatch, ["--urdf", str(robot_pkg), "--collision", "500"]) == 1

    def test_bad_package_path_syntax(self, monkeypatch, robot_pkg, tmp_path):
        argv = ["--urdf", str(robot_pkg), "--output", str(tmp_path / "o"), "--collision", "500"]
        assert self._run(monkeypatch, argv + ["--package-path", "nodir"]) == 1

    def test_package_path_flag(self, monkeypatch, robot_pkg, tmp_path):
        old = robot_pkg.parent.parent
        moved = tmp_path / "moved"
        old.rename(moved)  # package dir no longer named after the package
        urdf = moved / "urdf" / "robot.urdf"
        urdf.write_text(urdf.read_text().replace(old.as_posix(), moved.as_posix()))
        argv = ["--urdf", str(urdf), "--output", str(tmp_path / "o"), "--collision", "500"]
        monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)
        assert self._run(monkeypatch, argv + ["--skip-hausdorff"]) == 1
        argv += ["--skip-hausdorff", "--package-path", f"{PKG}={moved}"]
        assert self._run(monkeypatch, argv) == 0

    def test_dry_run(self, monkeypatch, robot_pkg, tmp_path, capsys):
        out = tmp_path / "out"
        argv = ["--urdf", str(robot_pkg), "--output", str(out), "--collision", "500", "--dry-run"]
        assert self._run(monkeypatch, argv) == 0
        assert not out.exists()
        assert "would be simplified" in capsys.readouterr().out
