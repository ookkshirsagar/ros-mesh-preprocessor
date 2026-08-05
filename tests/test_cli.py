"""
tests/test_cli.py - Unit tests for optimizer.cli

Exercises the CLI entry point directly (no subprocess) via sys.argv
patching, so exit codes and dry-run behaviour are checked cheaply.

Run with:
    pytest tests/ -v
"""

import open3d as o3d
import pytest

from optimizer.cli import main


@pytest.fixture
def sphere_stl(tmp_path):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    for _ in range(2):
        sphere = sphere.subdivide_midpoint(1)
    sphere.compute_triangle_normals()
    path = tmp_path / "sphere.stl"
    o3d.io.write_triangle_mesh(str(path), sphere)
    return path


def _run(monkeypatch, argv):
    monkeypatch.setattr("sys.argv", ["ros-mesh-preprocessor"] + argv)
    with pytest.raises(SystemExit) as exc_info:
        main()
    return exc_info.value.code


class TestExitCodes:
    def test_version_exits_zero(self, monkeypatch):
        assert _run(monkeypatch, ["--version"]) == 0

    def test_missing_input_file_exits_one(self, monkeypatch, tmp_path):
        code = _run(
            monkeypatch,
            [
                "--input",
                str(tmp_path / "missing.stl"),
                "--output",
                str(tmp_path / "out.obj"),
                "--triangles",
                "100",
            ],
        )
        assert code == 1

    def test_unsupported_output_format_exits_one(self, monkeypatch, sphere_stl, tmp_path):
        code = _run(
            monkeypatch,
            [
                "--input",
                str(sphere_stl),
                "--output",
                str(tmp_path / "out.obj"),
                "--triangles",
                "100",
                "--format",
                "fbx",
            ],
        )
        assert code == 1

    def test_missing_required_flags_exits_two(self, monkeypatch):
        # argparse itself rejects this (no --input/--folder) with exit code 2.
        assert _run(monkeypatch, []) == 2

    def test_single_mode_without_output_exits_two(self, monkeypatch, sphere_stl):
        code = _run(
            monkeypatch,
            [
                "--input",
                str(sphere_stl),
                "--triangles",
                "100",
            ],
        )
        assert code == 2

    def test_urdf_mode_missing_collision_exits_one(self, monkeypatch, sphere_stl, tmp_path):
        code = _run(
            monkeypatch,
            [
                "--input",
                str(sphere_stl),
                "--urdf-output",
                str(tmp_path / "out"),
                "--visual",
                "500",
            ],
        )
        assert code == 1


class TestDryRun:
    def test_dry_run_does_not_write_output(self, monkeypatch, sphere_stl, tmp_path, capsys):
        out_path = tmp_path / "out.obj"
        monkeypatch.setattr(
            "sys.argv",
            [
                "ros-mesh-preprocessor",
                "--input",
                str(sphere_stl),
                "--output",
                str(out_path),
                "--triangles",
                "100",
                "--dry-run",
            ],
        )
        main()  # should return normally, no SystemExit
        assert not out_path.exists()
        assert "dry-run" in capsys.readouterr().out

    def test_dry_run_batch_does_not_create_output_dir(self, monkeypatch, sphere_stl, tmp_path):
        out_dir = tmp_path / "optimized"
        monkeypatch.setattr(
            "sys.argv",
            [
                "ros-mesh-preprocessor",
                "--folder",
                str(sphere_stl.parent),
                "--output",
                str(out_dir),
                "--triangles",
                "100",
                "--dry-run",
            ],
        )
        main()
        assert not out_dir.exists()


class TestSuccessfulRun:
    def test_single_file_run_writes_output(self, monkeypatch, sphere_stl, tmp_path):
        out_path = tmp_path / "out.obj"
        monkeypatch.setattr(
            "sys.argv",
            [
                "ros-mesh-preprocessor",
                "--input",
                str(sphere_stl),
                "--output",
                str(out_path),
                "--triangles",
                "100",
                "--skip-hausdorff",
            ],
        )
        main()  # returns normally on success
        assert out_path.exists()
