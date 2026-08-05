# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `pyproject.toml` packaging: install with `pip install .` and run the
  `ros-mesh-preprocessor` command directly instead of `python -m optimizer.cli`.
- `--version`, `--dry-run`, `-v/--verbose`, and `--format` CLI flags.
- Consistent process exit codes: `0` success, `1` known runtime error, `2`
  argument-parsing error.
- Unit tests for mesh load/save round-trips, malformed/missing input, URDF
  export, MoveIt load-check logic, and CLI exit-code behaviour (49 tests
  total, up from 10).
- Triangle-count reduction regression tests across a range of targets.
- GitHub Actions CI: lint (black, flake8), test matrix (Python 3.10-3.12),
  and a packaging build check.
- `.pre-commit-config.yaml` for black, flake8, and basic hygiene hooks.
- `optimizer/moveit_check.py`: static validation of a generated URDF
  snippet and its referenced meshes (well-formed XML, files resolve, mesh
  loads with a positive triangle count and surface area, collision mesh
  within a triangle budget), wired into `examples/moveit_cell_demo` as a
  final pipeline step.
- Reproducible benchmark and before/after-visual generators
  (`benchmarks/generate_report.py`, `docs/generate_before_after.py`).

### Fixed
- `simplify_mesh` called `TriangleMesh.simplify_quadric_error_metrics`, which
  open3d removed in 0.19 in favor of `simplify_quadric_decimation`. Since
  `requirements.txt` pinned `open3d>=0.18.0` with no upper bound, a fresh
  install on any open3d >=0.19 crashed on every simplification call. Fixed
  to support both method names.
- `save_mesh` never computed triangle normals before writing, so every STL
  output, including the visual/collision meshes from `export_urdf_pair`
  (the tool's headline feature), was silently written as an empty, 0-byte
  file. `write_triangle_mesh`'s return value was also previously ignored,
  so this failure produced no error. Fixed by recomputing normals before
  every write and raising `RuntimeError` if the write itself reports
  failure. Caught by the new MoveIt load check, which failed the exported
  meshes' "has triangles" check before the fix.
- `load_mesh` now raises `FileNotFoundError` for a missing input path
  instead of letting open3d silently return an empty mesh.

### Changed
- Status output now goes through Python's `logging` module instead of bare
  `print`, so verbosity is controllable with `-v`.

## [0.1.0] - 2026-02-24

### Added
- Initial release: QEM mesh decimation, visual/collision URDF pair export,
  Hausdorff-distance validation, batch folder processing, and JSON benchmark
  reports.
