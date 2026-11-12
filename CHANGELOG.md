# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed
- The Hugging Face Space deployment now runs only when started manually
  (Actions -> Deploy demo to Hugging Face -> Run workflow), since hosting a
  Gradio Space requires a Hugging Face PRO subscription.

## [0.3.0] - 2026-11-12

### Added
- Browser GUI (`urdf-mesh-optimizer --gui`, optional `gui` extra) built with
  Gradio: a single-mesh tab with side-by-side 3D before/after previews,
  triangle budget, output format, deviation metrics and download; and a
  whole-robot tab that takes a zipped description package and returns the
  optimized URDF and meshes. Zip uploads are checked for unsafe paths and
  size. Example meshes and a demo robot package in `examples/gui/`
  (regenerate with `examples/gui/make_examples.py`).
- Hugging Face Space deployment (`space/`, `.github/workflows/space.yaml`):
  publishes the GUI as a public demo on every change to `main` once the
  `HF_TOKEN` secret and `HF_SPACE` variable are set.

## [0.2.0] - 2026-11-12

### Added
- Whole-robot mode: `--urdf robot.urdf --output DIR --collision N [--visual N]`
  (`optimizer/urdf_robot.py`) optimizes every mesh a URDF references and
  writes a rewritten URDF plus `urdf_report.json`. Resolves `package://`
  (next to the URDF, via `AMENT_PREFIX_PATH`, or `--package-path NAME=DIR`),
  `file://` and relative URIs; processes shared meshes once; copies meshes
  already under budget; leaves visuals untouched unless `--visual` is given;
  skips formats open3d cannot read; passes every non-mesh element and comment
  through; rejects unexpanded xacro. `--mesh-uri-prefix` sets the rewritten
  URIs, e.g. `package://my_robot_description/meshes_optimized`.
- Release workflow (`.github/workflows/release.yaml`): pushing a `v*.*.*` tag
  runs the tests, checks the tag matches `optimizer.__version__`, builds,
  publishes to PyPI via Trusted Publishing, and creates a GitHub Release
  with the changelog notes. README links are made absolute for the PyPI page.
  See `RELEASING.md`.
- `pyproject.toml` packaging: install with `pip install .` and run the
  `urdf-mesh-optimizer` command directly instead of `python -m optimizer.cli`.
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
  (`benchmarks/generate_report.py`, `docs/generate_figures.py`).

- README figures generated from real pipeline output by
  `docs/generate_figures.py`: six parametric industrial components
  (servo motor, involute spur gear, weld-neck flange, T-slot extrusion,
  pillow block bearing, timing pulley) built with CSG in
  `docs/figures/parts.py`, rendered headlessly with pyrender/EGL into a
  CAD -> visual -> collision hero, a split-view gallery grid, a deviation
  heatmap and benchmark plots; per-part metrics in
  `docs/figures/results.json`. Plus a pipeline diagram
  (`docs/img/pipeline.svg`).
- `Dockerfile` for running the CLI without a local Open3D/OpenGL setup, built
  in CI.
- Issue forms, pull request template, Dependabot config, and `CITATION.cff`.

### Removed
- `setup_repo.sh` (one-off bootstrap script) and the torus-only
  `docs/generate_before_after.py`, superseded by `docs/generate_figures.py`.

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
- Renamed the project, PyPI distribution, CLI command and Docker image from
  `ros-mesh-preprocessor` to `urdf-mesh-optimizer`. The Python import
  package is still `optimizer`.
- Status output now goes through Python's `logging` module instead of bare
  `print`, so verbosity is controllable with `-v`.
- Package metadata uses an SPDX license expression (`license = "MIT"`),
  replacing the deprecated license table and classifier.

## [0.1.0] - 2026-02-24

### Added
- Initial release: QEM mesh decimation, visual/collision URDF pair export,
  Hausdorff-distance validation, batch folder processing, and JSON benchmark
  reports.

[Unreleased]: https://github.com/ookkshirsagar/urdf-mesh-optimizer/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/ookkshirsagar/urdf-mesh-optimizer/releases/tag/v0.3.0
[0.2.0]: https://github.com/ookkshirsagar/urdf-mesh-optimizer/releases/tag/v0.2.0
