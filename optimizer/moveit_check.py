"""
moveit_check.py - Lightweight "would this load in MoveIt?" sanity check for
a generated URDF visual/collision mesh pair.

This does not require a ROS2 / MoveIt install. It statically validates the
things that most commonly break a MoveIt planning scene load:

  1. The URDF snippet is well-formed XML with the expected
     <link>/<visual>/<collision>/<geometry>/<mesh> structure.
  2. Every mesh file referenced in the snippet exists on disk next to it
     (a common failure is a package:// path that doesn't resolve).
  3. Each mesh loads with open3d, is non-empty, and has positive surface
     area (catches degenerate/zero-volume decimation results).
  4. The collision mesh is within a triangle budget reasonable for
     real-time collision checking (default: 10,000 triangles); an
     oversized collision mesh is a common cause of slow MoveIt planning.
"""

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import open3d as o3d

log = logging.getLogger("optimizer")

DEFAULT_COLLISION_TRIANGLE_BUDGET = 10_000


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


@dataclass
class LoadCheckReport:
    urdf_snippet: str
    checks: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(c.passed for c in self.checks)


def _mesh_path_from_geometry(geometry_el: ET.Element, snippet_dir: Path) -> Path:
    mesh_el = geometry_el.find("mesh")
    filename = mesh_el.attrib["filename"]
    # package://YOUR_PACKAGE/meshes/foo.stl -> foo.stl, resolved next to the snippet
    return snippet_dir / Path(filename).name


def run_load_check(
    urdf_snippet_path: str,
    collision_triangle_budget: int = DEFAULT_COLLISION_TRIANGLE_BUDGET,
) -> LoadCheckReport:
    """
    Validate a generated URDF snippet and its referenced meshes.

    Returns a LoadCheckReport; check `.ok` for overall pass/fail and
    `.checks` for the itemized results.
    """
    snippet_path = Path(urdf_snippet_path)
    snippet_dir = snippet_path.parent
    report = LoadCheckReport(urdf_snippet=str(snippet_path))

    if not snippet_path.exists():
        report.checks.append(
            CheckResult(
                "URDF snippet is well-formed XML", False, f"file not found: {snippet_path}"
            )
        )
        return report

    try:
        root = ET.fromstring(snippet_path.read_text())
        report.checks.append(
            CheckResult("URDF snippet is well-formed XML", True, str(snippet_path))
        )
    except ET.ParseError as exc:
        report.checks.append(CheckResult("URDF snippet is well-formed XML", False, str(exc)))
        return report

    if root.tag != "link":
        report.checks.append(CheckResult("Root element is <link>", False, f"got <{root.tag}>"))
        return report
    report.checks.append(
        CheckResult("Root element is <link>", True, f"name={root.attrib.get('name')}")
    )

    for tag, budget in (("visual", None), ("collision", collision_triangle_budget)):
        el = root.find(tag)
        if el is None:
            report.checks.append(CheckResult(f"<{tag}> element present", False, "missing"))
            continue
        report.checks.append(CheckResult(f"<{tag}> element present", True, ""))

        geometry_el = el.find("geometry")
        if geometry_el is None or geometry_el.find("mesh") is None:
            report.checks.append(
                CheckResult(f"<{tag}> has a <geometry><mesh> reference", False, "missing")
            )
            continue

        mesh_path = _mesh_path_from_geometry(geometry_el, snippet_dir)
        if not mesh_path.exists():
            report.checks.append(
                CheckResult(f"{tag} mesh file exists on disk", False, str(mesh_path))
            )
            continue
        report.checks.append(CheckResult(f"{tag} mesh file exists on disk", True, str(mesh_path)))

        mesh = o3d.io.read_triangle_mesh(str(mesh_path))
        n_tri = len(mesh.triangles)
        if n_tri == 0:
            report.checks.append(
                CheckResult(f"{tag} mesh has triangles", False, "0 triangles loaded")
            )
            continue
        report.checks.append(
            CheckResult(f"{tag} mesh has triangles", True, f"{n_tri:,} triangles")
        )

        area = mesh.get_surface_area()
        report.checks.append(
            CheckResult(f"{tag} mesh has positive surface area", area > 0.0, f"{area:.6f}")
        )

        if budget is not None:
            report.checks.append(
                CheckResult(
                    f"{tag} mesh is within the {budget:,}-triangle real-time budget",
                    n_tri <= budget,
                    f"{n_tri:,} / {budget:,}",
                )
            )

    return report


def print_load_check(report: LoadCheckReport) -> None:
    sep = "─" * 60
    print(f"\n{sep}")
    print("  MoveIt Load Check")
    print(sep)
    for check in report.checks:
        mark = "PASS" if check.passed else "FAIL"
        print(f"  [{mark}] {check.name}" + (f"  ({check.detail})" if check.detail else ""))
    print(sep)
    print(f"  Overall: {'READY FOR MOVEIT' if report.ok else 'NOT READY, see FAIL lines above'}")
    print(sep + "\n")
