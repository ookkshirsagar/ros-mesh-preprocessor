"""
urdf_robot.py - Optimize every mesh referenced by a complete robot URDF.

Reads a URDF, resolves each <visual>/<collision> <mesh filename="..."> (package://,
file:// or relative paths), decimates each unique mesh once, and writes:

    <output>/<robot>.urdf                 the same URDF with mesh URIs rewritten
    <output>/meshes/collision/*.stl       collision meshes, each within the triangle budget
    <output>/meshes/visual/*.stl          visual meshes (only when a visual budget is given)
    <output>/urdf_report.json             per-mesh triangle counts, timing and deviation

Everything else in the URDF (joints, inertials, materials, primitives, comments,
ros2_control / gazebo tags) is passed through untouched. Meshes already within
budget are copied (cleaned) without decimation. Meshes in formats open3d cannot
read (e.g. COLLADA .dae) are left pointing at their original files and reported
as skipped.

Usage (programmatic):
    from optimizer.urdf_robot import optimize_urdf

    summary = optimize_urdf("my_robot.urdf", "out/", collision_triangles=3000)
"""

from __future__ import annotations

import json
import logging
import os
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
from urllib.parse import unquote, urlparse

from optimizer.core import SUPPORTED_FORMATS, clean_mesh, load_mesh, save_mesh, simplify_mesh
from optimizer.metrics import hausdorff_distance

log = logging.getLogger("optimizer")

XACRO_NS = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", XACRO_NS)

ROLES = ("visual", "collision")


class UrdfError(Exception):
    """A URDF could not be read or one of its mesh references could not be resolved."""


@dataclass
class MeshResult:
    role: str
    source_uri: str
    source_path: str
    links: list[str] = field(default_factory=list)
    status: str = "pending"  # simplified | copied | kept | skipped
    output_path: str | None = None
    output_uri: str | None = None
    original_triangles: int | None = None
    output_triangles: int | None = None
    processing_time_sec: float | None = None
    hausdorff_max: float | None = None
    hausdorff_rms: float | None = None
    reason: str | None = None


@dataclass
class UrdfSummary:
    robot: str
    input_urdf: str
    output_urdf: str | None
    meshes: list[MeshResult]

    def totals(self, role: str) -> tuple[int, int]:
        """(triangles before, triangles after) over processed meshes of one role."""
        done = [m for m in self.meshes if m.role == role and m.output_triangles is not None]
        return (
            sum(m.original_triangles for m in done),
            sum(m.output_triangles for m in done),
        )

    def to_dict(self) -> dict:
        d = {
            "robot": self.robot,
            "input_urdf": self.input_urdf,
            "output_urdf": self.output_urdf,
            "meshes": [asdict(m) for m in self.meshes],
        }
        for role in ROLES:
            before, after = self.totals(role)
            d[f"{role}_triangles_before"] = before
            d[f"{role}_triangles_after"] = after
        return d


# ---------------------------------------------------------------------------
# URI resolution
# ---------------------------------------------------------------------------


def _find_package(name: str, urdf_dir: Path, package_paths: dict[str, Path]) -> Path:
    """Locate a ROS package directory for package://NAME/... URIs."""
    if name in package_paths:
        return package_paths[name]
    # 1. An ancestor directory of the URDF with that name (the usual *_description layout).
    for parent in [urdf_dir, *urdf_dir.parents]:
        if parent.name == name and (parent / "package.xml").exists():
            return parent
        if (parent / name / "package.xml").exists():
            return parent / name
    for parent in [urdf_dir, *urdf_dir.parents]:
        if parent.name == name:
            return parent
    # 2. A sourced ROS 2 workspace / install (ament index layout).
    for prefix in filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)):
        candidate = Path(prefix) / "share" / name
        if candidate.is_dir():
            return candidate
    raise UrdfError(
        f"Cannot resolve package '{name}'. Pass --package-path {name}=/path/to/{name} "
        "or source the ROS 2 workspace that provides it."
    )


def resolve_mesh_uri(uri: str, urdf_dir: Path, package_paths: dict[str, Path]) -> Path:
    """Turn a URDF mesh filename (package://, file:// or relative) into a local path."""
    parsed = urlparse(uri)
    if parsed.scheme == "package":
        return _find_package(parsed.netloc, urdf_dir, package_paths) / unquote(
            parsed.path.lstrip("/")
        )
    if parsed.scheme == "file":
        return Path(unquote(parsed.netloc + parsed.path))
    if parsed.scheme in ("", None) or len(parsed.scheme) == 1:  # relative or C:\ path
        p = Path(uri)
        return p if p.is_absolute() else (urdf_dir / p)
    raise UrdfError(f"Unsupported mesh URI scheme '{parsed.scheme}://' in '{uri}'.")


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def _parse(urdf_path: Path) -> ET.ElementTree:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    try:
        tree = ET.parse(urdf_path, parser=parser)
    except ET.ParseError as exc:
        raise UrdfError(f"'{urdf_path}' is not well-formed XML: {exc}") from exc
    if tree.getroot().tag != "robot":
        raise UrdfError(f"'{urdf_path}' root element is <{tree.getroot().tag}>, expected <robot>.")
    if any(isinstance(el.tag, str) and el.tag.startswith(f"{{{XACRO_NS}}}") for el in tree.iter()):
        raise UrdfError(
            f"'{urdf_path}' still contains xacro macros. Expand it first: "
            f"xacro {urdf_path.name} > {urdf_path.stem.removesuffix('.urdf')}.urdf"
        )
    return tree


def _collect(root: ET.Element, urdf_dir: Path, package_paths: dict[str, Path]):
    """Yield (role, link name, <mesh> element, resolved path) for every mesh reference."""
    for link in root.iter("link"):
        for role in ROLES:
            for el in link.findall(role):
                mesh_el = el.find("geometry/mesh")
                if mesh_el is None or "filename" not in mesh_el.attrib:
                    continue
                uri = mesh_el.attrib["filename"]
                path = resolve_mesh_uri(uri, urdf_dir, package_paths)
                yield role, link.attrib.get("name", "?"), mesh_el, uri, path


def _unique_name(stem: str, used: set[str]) -> str:
    name, i = stem, 1
    while name in used:
        name = f"{stem}_{i}"
        i += 1
    used.add(name)
    return name


def optimize_urdf(
    urdf_path: str,
    output_dir: str,
    collision_triangles: int,
    visual_triangles: int | None = None,
    package_paths: dict[str, str] | None = None,
    mesh_uri_prefix: str | None = None,
    compute_hausdorff: bool = True,
    dry_run: bool = False,
) -> UrdfSummary:
    """
    Optimize every mesh in a robot URDF and write a rewritten copy.

    collision_triangles  per-mesh triangle budget for <collision> meshes
    visual_triangles     per-mesh budget for <visual> meshes; None leaves visuals untouched
    package_paths        {"pkg_name": "/path/to/pkg"} overrides for package:// resolution
    mesh_uri_prefix      URI prefix for rewritten meshes, e.g.
                         "package://my_robot_description/meshes_optimized"; defaults to
                         paths relative to the output URDF ("meshes/collision/x.stl")
    dry_run              resolve and count everything, write nothing
    """
    if collision_triangles is None or collision_triangles < 1:
        raise ValueError("collision_triangles must be a positive integer.")
    if visual_triangles is not None and visual_triangles < 1:
        raise ValueError("visual_triangles must be a positive integer.")

    src = Path(urdf_path)
    if not src.exists():
        raise FileNotFoundError(f"URDF not found: '{urdf_path}'.")
    urdf_dir = src.resolve().parent
    pkg_paths = {k: Path(v).resolve() for k, v in (package_paths or {}).items()}
    out = Path(output_dir)

    tree = _parse(src)
    root = tree.getroot()
    robot = root.attrib.get("name", src.stem)
    budgets = {"collision": collision_triangles, "visual": visual_triangles}

    # Group references so a mesh shared by several links is processed once per role.
    results: dict[tuple[str, Path], MeshResult] = {}
    elements: dict[tuple[str, Path], list[ET.Element]] = {}
    for role, link, mesh_el, uri, path in _collect(root, urdf_dir, pkg_paths):
        key = (role, path.resolve())
        if key not in results:
            results[key] = MeshResult(role=role, source_uri=uri, source_path=str(path))
            elements[key] = []
        results[key].links.append(link)
        elements[key].append(mesh_el)

    if not results:
        raise UrdfError(f"'{urdf_path}' references no mesh files.")

    used_names: dict[str, set[str]] = {r: set() for r in ROLES}
    for key, res in results.items():
        role, path = key
        budget = budgets[role]
        if budget is None:
            res.status, res.reason = "kept", "no visual budget given; left untouched"
            continue
        if path.suffix.lower() not in SUPPORTED_FORMATS:
            res.status = "skipped"
            res.reason = f"'{path.suffix}' is not a supported mesh format; left untouched"
            log.warning("Skipping %s mesh '%s': %s", role, path.name, res.reason)
            continue
        if not path.exists():
            raise UrdfError(f"Mesh referenced by link '{res.links[0]}' not found: '{path}'.")

        mesh = clean_mesh(load_mesh(str(path)))
        res.original_triangles = len(mesh.triangles)
        name = _unique_name(path.stem, used_names[role]) + ".stl"
        rel = Path("meshes") / role / name
        res.output_path = str(out / rel)
        res.output_uri = (
            f"{mesh_uri_prefix.rstrip('/')}/{role}/{name}" if mesh_uri_prefix else (rel.as_posix())
        )

        if res.original_triangles <= budget:
            res.status, result, elapsed = "copied", mesh, 0.0
        else:
            res.status = "simplified"
            result, elapsed = (mesh, 0.0) if dry_run else simplify_mesh(mesh, budget)
        res.output_triangles = min(res.original_triangles, budget)
        if dry_run:
            continue

        res.output_triangles = len(result.triangles)
        res.processing_time_sec = round(elapsed, 4)
        if compute_hausdorff and res.status == "simplified":
            h = hausdorff_distance(mesh, result)
            res.hausdorff_max, res.hausdorff_rms = h["hausdorff_max"], h["hausdorff_rms"]
        save_mesh(result, res.output_path)
        for el in elements[key]:
            el.set("filename", res.output_uri)
        log.info(
            "%-9s %-28s %9s -> %7s tris  (%s)",
            role,
            path.name,
            f"{res.original_triangles:,}",
            f"{res.output_triangles:,}",
            ", ".join(res.links),
        )

    summary = UrdfSummary(
        robot=robot,
        input_urdf=str(src),
        output_urdf=None if dry_run else str(out / f"{robot}.urdf"),
        meshes=list(results.values()),
    )
    if dry_run:
        return summary

    out.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(summary.output_urdf, encoding="utf-8", xml_declaration=True)
    (out / "urdf_report.json").write_text(json.dumps(summary.to_dict(), indent=2) + "\n")
    log.info("Rewritten URDF saved -> %s", summary.output_urdf)
    return summary


def print_urdf_summary(summary: UrdfSummary, dry_run: bool = False) -> None:
    sep = "─" * 78
    print(f"\n{sep}")
    print(f"  URDF Optimization{' (dry run)' if dry_run else ''}: {summary.robot}")
    print(sep)
    print(f"  {'role':9s} {'mesh':26s} {'triangles':>19s} {'dev RMS':>9s}  status")
    kept = [m for m in summary.meshes if m.status == "kept"]
    for m in summary.meshes:
        if m.status == "kept":
            continue
        name = Path(m.source_path).name
        tris = (
            "-"
            if m.original_triangles is None
            else (f"{m.original_triangles:,} -> {m.output_triangles:,}")
        )
        dev = "-" if m.hausdorff_rms is None else f"{m.hausdorff_rms:.2e}"
        status = m.status if not dry_run or m.status == "skipped" else f"would be {m.status}"
        print(f"  {m.role:9s} {name[:26]:26s} {tris:>19s} {dev:>9s}  {status}")
    if kept:
        print(f"  ({len(kept)} visual mesh(es) left untouched: pass --visual N to optimize them)")
    print(sep)
    for role in ROLES:
        before, after = summary.totals(role)
        if before:
            pct = (1 - after / before) * 100
            print(f"  Total {role:9s}: {before:,} -> {after:,} triangles ({pct:.1f}% fewer)")
    if summary.output_urdf:
        print(f"  Output URDF    : {summary.output_urdf}")
        print("  dev RMS is the Hausdorff RMS deviation in mesh units (metres for most URDFs).")
    print(sep + "\n")
