"""
gui.py - Browser GUI for urdf-mesh-optimizer (optional `gui` extra).

    pip install "urdf-mesh-optimizer[gui]"
    urdf-mesh-optimizer --gui            # opens http://127.0.0.1:7860

Two tabs:
  Single mesh   upload a mesh, pick a triangle budget, compare before / after in 3D,
                read the deviation, download the result.
  Whole robot   upload a zipped *_description package, get the optimized URDF +
                meshes back as a zip, with a per-mesh report.

The processing functions (optimize_mesh_file, optimize_urdf_zip) do not import
gradio, so they are usable and testable without the GUI dependencies.
"""

from __future__ import annotations

import os
import shutil
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d

from optimizer import __version__
from optimizer.core import SUPPORTED_FORMATS, clean_mesh, load_mesh, save_mesh, simplify_mesh
from optimizer.metrics import hausdorff_distance
from optimizer.urdf_robot import UrdfError, optimize_urdf

PREVIEW_MAX_TRIANGLES = 300_000  # larger inputs are previewed (not processed) at this size
OUTPUT_FORMATS = ("stl", "obj", "ply", "glb")
EXAMPLES_DIR = Path(__file__).resolve().parent.parent / "examples" / "gui"


class GuiError(Exception):
    """A user-facing problem with an uploaded file or setting."""


# ---------------------------------------------------------------------------
# Processing (no gradio imports)
# ---------------------------------------------------------------------------


@dataclass
class MeshJob:
    original_preview: str
    optimized_preview: str
    download: str
    stats: dict
    preview_note: str | None = None


def _workdir() -> Path:
    return Path(tempfile.mkdtemp(prefix="urdf_mesh_optimizer_"))


BEFORE_COLOR = (0.55, 0.62, 0.72)  # steel blue-grey, as in the README figures
AFTER_COLOR = (0.97, 0.51, 0.40)  # collision-mesh orange


def _preview(
    mesh: o3d.geometry.TriangleMesh, path: Path, color: tuple, faceted: bool = False
) -> str:
    """Write a coloured GLB preview; faceted=True gives every triangle its own flat normal."""
    if faceted:
        v = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].reshape(-1, 3)
        m = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(v),
            o3d.utility.Vector3iVector(np.arange(len(v)).reshape(-1, 3)),
        )
        m.compute_triangle_normals()
        m.vertex_normals = o3d.utility.Vector3dVector(
            np.repeat(np.asarray(m.triangle_normals), 3, axis=0)
        )
    else:
        m = o3d.geometry.TriangleMesh(mesh)
        m.compute_vertex_normals()
    m.paint_uniform_color(color)
    if not o3d.io.write_triangle_mesh(str(path), m):
        raise GuiError("Could not write the 3D preview.")
    return str(path)


def optimize_mesh_file(
    path: str,
    target_triangles: int,
    out_format: str = "stl",
    compute_hausdorff: bool = True,
) -> MeshJob:
    """Decimate one mesh file; return preview files, the download file and the metrics."""
    src = Path(path)
    if src.suffix.lower() not in SUPPORTED_FORMATS:
        raise GuiError(
            f"'{src.suffix}' files aren't supported. Use one of: "
            + ", ".join(sorted(s.lstrip(".") for s in SUPPORTED_FORMATS))
        )
    if out_format not in OUTPUT_FORMATS:
        raise GuiError(f"Unsupported output format '{out_format}'.")
    target = int(target_triangles)
    if target < 4:
        raise GuiError("The triangle budget must be at least 4.")

    work = _workdir()
    mesh = clean_mesh(load_mesh(str(src)))
    original = len(mesh.triangles)
    if target >= original:
        result, elapsed = mesh, 0.0
    else:
        result, elapsed = simplify_mesh(mesh, target)

    download = work / f"{src.stem}_{len(result.triangles)}tris.{out_format}"
    save_mesh(result, str(download))

    note = None
    preview_src = mesh
    if original > PREVIEW_MAX_TRIANGLES:
        preview_src, _ = simplify_mesh(o3d.geometry.TriangleMesh(mesh), PREVIEW_MAX_TRIANGLES)
        note = (
            f"The original has {original:,} triangles; its 3D preview is shown at "
            f"{PREVIEW_MAX_TRIANGLES:,} to keep the browser responsive. "
            "Processing always uses the full mesh."
        )

    diag = float(np.linalg.norm(mesh.get_axis_aligned_bounding_box().get_extent()))
    stats = {
        "original_triangles": original,
        "optimized_triangles": len(result.triangles),
        "reduction_percent": round((1 - len(result.triangles) / original) * 100, 2),
        "time_sec": round(elapsed, 3),
        "file_size_before_mb": round(src.stat().st_size / 1e6, 3),
        "file_size_after_mb": round(download.stat().st_size / 1e6, 3),
        "size_diagonal": diag,
    }
    if compute_hausdorff and result is not mesh:
        h = hausdorff_distance(mesh, result)
        stats.update(h)

    return MeshJob(
        original_preview=_preview(preview_src, work / "original_preview.glb", BEFORE_COLOR),
        optimized_preview=_preview(
            result, work / "optimized_preview.glb", AFTER_COLOR, faceted=True
        ),
        download=str(download),
        stats=stats,
        preview_note=note,
    )


def _safe_extract(zip_path: str, dest: Path, max_total_bytes: int) -> None:
    with zipfile.ZipFile(zip_path) as zf:
        total = 0
        for info in zf.infolist():
            target = (dest / info.filename).resolve()
            if not str(target).startswith(str(dest.resolve()) + os.sep) and target != dest:
                raise GuiError(f"Refusing unsafe path in zip: '{info.filename}'.")
            total += info.file_size
            if total > max_total_bytes:
                raise GuiError(
                    f"The zip expands to more than {max_total_bytes // 1_000_000} MB; "
                    "use the command-line tool for robots this large."
                )
        zf.extractall(dest)


def _find_urdf(root: Path) -> Path:
    urdfs = sorted(p for p in root.rglob("*.urdf") if "__MACOSX" not in p.parts)
    if urdfs:
        return urdfs[0]
    if any(root.rglob("*.xacro")):
        raise GuiError(
            "Only .xacro files were found. Expand the robot first "
            "(xacro robot.urdf.xacro > robot.urdf) and include the .urdf in the zip."
        )
    raise GuiError("No .urdf file found in the zip.")


def _package_paths(root: Path) -> dict[str, str]:
    """Map ROS package names to directories: package.xml <name>, plus every directory name."""
    paths: dict[str, str] = {}
    for d in [root, *(p for p in root.rglob("*") if p.is_dir())]:
        paths.setdefault(d.name, str(d))
    for pkg_xml in root.rglob("package.xml"):
        try:
            name = ET.parse(pkg_xml).getroot().findtext("name")
        except ET.ParseError:
            continue
        if name:
            paths[name.strip()] = str(pkg_xml.parent)
    return paths


def optimize_urdf_zip(
    zip_path: str,
    collision_triangles: int,
    visual_triangles: int | None = None,
    max_total_bytes: int = 500_000_000,
) -> tuple[str, list[list], str]:
    """Run whole-robot mode on a zipped description package; return (zip, table rows, summary)."""
    if not zipfile.is_zipfile(zip_path):
        raise GuiError("Upload a .zip of your robot description package.")
    work = _workdir()
    src = work / "input"
    src.mkdir()
    _safe_extract(zip_path, src, max_total_bytes)
    urdf = _find_urdf(src)

    out = work / "optimized"
    try:
        summary = optimize_urdf(
            str(urdf),
            str(out),
            collision_triangles=int(collision_triangles),
            visual_triangles=int(visual_triangles) if visual_triangles else None,
            package_paths=_package_paths(src),
        )
    except (UrdfError, FileNotFoundError, ValueError, RuntimeError) as exc:
        raise GuiError(str(exc)) from exc

    rows = []
    for m in summary.meshes:
        rows.append(
            [
                m.role,
                Path(m.source_path).name,
                ", ".join(m.links),
                m.original_triangles if m.original_triangles is not None else "",
                m.output_triangles if m.output_triangles is not None else "",
                m.status,
            ]
        )
    lines = [f"**{summary.robot}**: `{urdf.relative_to(src)}`"]
    for role in ("collision", "visual"):
        before, after = summary.totals(role)
        if before:
            pct = (1 - after / before) * 100
            lines.append(f"- {role}: {before:,} → {after:,} triangles ({pct:.1f}% fewer)")
    if any(m.status == "kept" for m in summary.meshes):
        lines.append("- visual meshes were left unchanged (set a visual budget to optimize them)")

    archive = shutil.make_archive(str(work / f"{summary.robot}_optimized"), "zip", out)
    return archive, rows, "\n".join(lines)


def _fmt_stats(stats: dict, note: str | None) -> str:
    rows = [
        ("Triangles", f"{stats['original_triangles']:,} → {stats['optimized_triangles']:,}"),
        ("Reduction", f"{stats['reduction_percent']:.2f}%"),
        ("File size", f"{stats['file_size_before_mb']} MB → {stats['file_size_after_mb']} MB"),
        ("Decimation time", f"{stats['time_sec']:.2f} s"),
    ]
    if "hausdorff_rms" in stats:
        d = stats["size_diagonal"] or 1.0
        for key, label in (
            ("hausdorff_rms", "Deviation (RMS)"),
            ("hausdorff_max", "Deviation (max)"),
        ):
            v = stats[key]
            rows.append((label, f"{v:.4g} mesh units ({v / d * 100:.2f}% of part size)"))
    elif stats["optimized_triangles"] == stats["original_triangles"]:
        rows.append(("Note", "Already within budget; the mesh was only cleaned, not decimated."))
    text = "| Metric | Value |\n|---|---|\n" + "\n".join(f"| {a} | {b} |" for a, b in rows)
    if note:
        text += f"\n\n<sub>{note}</sub>"
    return text


# ---------------------------------------------------------------------------
# Gradio app
# ---------------------------------------------------------------------------


def _examples(*names: str) -> list[str]:
    return [str(EXAMPLES_DIR / n) for n in names if (EXAMPLES_DIR / n).exists()]


def build_app(max_upload_mb: float | None = None):
    """Build the Gradio Blocks app (requires the `gui` extra)."""
    try:
        import gradio as gr
    except ImportError as exc:  # pragma: no cover - exercised only without the extra
        raise RuntimeError(
            'The GUI needs extra packages: pip install "urdf-mesh-optimizer[gui]"'
        ) from exc

    limit = f" (up to {max_upload_mb:g} MB)" if max_upload_mb else ""

    def run_mesh(file, target, fmt):
        if file is None:
            raise gr.Error("Upload a mesh first.")
        if max_upload_mb and os.path.getsize(file) > max_upload_mb * 1e6:
            raise gr.Error(f"This demo accepts files up to {max_upload_mb:g} MB.")
        try:
            job = optimize_mesh_file(file, int(target), fmt)
        except (GuiError, FileNotFoundError, RuntimeError, ValueError) as exc:
            raise gr.Error(str(exc))
        return (
            job.original_preview,
            job.optimized_preview,
            job.download,
            _fmt_stats(job.stats, job.preview_note),
        )

    def run_urdf(file, collision, visual):
        if file is None:
            raise gr.Error("Upload a zipped robot description package first.")
        if max_upload_mb and os.path.getsize(file) > max_upload_mb * 1e6:
            raise gr.Error(f"This demo accepts files up to {max_upload_mb:g} MB.")
        try:
            archive, rows, summary = optimize_urdf_zip(file, int(collision), int(visual or 0))
        except GuiError as exc:
            raise gr.Error(str(exc))
        return archive, rows, summary

    with gr.Blocks(title="urdf-mesh-optimizer") as app:
        gr.Markdown(
            f"# urdf-mesh-optimizer <sub>v{__version__}</sub>\n"
            "Turn heavy CAD meshes into lightweight visual and collision meshes for "
            "ROS 2, MoveIt and Gazebo. "
            "[Docs and CLI](https://github.com/ookkshirsagar/urdf-mesh-optimizer)"
        )
        with gr.Tab("Single mesh"):
            with gr.Row():
                with gr.Column(scale=1, min_width=280):
                    mesh_in = gr.File(
                        label=f"Mesh: STL, OBJ, PLY, OFF, GLB{limit}",
                        file_types=[".stl", ".obj", ".ply", ".off", ".glb", ".gltf"],
                        type="filepath",
                    )
                    target = gr.Slider(
                        100, 50_000, value=2_000, step=100, label="Target triangles"
                    )
                    fmt = gr.Dropdown(list(OUTPUT_FORMATS), value="stl", label="Output format")
                    go = gr.Button("Optimize", variant="primary")
                    examples = _examples("spur_gear.stl", "weld_neck_flange.stl")
                    if examples:
                        gr.Examples([[e] for e in examples], inputs=[mesh_in], label="Examples")
                with gr.Column(scale=3):
                    with gr.Row():
                        view = dict(height=420, clear_color=(0.95, 0.96, 0.98, 1.0))
                        before = gr.Model3D(label="Before: original mesh", **view)
                        after = gr.Model3D(label="After: every triangle shown", **view)
                    stats = gr.Markdown()
                    download = gr.File(label="Download optimized mesh")
            go.click(run_mesh, [mesh_in, target, fmt], [before, after, download, stats])

        with gr.Tab("Whole robot (URDF)"):
            gr.Markdown(
                "Zip your robot's description package (the `.urdf` plus its `meshes/` "
                "folder) and upload it. Every collision mesh is reduced to the budget and "
                "the URDF is rewritten to point at the new files. Joints, inertials and "
                "everything else are kept as they are."
            )
            with gr.Row():
                with gr.Column(scale=1, min_width=280):
                    urdf_in = gr.File(
                        label=f"Robot description (.zip){limit}",
                        file_types=[".zip"],
                        type="filepath",
                    )
                    coll = gr.Number(
                        value=2_000, precision=0, label="Collision triangles per mesh"
                    )
                    vis = gr.Number(
                        value=0, precision=0, label="Visual triangles per mesh (0 = keep)"
                    )
                    go_urdf = gr.Button("Optimize robot", variant="primary")
                    examples = _examples("demo_cell_description.zip")
                    if examples:
                        gr.Examples([[e] for e in examples], inputs=[urdf_in], label="Example")
                with gr.Column(scale=3):
                    urdf_summary = gr.Markdown()
                    urdf_table = gr.Dataframe(
                        headers=["role", "mesh", "links", "before", "after", "status"],
                        label="Per-mesh results",
                        interactive=False,
                    )
                    urdf_out = gr.File(label="Download optimized robot (.zip)")
            go_urdf.click(run_urdf, [urdf_in, coll, vis], [urdf_out, urdf_table, urdf_summary])
    return app


def launch(port: int = 7860, share: bool = False, max_upload_mb: float | None = None) -> None:
    import gradio as gr

    app = build_app(max_upload_mb=max_upload_mb)
    app.queue(default_concurrency_limit=2)
    app.launch(
        server_port=port,
        share=share,
        theme=gr.themes.Soft(),
        max_file_size=f"{max_upload_mb:g}mb" if max_upload_mb else None,
        allowed_paths=[str(EXAMPLES_DIR)],
    )
