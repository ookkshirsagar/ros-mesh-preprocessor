"""
docs/generate_figures.py - Regenerate every README figure from real tool output.

Builds a synthetic 6-axis robot arm from open3d primitives, runs it through the
same clean_mesh() / simplify_mesh() / hausdorff_distance() functions the CLI
uses, and renders:

    docs/img/hero.png                 raw CAD -> visual mesh -> collision mesh
    docs/img/hausdorff_heatmap.png    per-vertex deviation of the collision mesh
    docs/img/benchmark_tradeoff.png   plots of benchmarks/performance_results.json

Nothing in these images is hand-drawn or retouched: every triangle count and
deviation number is printed straight from the pipeline.

Requires the 'viz' extra:  pip install -e ".[viz]"

Usage:
    python docs/generate_figures.py
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402
from matplotlib import cm, colors  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

from optimizer.core import clean_mesh, simplify_mesh  # noqa: E402
from optimizer.metrics import hausdorff_distance  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
IMG = ROOT / "docs" / "img"
BENCH_JSON = ROOT / "benchmarks" / "performance_results.json"

VISUAL_TARGET = 20_000
COLLISION_TARGET = 1_500

# GitHub-dark palette so the figures sit naturally on the repo page.
BG = "#0d1117"
FG = "#e6edf3"
MUTED = "#8b949e"
GRID = "#30363d"
ACCENT = {"raw": "#58a6ff", "visual": "#3fb950", "collision": "#f78166"}


# ---------------------------------------------------------------------------
# Synthetic robot arm (stand-in for a vendor CAD export)
# ---------------------------------------------------------------------------


def _rot(axis: str, deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _cyl(r, h, center, axis="z", res=160, split=64):
    m = o3d.geometry.TriangleMesh.create_cylinder(radius=r, height=h, resolution=res, split=split)
    if axis == "x":
        m.rotate(_rot("y", 90), center=(0, 0, 0))
    elif axis == "y":
        m.rotate(_rot("x", 90), center=(0, 0, 0))
    m.translate(center)
    return m


def _box(sx, sy, sz, center, rot=None):
    m = o3d.geometry.TriangleMesh.create_box(sx, sy, sz)
    m.translate((-sx / 2, -sy / 2, -sz / 2))
    m = m.subdivide_midpoint(5)
    if rot is not None:
        m.rotate(rot, center=(0, 0, 0))
    m.translate(center)
    return m


def build_robot_arm() -> o3d.geometry.TriangleMesh:
    """A dense, CAD-like articulated arm on a pedestal (~200k triangles)."""
    parts = [
        _box(0.70, 0.70, 0.10, (0, 0, 0.05)),  # floor plate
        _cyl(0.22, 0.30, (0, 0, 0.25)),  # pedestal
        _cyl(0.26, 0.06, (0, 0, 0.43)),  # base flange
        _cyl(0.18, 0.22, (0, 0, 0.57)),  # J1 turret
        _cyl(0.15, 0.30, (0, 0, 0.74), axis="y"),  # J2 shoulder motor
    ]
    # Upper arm, tilted forward
    upper_rot = _rot("y", 28)
    upper_len = 0.75
    shoulder = np.array([0, 0, 0.74])
    elbow = shoulder + upper_rot @ np.array([0, 0, upper_len])
    parts.append(_box(0.16, 0.20, upper_len, (shoulder + elbow) / 2, rot=upper_rot))
    parts.append(_cyl(0.12, 0.28, elbow, axis="y"))  # J3 elbow
    # Forearm, reaching out and slightly down
    fore_rot = _rot("y", 100)
    fore_len = 0.62
    wrist = elbow + fore_rot @ np.array([0, 0, fore_len])
    parts.append(_cyl(0.075, fore_len, (elbow + wrist) / 2, axis="x"))
    parts.append(_cyl(0.09, 0.16, wrist, axis="y"))  # J5 wrist
    tool = wrist + fore_rot @ np.array([0, 0, 0.12])
    parts.append(_cyl(0.06, 0.08, tool, axis="x"))  # J6 tool flange
    # Parallel gripper
    gbody = tool + np.array([0.08, 0, 0])
    parts.append(_box(0.06, 0.18, 0.10, gbody))
    for y in (-0.06, 0.06):
        parts.append(_box(0.12, 0.025, 0.05, gbody + np.array([0.09, y, 0])))

    arm = o3d.geometry.TriangleMesh()
    for p in parts:
        arm += p
    arm = clean_mesh(arm)
    arm.compute_triangle_normals()
    return arm


# ---------------------------------------------------------------------------
# Rendering helpers
# ---------------------------------------------------------------------------

LIGHT = np.array([0.45, -0.55, 0.75])
LIGHT = LIGHT / np.linalg.norm(LIGHT)


def _shade(mesh, base_rgb, face_values=None, cmap=None, norm=None):
    mesh.compute_triangle_normals()
    n = np.asarray(mesh.triangle_normals)
    lam = np.clip(n @ LIGHT, 0, 1)
    k = 0.35 + 0.65 * lam
    if face_values is not None:
        rgb = cmap(norm(face_values))[:, :3]
    else:
        rgb = np.tile(colors.to_rgb(base_rgb), (len(n), 1))
    return np.clip(rgb * k[:, None], 0, 1)


def _draw(ax, mesh, color, edges=False, face_values=None, cmap=None, norm=None):
    v = np.asarray(mesh.vertices)
    t = np.asarray(mesh.triangles)
    fc = _shade(mesh, color, face_values, cmap, norm)
    pc = Poly3DCollection(
        v[t],
        facecolors=fc,
        edgecolors=(0, 0, 0, 0.55) if edges else fc,
        linewidths=0.25 if edges else 0.0,
    )
    ax.add_collection3d(pc)
    _frame(ax)


def _frame(ax):
    ax.set_xlim(-0.55, 1.15)
    ax.set_ylim(-0.85, 0.85)
    ax.set_zlim(0.0, 1.45)
    ax.set_box_aspect((1.7, 1.7, 1.45))
    ax.view_init(elev=18, azim=-58)
    ax.set_facecolor(BG)
    ax.axis("off")


def _fig(w, h):
    fig = plt.figure(figsize=(w, h), dpi=130)
    fig.patch.set_facecolor(BG)
    return fig


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------


def make_hero(raw, visual, collision, stats):
    fig = _fig(15, 5.6)
    panels = [
        ("RAW CAD EXPORT", raw, "raw", False, "too dense for real-time planning"),
        ("VISUAL MESH", visual, "visual", False, "for RViz / Gazebo rendering"),
        ("COLLISION MESH", collision, "collision", True, "for MoveIt / FCL collision checks"),
    ]
    for i, (title, mesh, key, edges, sub) in enumerate(panels):
        ax = fig.add_axes([i / 3, 0.0, 1 / 3, 0.80], projection="3d")
        _draw(ax, mesh, ACCENT[key], edges=edges)
        cx = (i + 0.5) / 3
        fig.text(cx, 0.92, title, ha="center", color=ACCENT[key], fontsize=15, weight="bold")
        fig.text(
            cx,
            0.865,
            f"{len(mesh.triangles):,} triangles",
            ha="center",
            color=FG,
            fontsize=13,
            family="monospace",
        )
        fig.text(cx, 0.825, sub, ha="center", color=MUTED, fontsize=10.5)
        if i < 2:
            fig.text((i + 1) / 3, 0.45, "➜", ha="center", va="center", color=MUTED, fontsize=30)
    fig.text(
        0.5,
        0.035,
        f"collision mesh: {stats['reduction']:.2f}% fewer triangles  ·  "
        f"Hausdorff RMS {stats['rms'] * 1000:.1f} mm  ·  "
        f"max {stats['max'] * 1000:.1f} mm  (arm reach ≈ 1.6 m)",
        ha="center",
        color=MUTED,
        fontsize=11,
        family="monospace",
    )
    out = IMG / "hero.png"
    fig.savefig(out, facecolor=BG)
    plt.close(fig)
    return out


def _per_vertex_deviation(original, simplified):
    """Unsigned distance from every original vertex to the simplified surface."""
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(simplified))
    pts = o3d.core.Tensor(np.asarray(original.vertices), dtype=o3d.core.Dtype.Float32)
    return scene.compute_distance(pts).numpy()


def make_heatmap(raw, collision):
    d_vert = _per_vertex_deviation(raw, collision) * 1000.0  # m -> mm
    tris = np.asarray(raw.triangles)
    d_face = d_vert[tris].mean(axis=1)
    vmax = float(np.percentile(d_face, 99.5))
    # Neutral grey = untouched geometry, warming to red where QEM removed detail.
    cmap = colors.LinearSegmentedColormap.from_list(
        "deviation", ["#c9d1d9", "#58a6ff", "#f2cc60", "#f78166", "#da3633"]
    )
    norm = colors.Normalize(vmin=0.0, vmax=vmax)

    fig = _fig(9.5, 7)
    ax = fig.add_axes([-0.06, -0.08, 0.95, 0.98], projection="3d")
    _draw(ax, raw, "#888", face_values=d_face, cmap=cmap, norm=norm)

    cax = fig.add_axes([0.85, 0.18, 0.025, 0.56])
    cb = fig.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), cax=cax)
    cb.set_label("deviation from collision mesh [mm]", color=FG, fontsize=11)
    cb.ax.tick_params(colors=FG, labelsize=9)
    cb.outline.set_edgecolor(GRID)

    fig.text(0.05, 0.93, "Where did the geometry go?", color=FG, fontsize=17, weight="bold")
    fig.text(
        0.05,
        0.885,
        f"Raw arm coloured by one-sided distance to its "
        f"{len(collision.triangles):,}-triangle collision mesh  ·  "
        f"median {np.median(d_vert):.1f} mm  ·  p99 {np.percentile(d_vert, 99):.1f} mm",
        color=MUTED,
        fontsize=10.5,
    )
    out = IMG / "hausdorff_heatmap.png"
    fig.savefig(out, facecolor=BG)
    plt.close(fig)
    return out


def _style(ax):
    ax.set_facecolor(BG)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors=FG, labelsize=10)
    ax.xaxis.label.set_color(FG)
    ax.yaxis.label.set_color(FG)
    ax.grid(True, color=GRID, linewidth=0.6, alpha=0.8)


def make_benchmark_plot():
    data = json.loads(BENCH_JSON.read_text())["results"]
    data = sorted(data, key=lambda r: r["reduced_triangles"])
    tri = np.array([r["reduced_triangles"] for r in data])
    hmax = np.array([r["hausdorff_max"] for r in data])
    hrms = np.array([r["hausdorff_rms"] for r in data])
    size = np.array([r["file_size_after_mb"] for r in data])
    t = np.array([r["processing_time_sec"] for r in data])
    src_size = data[0]["file_size_before_mb"]
    src_tri = data[0]["original_triangles"]

    fig = _fig(14, 4.8)
    fig.subplots_adjust(left=0.06, right=0.98, bottom=0.15, top=0.80, wspace=0.28)

    ax = fig.add_subplot(1, 3, 1)
    _style(ax)
    ax.plot(tri, hmax, "o-", color=ACCENT["collision"], lw=2, label="Hausdorff max")
    ax.plot(tri, hrms, "o-", color=ACCENT["raw"], lw=2, label="Hausdorff RMS")
    ax.set_xscale("log")
    ax.set_ylim(0, max(hmax) * 1.35)
    ax.set_xlabel("target triangles (log)")
    ax.set_ylabel("deviation [mesh units]")
    ax.set_title("Shape error stays flat", color=FG, fontsize=12.5, weight="bold", loc="left")
    leg = ax.legend(facecolor=BG, edgecolor=GRID, fontsize=9.5)
    for txt in leg.get_texts():
        txt.set_color(FG)

    ax = fig.add_subplot(1, 3, 2)
    _style(ax)
    labels = [f"{n // 1000}k" if n >= 1000 else str(n) for n in tri]
    ax.bar(
        ["source"] + labels,
        [src_size] + list(size),
        color=[MUTED] + [ACCENT["visual"]] * len(size),
    )
    ax.set_yscale("log")
    ax.set_ylabel("file size [MB] (log)")
    ax.set_xlabel("target triangles")
    ax.set_title(
        f"{src_size:.1f} MB source → KB-scale assets",
        color=FG,
        fontsize=12.5,
        weight="bold",
        loc="left",
    )

    ax = fig.add_subplot(1, 3, 3)
    _style(ax)
    ax.bar(labels, t, color=ACCENT["raw"])
    ax.set_ylabel("wall time [s]")
    ax.set_xlabel("target triangles")
    ax.set_ylim(0, max(t) * 1.3)
    ax.set_title("Seconds, not minutes", color=FG, fontsize=12.5, weight="bold", loc="left")

    fig.text(
        0.06,
        0.92,
        f"QEM decimation of a {src_tri:,}-triangle mesh",
        color=FG,
        fontsize=15,
        weight="bold",
    )
    fig.text(
        0.06,
        0.875,
        "data: benchmarks/performance_results.json " "(python benchmarks/generate_report.py)",
        color=MUTED,
        fontsize=10,
        family="monospace",
    )
    out = IMG / "benchmark_tradeoff.png"
    fig.savefig(out, facecolor=BG)
    plt.close(fig)
    return out


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    print("[figures] building synthetic robot arm ...")
    raw = build_robot_arm()
    print(f"[figures] raw arm: {len(raw.triangles):,} triangles")

    visual, _ = simplify_mesh(o3d.geometry.TriangleMesh(raw), VISUAL_TARGET)
    collision, _ = simplify_mesh(o3d.geometry.TriangleMesh(raw), COLLISION_TARGET)
    h = hausdorff_distance(raw, collision, n_samples=50_000)
    stats = {
        "reduction": (1 - len(collision.triangles) / len(raw.triangles)) * 100,
        "rms": h["hausdorff_rms"],
        "max": h["hausdorff_max"],
    }
    print(f"[figures] visual {len(visual.triangles):,} / collision {len(collision.triangles):,}")

    for out in (
        make_hero(raw, visual, collision, stats),
        make_heatmap(raw, collision),
        make_benchmark_plot(),
    ):
        print(f"[figures] saved -> {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
