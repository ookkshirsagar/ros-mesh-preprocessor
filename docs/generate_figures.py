"""
docs/generate_figures.py - Regenerate every README figure from real pipeline output.

Builds six parametric industrial components (docs/figures/parts.py), runs each
through the same clean_mesh() / simplify_mesh() / hausdorff_distance() the CLI
uses, renders them headlessly (docs/figures/render.py), and lays out:

    docs/img/hero.png                 servo motor: CAD -> visual mesh -> collision mesh
    docs/img/gallery.png              3x2 split-view grid: CAD vs collision mesh, with metrics
    docs/img/hausdorff_heatmap.png    where the geometry went: per-vertex deviation
    docs/img/benchmark_tradeoff.png   plots of benchmarks/performance_results.json
    docs/figures/results.json         every number printed on the figures

Nothing is retouched: every triangle count and deviation is computed here.

Requires the 'viz' extra (pip install -e ".[viz]") and an EGL-capable
OpenGL driver (Mesa's llvmpipe is fine; on Ubuntu: apt install libegl1 libgl1-mesa-dri).

Usage:
    python docs/generate_figures.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402
import trimesh  # noqa: E402
from PIL import Image, ImageDraw, ImageFilter, ImageFont  # noqa: E402

DOCS = Path(__file__).resolve().parent
ROOT = DOCS.parent
sys.path.insert(0, str(DOCS))

from figures.parts import CATALOG  # noqa: E402
from figures.render import Camera, render  # noqa: E402

from optimizer.core import clean_mesh, simplify_mesh  # noqa: E402
from optimizer.metrics import hausdorff_distance  # noqa: E402

IMG = DOCS / "img"
RESULTS_JSON = DOCS / "figures" / "results.json"
BENCH_JSON = ROOT / "benchmarks" / "performance_results.json"

VISUAL_TARGET = 20_000
# Collision budgets per part: enough to keep teeth / slots recognisable.
COLLISION_TARGET = {
    "servo_motor": 1_500,
    "spur_gear": 3_000,
    "weld_neck_flange": 2_000,
    "tslot_extrusion": 1_000,
    "pillow_block": 2_000,
    "timing_pulley": 2_500,
}
MATERIAL = {
    "servo_motor": "motor",
    "spur_gear": "steel",
    "weld_neck_flange": "steel",
    "tslot_extrusion": "aluminium",
    "pillow_block": "cast_iron",
    "timing_pulley": "aluminium",
}

# GitHub-dark palette so the figures sit naturally on the repo page.
BG = (13, 17, 23)
PANEL = (22, 27, 34)
BORDER = (48, 54, 61)
FG = (230, 237, 243)
MUTED = (139, 148, 158)
BLUE = (88, 166, 255)
GREEN = (63, 185, 80)
ORANGE = (247, 129, 102)


def _hex(rgb):
    return "#%02x%02x%02x" % rgb


# ---------------------------------------------------------------------------
# Fonts / drawing helpers
# ---------------------------------------------------------------------------

_FONT_CANDIDATES = {
    "bold": ["/usr/share/fonts/opentype/inter/Inter-Bold.otf", "DejaVuSans-Bold.ttf"],
    "semibold": ["/usr/share/fonts/opentype/inter/Inter-SemiBold.otf", "DejaVuSans-Bold.ttf"],
    "regular": ["/usr/share/fonts/opentype/inter/Inter-Regular.otf", "DejaVuSans.ttf"],
    "mono": ["/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", "DejaVuSansMono.ttf"],
}
_MPL_FONTS = Path(matplotlib.get_data_path()) / "fonts" / "ttf"


def font(kind: str, size: int) -> ImageFont.FreeTypeFont:
    for cand in _FONT_CANDIDATES[kind]:
        p = Path(cand) if Path(cand).is_absolute() else _MPL_FONTS / cand
        if p.exists():
            return ImageFont.truetype(str(p), size)
    return ImageFont.load_default(size)


def text(draw, xy, s, kind="regular", size=16, fill=FG, anchor="la"):
    draw.text(xy, s, font=font(kind, size), fill=fill, anchor=anchor)


def stage(size, top=(32, 38, 47), bottom=(15, 19, 25)):
    """Studio backdrop: soft vertical gradient plus a floor glow under the part."""
    w, h = size
    t = np.linspace(0, 1, h)[:, None, None]
    arr = (np.asarray(top) * (1 - t) + np.asarray(bottom) * t).astype(np.uint8)
    bg = Image.fromarray(np.repeat(arr, w, axis=1), "RGB").convert("RGBA")
    glow = Image.new("L", (w, h), 0)
    ImageDraw.Draw(glow).ellipse((w * 0.15, h * 0.62, w * 0.85, h * 0.92), fill=34)
    glow = glow.filter(ImageFilter.GaussianBlur(h * 0.07))
    bg.paste(Image.new("RGBA", (w, h), (130, 150, 180, 255)), (0, 0), glow)
    return bg


def paste_rounded(canvas, img, xy, radius=14):
    mask = Image.new("L", img.size, 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, img.size[0] - 1, img.size[1] - 1), radius, 255)
    canvas.paste(img, xy, mask)


# ---------------------------------------------------------------------------
# Pipeline: build, clean, decimate, measure
# ---------------------------------------------------------------------------


def to_o3d(m: trimesh.Trimesh) -> o3d.geometry.TriangleMesh:
    return o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(np.asarray(m.vertices)),
        o3d.utility.Vector3iVector(np.asarray(m.faces)),
    )


def to_tri(m: o3d.geometry.TriangleMesh) -> trimesh.Trimesh:
    return trimesh.Trimesh(np.asarray(m.vertices), np.asarray(m.triangles), process=False)


def process_parts():
    parts = {}
    for key, (name, spec, build) in CATALOG.items():
        raw = build()
        raw_o3d = clean_mesh(to_o3d(raw))
        visual, _ = simplify_mesh(o3d.geometry.TriangleMesh(raw_o3d), VISUAL_TARGET)
        collision, t = simplify_mesh(o3d.geometry.TriangleMesh(raw_o3d), COLLISION_TARGET[key])
        h = hausdorff_distance(raw_o3d, collision, n_samples=50_000)
        stats = {
            "name": name,
            "spec": spec,
            "bbox_mm": (raw.extents * 1000).round(1).tolist(),
            "raw_triangles": len(raw.faces),
            "visual_triangles": len(visual.triangles),
            "collision_triangles": len(collision.triangles),
            "reduction_percent": round((1 - len(collision.triangles) / len(raw.faces)) * 100, 2),
            "simplify_time_sec": round(t, 3),
            "hausdorff_max_mm": round(h["hausdorff_max"] * 1000, 3),
            "hausdorff_mean_mm": round(h["hausdorff_mean"] * 1000, 3),
            "hausdorff_rms_mm": round(h["hausdorff_rms"] * 1000, 3),
        }
        print(
            f"[figures] {key:18s} {stats['raw_triangles']:>9,} -> "
            f"{stats['collision_triangles']:>5,} tris  RMS {stats['hausdorff_rms_mm']:.3f} mm"
        )
        parts[key] = {
            "raw": raw,
            "raw_o3d": raw_o3d,
            "visual": to_tri(visual),
            "collision": to_tri(collision),
            "collision_o3d": collision,
            "stats": stats,
        }
    return parts


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------


def make_hero(p):
    """Servo motor: CAD -> visual -> collision, three panels on one stage."""
    pw, ph, gap, x0 = 560, 470, 40, 40
    W, H = 2 * x0 + 3 * pw + 2 * gap, 690
    canvas = stage((W, H), top=(30, 36, 45), bottom=(13, 17, 23))
    d = ImageDraw.Draw(canvas)
    cam = Camera(p["raw"], fill=0.80)
    s = p["stats"]
    panels = [
        ("CAD EXPORT", s["raw_triangles"], BLUE, "dense vendor STL", "motor", "smooth", "raw"),
        ("VISUAL MESH", s["visual_triangles"], GREEN, "RViz / Gazebo rendering", "motor",
         "smooth", "visual"),
        ("COLLISION MESH", s["collision_triangles"], ORANGE, "MoveIt / FCL collision checking",
         "collision", "facets", "collision"),
    ]  # fmt: skip
    for i, (title, tris, color, sub, mat, mode, mesh_key) in enumerate(panels):
        x = x0 + i * (pw + gap)
        cx = x + pw // 2
        canvas.alpha_composite(render(p[mesh_key], cam, (pw, ph), mat, mode=mode), (x, 128))
        text(d, (cx, 42), title, "bold", 26, color, "mm")
        text(d, (cx, 80), f"{tris:,} triangles", "mono", 22, FG, "mm")
        text(d, (cx, 110), sub, "regular", 17, MUTED, "mm")
        if i < 2:
            ax = x + pw + gap // 2
            d.polygon([(ax - 9, 350), (ax + 9, 363), (ax - 9, 376)], fill=MUTED)

    strip = (
        f"{s['name']}  ·  {s['reduction_percent']:.2f}% fewer collision triangles  ·  "
        f"Hausdorff RMS {s['hausdorff_rms_mm']:.2f} mm, max {s['hausdorff_max_mm']:.2f} mm  ·  "
        f"decimated in {s['simplify_time_sec']:.2f} s"
    )
    d.line((60, H - 62, W - 60, H - 62), fill=BORDER, width=1)
    text(d, (W // 2, H - 32), strip, "mono", 17, MUTED, "mm")
    out = IMG / "hero.png"
    canvas.convert("RGB").save(out, optimize=True)
    return out


def _split_view(raw_img, col_img):
    """Left half CAD render, right half collision mesh, slider-style divider."""
    w, h = raw_img.size
    out = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    out.alpha_composite(raw_img.crop((0, 0, w // 2, h)), (0, 0))
    out.alpha_composite(col_img.crop((w // 2, 0, w, h)), (w // 2, 0))
    d = ImageDraw.Draw(out)
    m = w // 2
    d.line((m, 0, m, h), fill=(230, 237, 243, 170), width=2)
    r = 14
    d.ellipse((m - r, h // 2 - r, m + r, h // 2 + r), fill=FG + (255,))
    d.polygon([(m - 3, h // 2 - 5), (m - 8, h // 2), (m - 3, h // 2 + 5)], fill=BG)
    d.polygon([(m + 3, h // 2 - 5), (m + 8, h // 2), (m + 3, h // 2 + 5)], fill=BG)
    return out


def make_gallery(parts):
    cols, rows = 3, 2
    tw, th, foot = 580, 440, 118
    gap, margin, header = 24, 40, 124
    W = margin * 2 + cols * tw + (cols - 1) * gap
    H = header + margin + rows * (th + foot) + (rows - 1) * gap
    canvas = Image.new("RGBA", (W, H), BG + (255,))
    d = ImageDraw.Draw(canvas)
    text(d, (margin, 40), "Industrial components: CAD export vs. collision mesh", "bold", 32, FG)
    text(
        d,
        (margin, 86),
        "Left of each divider: the dense CAD tessellation.  Right: the decimated collision mesh "
        "with every triangle drawn.  Same part, same camera.",
        "regular",
        18,
        MUTED,
    )
    for i, (key, p) in enumerate(parts.items()):
        cx = margin + (i % cols) * (tw + gap)
        cy = header + (i // cols) * (th + foot + gap)
        s = p["stats"]
        cam = Camera(p["raw"], fill=0.82)
        raw_img = render(p["raw"], cam, (tw, th), MATERIAL[key])
        col_img = render(p["collision"], cam, (tw, th), "collision", mode="facets")
        tile = stage((tw, th + foot))
        tile.alpha_composite(_split_view(raw_img, col_img), (0, 0))
        td = ImageDraw.Draw(tile)
        text(td, (18, 16), "CAD", "bold", 15, BLUE)
        text(td, (18, 37), f"{s['raw_triangles']:,} tris", "mono", 15, FG)
        text(td, (tw - 18, 16), "COLLISION", "bold", 15, ORANGE, "ra")
        text(td, (tw - 18, 37), f"{s['collision_triangles']:,} tris", "mono", 15, FG, "ra")

        fy = th
        td.rectangle((0, fy, tw, th + foot), fill=PANEL)
        td.line((0, fy, tw, fy), fill=BORDER, width=1)
        text(td, (20, fy + 16), s["name"], "bold", 21, FG)
        text(td, (20, fy + 46), s["spec"], "regular", 15, MUTED)
        metrics = [
            (f"−{s['reduction_percent']:.1f}%", "triangles"),
            (f"{s['hausdorff_rms_mm']:.2f} mm", "RMS deviation"),
            (f"{s['hausdorff_max_mm']:.2f} mm", "max deviation"),
            (f"{s['simplify_time_sec']:.2f} s", "decimation"),
        ]
        mw = (tw - 40) / 4
        for j, (val, lab) in enumerate(metrics):
            mx = 20 + j * mw
            text(td, (mx, fy + 76), val, "semibold", 18, GREEN if j == 0 else FG)
            text(td, (mx, fy + 99), lab, "regular", 12, MUTED)
        paste_rounded(canvas, tile, (cx, cy))
        d.rounded_rectangle((cx, cy, cx + tw - 1, cy + th + foot - 1), 14, outline=BORDER, width=2)
    out = IMG / "gallery.png"
    canvas.convert("RGB").save(out, optimize=True)
    return out


def _deviation(raw_o3d, collision_o3d):
    """Unsigned distance (mm) from every CAD vertex to the collision surface."""
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(collision_o3d))
    pts = o3d.core.Tensor(np.asarray(raw_o3d.vertices), dtype=o3d.core.Dtype.Float32)
    return scene.compute_distance(pts).numpy() * 1000.0


def make_heatmap(parts, keys=("spur_gear", "weld_neck_flange", "pillow_block")):
    tw, th = 580, 480
    margin, header, gap, cbar_h = 40, 124, 24, 100
    W = margin * 2 + len(keys) * tw + (len(keys) - 1) * gap
    H = header + th + cbar_h + 20
    canvas = Image.new("RGBA", (W, H), BG + (255,))
    d = ImageDraw.Draw(canvas)

    devs = {}
    for k in keys:
        p = parts[k]
        dv = _deviation(p["raw_o3d"], p["collision_o3d"])
        devs[k] = (dv, dv[np.asarray(p["raw_o3d"].triangles)].mean(axis=1))
    vmax = float(np.ceil(max(np.percentile(f, 99.5) for _, f in devs.values()) * 2) / 2)
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
        "dev", ["#c9d1d9", "#58a6ff", "#f2cc60", "#f78166", "#da3633"]
    )
    text(d, (margin, 40), "Where did the geometry go?", "bold", 32, FG)
    text(
        d,
        (margin, 86),
        "CAD surface coloured by its distance to the collision mesh.  Flat faces survive "
        "untouched; error concentrates on curved surfaces and bores.",
        "regular",
        18,
        MUTED,
    )
    for i, k in enumerate(keys):
        p = parts[k]
        dv, fdev = devs[k]
        raw = to_tri(p["raw_o3d"])
        rgba = (cmap(np.clip(fdev / vmax, 0, 1)) * 255).astype(np.uint8)
        img = render(raw, Camera(raw, fill=0.86), (tw, th - 80), mode="colors", face_colors=rgba)
        tile = stage((tw, th))
        tile.alpha_composite(img, (0, 66))
        td = ImageDraw.Draw(tile)
        text(td, (20, 18), p["stats"]["name"], "bold", 20, FG)
        text(
            td,
            (20, 47),
            f"vs {p['stats']['collision_triangles']:,}-tri collision mesh  ·  "
            f"median {np.median(dv):.2f} mm  ·  p99 {np.percentile(dv, 99):.2f} mm",
            "mono",
            13,
            MUTED,
        )
        x = margin + i * (tw + gap)
        paste_rounded(canvas, tile, (x, header))
        d.rounded_rectangle((x, header, x + tw - 1, header + th - 1), 14, outline=BORDER, width=2)

    bx0, bx1, by = margin + 260, W - margin - 260, header + th + 36
    grad = (cmap(np.linspace(0, 1, bx1 - bx0))[:, :3] * 255).astype(np.uint8)
    paste_rounded(canvas, Image.fromarray(np.repeat(grad[None], 14, axis=0), "RGB"), (bx0, by), 7)
    for v in np.linspace(0, vmax, 6):
        text(d, (bx0 + (bx1 - bx0) * v / vmax, by + 36), f"{v:.1f}", "mono", 14, MUTED, "mm")
    text(d, (bx0 - 18, by + 7), "0 mm  (unchanged)", "regular", 15, MUTED, "rm")
    text(d, (bx1 + 18, by + 7), f"≥ {vmax:.1f} mm deviation", "regular", 15, MUTED, "lm")
    out = IMG / "hausdorff_heatmap.png"
    canvas.convert("RGB").save(out, optimize=True)
    return out


def _style(ax):
    ax.set_facecolor(_hex(BG))
    for s in ax.spines.values():
        s.set_color(_hex(BORDER))
    ax.tick_params(colors=_hex(FG), labelsize=10)
    ax.xaxis.label.set_color(_hex(FG))
    ax.yaxis.label.set_color(_hex(FG))
    ax.grid(True, color=_hex(BORDER), linewidth=0.6, alpha=0.8)


def make_benchmark_plot():
    data = json.loads(BENCH_JSON.read_text())["results"]
    data = sorted(data, key=lambda r: r["reduced_triangles"])
    tri = np.array([r["reduced_triangles"] for r in data])
    hmax = np.array([r["hausdorff_max"] for r in data])
    hrms = np.array([r["hausdorff_rms"] for r in data])
    size = np.array([r["file_size_after_mb"] for r in data])
    t = np.array([r["processing_time_sec"] for r in data])
    src_size, src_tri = data[0]["file_size_before_mb"], data[0]["original_triangles"]

    fig = plt.figure(figsize=(14, 4.8), dpi=130)
    fig.patch.set_facecolor(_hex(BG))
    fig.subplots_adjust(left=0.06, right=0.98, bottom=0.15, top=0.80, wspace=0.28)
    title_kw = dict(color=_hex(FG), fontsize=12.5, weight="bold", loc="left")

    ax = fig.add_subplot(1, 3, 1)
    _style(ax)
    ax.plot(tri, hmax, "o-", color=_hex(ORANGE), lw=2, label="Hausdorff max")
    ax.plot(tri, hrms, "o-", color=_hex(BLUE), lw=2, label="Hausdorff RMS")
    ax.set_xscale("log")
    ax.set_ylim(0, max(hmax) * 1.35)
    ax.set_xlabel("target triangles (log)")
    ax.set_ylabel("deviation [mesh units]")
    ax.set_title("Shape error stays flat", **title_kw)
    leg = ax.legend(facecolor=_hex(BG), edgecolor=_hex(BORDER), fontsize=9.5)
    for txt in leg.get_texts():
        txt.set_color(_hex(FG))

    ax = fig.add_subplot(1, 3, 2)
    _style(ax)
    labels = [f"{n // 1000}k" if n >= 1000 else str(n) for n in tri]
    colors = [_hex(MUTED)] + [_hex(GREEN)] * len(size)
    ax.bar(["source"] + labels, [src_size] + list(size), color=colors)
    ax.set_yscale("log")
    ax.set_ylabel("file size [MB] (log)")
    ax.set_xlabel("target triangles")
    ax.set_title(f"{src_size:.1f} MB source → KB-scale assets", **title_kw)

    ax = fig.add_subplot(1, 3, 3)
    _style(ax)
    ax.bar(labels, t, color=_hex(BLUE))
    ax.set_ylabel("wall time [s]")
    ax.set_xlabel("target triangles")
    ax.set_ylim(0, max(t) * 1.3)
    ax.set_title("Seconds, not minutes", **title_kw)

    fig.text(0.06, 0.92, f"QEM decimation of a {src_tri:,}-triangle mesh", color=_hex(FG),
             fontsize=15, weight="bold")  # fmt: skip
    fig.text(0.06, 0.875, "data: benchmarks/performance_results.json "
             "(python benchmarks/generate_report.py)", color=_hex(MUTED), fontsize=10,
             family="monospace")  # fmt: skip
    out = IMG / "benchmark_tradeoff.png"
    fig.savefig(out, facecolor=_hex(BG))
    plt.close(fig)
    return out


def main() -> None:
    IMG.mkdir(parents=True, exist_ok=True)
    parts = process_parts()
    RESULTS_JSON.write_text(
        json.dumps({k: p["stats"] for k, p in parts.items()}, indent=2, ensure_ascii=False) + "\n"
    )
    outs = [
        make_hero(parts["servo_motor"]),
        make_gallery(parts),
        make_heatmap(parts),
        make_benchmark_plot(),
    ]
    for out in outs:
        print(f"[figures] saved -> {out.relative_to(ROOT)}")
    print(f"[figures] saved -> {RESULTS_JSON.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
