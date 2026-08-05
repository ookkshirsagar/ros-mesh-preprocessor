"""
docs/generate_before_after.py - Regenerate docs/img/before_after.png.

Decimates a torus mesh from 7,200 to 250 triangles using the same
simplify_mesh() the CLI calls, and renders both as a side-by-side PNG.
Requires the 'viz' extra: pip install -e ".[viz]"

Usage:
    python docs/generate_before_after.py
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image

from optimizer.core import clean_mesh, simplify_mesh

OUT_PATH = Path(__file__).parent / "img" / "before_after.png"


def _render(mesh: o3d.geometry.TriangleMesh, title: str, color: str) -> Image.Image:
    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    fig = plt.figure(figsize=(5, 5), dpi=110)
    fig.patch.set_facecolor("white")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("white")
    ax.add_collection3d(
        Poly3DCollection(
            verts[tris],
            facecolor=color,
            edgecolor="black",
            linewidths=0.15 if len(tris) > 2000 else 0.4,
        )
    )
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_zlim(-1.0, 1.0)
    ax.set_box_aspect([1, 1, 0.7])
    ax.axis("off")
    ax.view_init(elev=28, azim=35)
    ax.set_title(title, fontsize=13, fontweight="bold", color="#222")
    plt.tight_layout()

    tmp_path = OUT_PATH.parent / f".{title[:6]}.tmp.png"
    plt.savefig(tmp_path, facecolor="white")
    plt.close(fig)
    img = Image.open(tmp_path).convert("RGB")
    tmp_path.unlink()
    return img


def main() -> None:
    torus = o3d.geometry.TriangleMesh.create_torus(
        torus_radius=1.0, tube_radius=0.4, radial_resolution=60, tubular_resolution=60
    )
    torus.compute_triangle_normals()
    before_tris = len(torus.triangles)

    cleaned = clean_mesh(torus)
    after, _ = simplify_mesh(cleaned, target_triangles=250)
    after_tris = len(after.triangles)

    before_img = _render(torus, f"Before: {before_tris:,} triangles", "#4C8BF5")
    after_img = _render(after, f"After: {after_tris:,} triangles", "#F5734C")

    w, h = before_img.size
    combined = Image.new("RGB", (w * 2, h), (255, 255, 255))
    combined.paste(before_img, (0, 0))
    combined.paste(after_img, (w, 0))
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    combined.save(OUT_PATH, optimize=True)
    print(f"Saved → {OUT_PATH}")


if __name__ == "__main__":
    main()
