"""
docs/figures/render.py - Headless studio renderer for the README figures.

pyrender over EGL (works on a GPU-less CI box with Mesa's llvmpipe), with a
three-point light rig, crease-angle smooth shading for "CAD" renders, and a
faceted + wireframe mode that makes a decimated collision mesh's triangles
visible. Images are supersampled 2x and downsampled for clean edges.
"""

from __future__ import annotations

import os

os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

import numpy as np  # noqa: E402
import pyrender  # noqa: E402
import trimesh  # noqa: E402
from PIL import Image  # noqa: E402

SS = 2  # supersampling factor

MATERIALS = {
    "steel": dict(
        baseColorFactor=(0.70, 0.72, 0.75, 1.0), metallicFactor=0.35, roughnessFactor=0.38
    ),
    "aluminium": dict(
        baseColorFactor=(0.80, 0.82, 0.85, 1.0), metallicFactor=0.25, roughnessFactor=0.45
    ),
    "cast_iron": dict(
        baseColorFactor=(0.33, 0.40, 0.48, 1.0), metallicFactor=0.15, roughnessFactor=0.6
    ),
    "motor": dict(
        baseColorFactor=(0.16, 0.17, 0.19, 1.0), metallicFactor=0.2, roughnessFactor=0.5
    ),
    "brass": dict(
        baseColorFactor=(0.80, 0.62, 0.32, 1.0), metallicFactor=0.35, roughnessFactor=0.4
    ),
    "collision": dict(
        baseColorFactor=(0.97, 0.51, 0.40, 1.0), metallicFactor=0.0, roughnessFactor=0.7
    ),
    "visual": dict(
        baseColorFactor=(0.25, 0.73, 0.31, 1.0), metallicFactor=0.1, roughnessFactor=0.55
    ),
}


def _look_at(eye, target, up=(0.0, 0.0, 1.0)):
    eye, target, up = map(np.asarray, (eye, target, up))
    f = target - eye
    f = f / np.linalg.norm(f)
    s = np.cross(f, up)
    s = s / np.linalg.norm(s)
    u = np.cross(s, f)
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = s, u, -f, eye
    return m


class Camera:
    """A camera framed on a reference mesh, shared by all renders of one part."""

    def __init__(
        self, ref: trimesh.Trimesh, direction=(1.0, -1.25, 0.8), yfov_deg=28.0, fill=0.86
    ):
        self.yfov = np.deg2rad(yfov_deg)
        center = ref.bounding_box.centroid
        radius = np.linalg.norm(ref.extents) / 2
        d = np.asarray(direction, float)
        d = d / np.linalg.norm(d)
        dist = radius / np.sin(self.yfov / 2) * fill
        self.eye = center + d * dist
        self.pose = _look_at(self.eye, center)
        self.center = center
        self.radius = radius


def _lights(scene, cam: Camera):
    # Key (upper left-front), fill (right), rim (behind), all relative to the camera.
    R = cam.pose[:3, :3]
    for direction, intensity in (
        ((-0.6, 0.7, 1.0), 3.6),  # key
        ((0.9, -0.1, 0.6), 1.4),  # fill
        ((0.1, 0.6, -1.0), 2.2),  # rim (from behind the part)
        ((0.0, -1.0, 0.2), 0.8),  # bounce from below
    ):
        d = R @ (np.asarray(direction, float) / np.linalg.norm(direction))
        pose = _look_at(cam.center + d * cam.radius * 4, cam.center)
        scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=intensity), pose=pose)


def _wire(mesh: trimesh.Trimesh, cam: Camera, rgba=(0.12, 0.05, 0.03, 1.0)):
    edges = mesh.vertices[mesh.edges_unique].reshape(-1, 3)
    # Nudge lines toward the camera so they never z-fight with the faces.
    to_cam = cam.eye - edges
    to_cam /= np.linalg.norm(to_cam, axis=1, keepdims=True)
    edges = edges + to_cam * cam.radius * 0.004
    prim = pyrender.Primitive(
        positions=edges.astype(np.float32),
        mode=pyrender.constants.GLTF.LINES,
        color_0=np.tile(np.asarray(rgba, np.float32), (len(edges), 1)),
        material=pyrender.MetallicRoughnessMaterial(baseColorFactor=rgba, emissiveFactor=rgba[:3]),
    )
    return pyrender.Mesh([prim])


def render(
    mesh: trimesh.Trimesh,
    cam: Camera,
    size=(640, 520),
    material="steel",
    mode="smooth",
    face_colors: np.ndarray | None = None,
) -> Image.Image:
    """
    mode = "smooth"  crease-angle smooth shading (looks like a CAD render)
           "facets"  flat shading + wireframe, exposes every triangle
           "colors"  flat shading with per-face RGBA colours (heatmaps)
    Returns an RGBA image with a transparent background.
    """
    w, h = size
    scene = pyrender.Scene(bg_color=(0, 0, 0, 0), ambient_light=(0.16, 0.16, 0.18))
    mat = pyrender.MetallicRoughnessMaterial(**MATERIALS[material])
    if mode == "smooth":
        m = mesh.smooth_shaded
        scene.add(pyrender.Mesh.from_trimesh(m, material=mat, smooth=True))
    elif mode == "facets":
        scene.add(pyrender.Mesh.from_trimesh(mesh, material=mat, smooth=False))
        scene.add(_wire(mesh, cam))
    elif mode == "colors":
        m = mesh.copy()
        m.visual = trimesh.visual.ColorVisuals(m, face_colors=face_colors)
        scene.add(pyrender.Mesh.from_trimesh(m, smooth=False))
    else:
        raise ValueError(mode)

    scene.add(
        pyrender.PerspectiveCamera(yfov=cam.yfov, aspectRatio=w / h, znear=cam.radius * 0.05),
        pose=cam.pose,
    )
    _lights(scene, cam)
    r = pyrender.OffscreenRenderer(w * SS, h * SS)
    try:
        color, _ = r.render(scene, flags=pyrender.RenderFlags.RGBA)
    finally:
        r.delete()
    return Image.fromarray(color, "RGBA").resize((w, h), Image.LANCZOS)
