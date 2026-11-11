"""
docs/figures/parts.py - Parametric industrial components for the README figures.

Every part is built with real CSG (trimesh + manifold3d booleans) from
catalogue-style dimensions, then tessellated finely, the way a CAD package's
"high quality" STL export would be. Units are metres, like a URDF.

    servo_motor()        NEMA 34-style servo: finned body, pilot flange, keyed shaft,
                         encoder cap and power connector
    spur_gear()          involute spur gear, module 3, 36 teeth, hub, keyway, lightening holes
    weld_neck_flange()   DN100 PN16-style weld-neck flange, raised face, 8 bolt holes
    tslot_extrusion()    40x40 B-type aluminium T-slot profile, 160 mm long
    pillow_block()       UCP-style pillow block bearing with slotted base and grease nipple
    timing_pulley()      HTD-5M style timing pulley with flanges, hub and set-screw bore
"""

from __future__ import annotations

import numpy as np
import shapely.affinity as sa
import trimesh
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

MM = 1e-3
SECTIONS = 192  # facets per full circle for round features


# ---------------------------------------------------------------------------
# Small CSG helpers (all dimensions in mm, converted to metres at the end)
# ---------------------------------------------------------------------------


def _T(x=0.0, y=0.0, z=0.0):
    return trimesh.transformations.translation_matrix((x, y, z))


def _R(axis, deg):
    v = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
    return trimesh.transformations.rotation_matrix(np.deg2rad(deg), v)


def _cyl(r, h, z0=0.0, x=0.0, y=0.0, sections=SECTIONS):
    """Cylinder along +Z from z0 to z0+h."""
    return trimesh.creation.cylinder(
        radius=r, height=h, sections=sections, transform=_T(x, y, z0 + h / 2)
    )


def _box(sx, sy, sz, cx=0.0, cy=0.0, cz=0.0):
    return trimesh.creation.box((sx, sy, sz), transform=_T(cx, cy, cz))


def _extrude(poly, h, z0=0.0):
    return trimesh.creation.extrude_polygon(poly, h, transform=_T(z=z0))


def _revolve(profile_rz):
    """Revolve a closed (r, z) profile around Z."""
    pts = np.asarray(profile_rz, dtype=float)
    if not np.allclose(pts[0], pts[-1]):
        pts = np.vstack([pts, pts[:1]])  # revolve() needs an explicitly closed profile
    if Polygon(pts).exterior.is_ccw is False:
        pts = pts[::-1]
    m = trimesh.creation.revolve(pts, sections=SECTIONS)
    if m.volume < 0:  # profile wound clockwise
        m.invert()
    return m


def _union(*meshes):
    return trimesh.boolean.union(list(meshes), engine="manifold")


def _diff(a, *cutters):
    return trimesh.boolean.difference([a, *cutters], engine="manifold")


def _finish(mesh: trimesh.Trimesh, max_edge_mm: float) -> trimesh.Trimesh:
    """Fine 'CAD export' tessellation, then convert mm -> m."""
    v, f = trimesh.remesh.subdivide_to_size(mesh.vertices, mesh.faces, max_edge=max_edge_mm)
    out = trimesh.Trimesh(v * MM, f, process=True)
    out.merge_vertices()
    return out


def _circle(r, x=0.0, y=0.0):
    return Point(x, y).buffer(r, quad_segs=SECTIONS // 4)


# ---------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------


def servo_motor() -> trimesh.Trimesh:
    """NEMA 34-style AC servo, 86 mm frame, ~180 mm long including shaft."""
    body_len = 118.0
    # Square body with 45-degree chamfered long edges
    body2d = box(-43, -43, 43, 43).intersection(sa.rotate(box(-55, -55, 55, 55), 45))
    body = _extrude(body2d, body_len, z0=0.0)
    # Cooling fins: shallow grooves around the body
    grooves = []
    for i in range(9):
        z = 18 + i * 10.0
        ring = _extrude(box(-50, -50, 50, 50).difference(body2d.buffer(-2.2)), 4.0, z0=z)
        grooves.append(ring)
    body = _diff(body, *grooves)

    # Front mounting flange with 4 counterbored holes and pilot boss
    flange = _extrude(
        box(-50, -50, 50, 50).buffer(-6, join_style=1).buffer(6, join_style=1), 12, -12
    )
    holes = [_cyl(3.3, 20, -16, x, y) for x in (-39.6, 39.6) for y in (-39.6, 39.6)]
    cbores = [_cyl(5.5, 5, -12.01, x, y) for x in (-39.6, 39.6) for y in (-39.6, 39.6)]
    flange = _diff(flange, *holes, *cbores)
    pilot = _cyl(36.5, 4.0, -16.0)

    # Keyed output shaft
    shaft = _cyl(9.5, 40.0, -56.0)
    key = _box(6.0, 3.0, 26.0, 0, 9.5, -40.0)
    shaft = _union(_diff(shaft, _box(6.0, 6.0, 28.0, 0, 9.5, -40.0)), key)

    # Rear encoder cap and power / feedback connectors
    cap = _extrude(body2d.buffer(-4), 26.0, z0=body_len)
    cap = _diff(cap, _cyl(26.0, 3.0, body_len + 23.5))
    conn1 = _union(
        _box(26, 22, 18, 0, 43 + 9, body_len - 16),
        trimesh.creation.cylinder(
            radius=9,
            height=22,
            sections=96,
            transform=_T(0, 43 + 18 + 11, body_len - 16) @ _R("x", 90),
        ),
    )
    conn2 = trimesh.creation.cylinder(
        radius=7.5, height=16, sections=96, transform=_T(0, 43 + 8, body_len - 50) @ _R("x", 90)
    )
    motor = _union(body, flange, pilot, shaft, cap, conn1, conn2)
    # Lay it horizontally: shaft along +X, connectors on top (+Z).
    motor.apply_transform(_R("z", -18) @ _R("x", 90) @ _R("y", -90))
    return _finish(motor, 3.6)


def _involute_gear_2d(module, teeth, pressure_deg=20.0, n=14):
    pa = np.deg2rad(pressure_deg)
    rp = module * teeth / 2
    rb = rp * np.cos(pa)
    ra = rp + module
    rf = rp - 1.25 * module

    def inv(a):
        return np.tan(a) - a

    # half tooth angle at the pitch circle
    half = np.pi / (2 * teeth) + inv(pa)
    r = np.linspace(max(rb, rf), ra, n)
    phi = np.arccos(np.clip(rb / r, -1, 1))
    theta = half - inv(phi)
    pts = []
    for k in range(teeth):
        c = 2 * np.pi * k / teeth
        right = [(ri * np.cos(c - t), ri * np.sin(c - t)) for ri, t in zip(r, theta)]
        left = [(ri * np.cos(c + t), ri * np.sin(c + t)) for ri, t in zip(r[::-1], theta[::-1])]
        # root arc to next tooth
        a0 = c + theta[0]
        a1 = c + 2 * np.pi / teeth - theta[0]
        root = [(rf * np.cos(a), rf * np.sin(a)) for a in np.linspace(a0, a1, 8)]
        pts += [(rf * np.cos(c - theta[0]), rf * np.sin(c - theta[0]))] if k == 0 else []
        pts += right + left + root
    return Polygon(pts).buffer(0).simplify(1e-3)


def spur_gear() -> trimesh.Trimesh:
    """Module 3, 36 teeth (PCD 108 mm), 20 mm face, with hub, keyway and lightening holes."""
    outline = _involute_gear_2d(3.0, 36)
    web_cut = [
        _circle(9.5, 31 * np.cos(a), 31 * np.sin(a))
        for a in np.linspace(0, 2 * np.pi, 6, endpoint=False)
    ]
    gear2d = outline.difference(unary_union(web_cut))
    gear = _extrude(gear2d, 20.0)
    # Recess the web on both faces, leaving the rim and hub full width
    recess = _circle(43).difference(_circle(20))
    gear = _diff(gear, _extrude(recess, 5.0, 15.01), _extrude(recess, 5.0, -0.01))
    hub = _cyl(20.0, 34.0, -7.0)
    bore2d = _circle(12.5).union(box(-4, 0, 4, 15.8))  # 25 mm bore + 8 mm DIN 6885 keyway
    gear = _diff(_union(gear, hub), _extrude(bore2d, 40, -10))
    # Set-screw hole through the hub
    gear = _diff(
        gear,
        trimesh.creation.cylinder(
            radius=2.5, height=30, sections=64, transform=_T(0, -15, -1) @ _R("x", 90)
        ),
    )
    gear.apply_transform(_R("x", -90))
    return _finish(gear, 2.2)


def weld_neck_flange() -> trimesh.Trimesh:
    """DN100 PN16-style weld neck flange: OD 220, 8x M16 on PCD 180, raised face."""
    profile = [
        (57.5, 0.0),
        (110.0, 0.0),
        (110.0, 20.0),
        (78.0, 20.0),
        (66.0, 45.0),
        (60.3, 62.0),
        (57.5, 62.0),
    ]
    flange = _revolve(profile)
    raised = _revolve([(57.5, -3.0), (79.0, -3.0), (79.0, 0.5), (57.5, 0.5)])
    flange = _union(flange, raised)
    bolts = [
        _cyl(9.0, 40, -10, 90 * np.cos(a), 90 * np.sin(a), sections=96)
        for a in np.linspace(0, 2 * np.pi, 8, endpoint=False) + np.pi / 8
    ]
    # Concentric serrations on the raised face (gasket finish)
    serr = [
        _revolve([(r, -3.3), (r + 0.8, -3.3), (r + 0.8, -2.6), (r, -2.6)])
        for r in np.arange(60, 78, 2.0)
    ]
    flange = _diff(flange, *bolts, *serr)
    flange.apply_transform(_R("z", 25) @ _R("x", 90))  # neck toward the viewer
    return _finish(flange, 3.0)


def tslot_extrusion() -> trimesh.Trimesh:
    """40x40 B-type (slot 8) aluminium profile, 160 mm long."""
    sq = box(-20, -20, 20, 20).buffer(-1.2, join_style=1).buffer(1.2, join_style=1)
    cuts = []
    for ang in (0, 90, 180, 270):
        opening = box(20 - 4.3, -4.1, 21, 4.1)
        cavity = Polygon([(20 - 4.3, -8.2), (20 - 11.5, -4.0), (20 - 11.5, 4.0), (20 - 4.3, 8.2)])
        cuts.append(sa.rotate(opening.union(cavity), ang, origin=(0, 0)))
    core = _circle(3.4)
    # Corner cavities that make the real profile lightweight
    for sx in (-1, 1):
        for sy in (-1, 1):
            cuts.append(
                Polygon(
                    [(sx * 10.0, sy * 13.2), (sx * 13.2, sy * 13.2), (sx * 13.2, sy * 10.0)]
                ).buffer(1.2)
            )
    prof = sq.difference(unary_union(cuts + [core]))
    prof = prof.buffer(0.3, join_style=1).buffer(-0.3, join_style=1)
    ext = _extrude(prof, 160.0)
    ext.apply_transform(_R("y", 90) @ _T(z=-80))
    ext.apply_transform(_R("z", -32))  # show the slot profile end-on
    return _finish(ext, 9.0)


def pillow_block() -> trimesh.Trimesh:
    """UCP205-style pillow block: shaft 25 mm, centre height 36.5 mm, bolt slots 105 mm apart."""
    h = 36.5
    base2d = box(-63.5, -19, 63.5, 19).buffer(-3, join_style=1).buffer(3, join_style=1)
    base = _extrude(base2d, 14.0)
    slots = [
        _extrude(LineString([(x - 3, 0), (x + 3, 0)]).buffer(6.5), 30, -5) for x in (-52.5, 52.5)
    ]
    # Housing: horizontal ring + web to the base
    housing = trimesh.creation.cylinder(
        radius=33.0, height=34.0, sections=SECTIONS, transform=_T(0, 0, h) @ _R("x", 90)
    )
    web2d = Polygon([(-40, 0), (40, 0), (28, h), (-28, h)])
    web = _extrude(web2d, 28.0)
    web.apply_transform(_T(0, 14, 0) @ _R("x", 90))
    body = _union(base, housing, web)
    body = _diff(
        body,
        *slots,
        trimesh.creation.cylinder(
            radius=26.0, height=40, sections=SECTIONS, transform=_T(0, 0, h) @ _R("x", 90)
        ),
    )
    # Bearing outer ring visible inside the housing, then inner ring with set screws
    outer = trimesh.creation.annulus(
        r_min=21.0, r_max=26.0, height=17.0, sections=SECTIONS, transform=_T(0, 0, h) @ _R("x", 90)
    )
    inner = trimesh.creation.annulus(
        r_min=12.5, r_max=17.5, height=34.0, sections=SECTIONS, transform=_T(0, 0, h) @ _R("x", 90)
    )
    balls = [
        trimesh.creation.icosphere(subdivisions=2, radius=3.6).apply_translation(
            (19.3 * np.cos(a), 0, h + 19.3 * np.sin(a))
        )
        for a in np.linspace(0, 2 * np.pi, 10, endpoint=False)
    ]
    nipple = _union(
        _cyl(4.0, 8.0, h + 30.0, sections=6),
        trimesh.creation.icosphere(subdivisions=2, radius=3.2).apply_translation((0, 0, h + 40)),
    )
    part = _union(body, outer, inner, nipple, *balls)
    part.apply_transform(_R("z", -20))
    return _finish(part, 2.4)


def timing_pulley() -> trimesh.Trimesh:
    """HTD-5M style pulley, 40 teeth (PD 63.66 mm), 25 mm belt, flanges, hub with set screws."""
    teeth, pitch = 40, 5.0
    pd = teeth * pitch / np.pi
    od = pd - 1.144
    tooth_cuts = [
        _circle(1.75, (od / 2 + 0.25) * np.cos(a), (od / 2 + 0.25) * np.sin(a))
        for a in np.linspace(0, 2 * np.pi, teeth, endpoint=False)
    ]
    body2d = _circle(od / 2).difference(unary_union(tooth_cuts))
    body = _extrude(body2d, 27.0)
    fl = _revolve([(0, 0), (od / 2 + 4.5, 0), (od / 2 + 4.5, 0.8), (od / 2 + 1.0, 1.6), (0, 1.6)])
    fl_top = fl.copy().apply_transform(_T(z=27.0 + 1.6) @ _R("x", 180))
    fl.apply_transform(_T(z=-1.6))
    hub = _cyl(19.0, 16.0, 28.6)
    part = _union(body, fl, fl_top, hub)
    lighten = [
        _cyl(5.5, 40, -5, 18.5 * np.cos(a), 18.5 * np.sin(a), sections=96)
        for a in np.linspace(0, 2 * np.pi, 5, endpoint=False)
    ]
    bore2d = _circle(7.0).union(box(-2.5, 0, 2.5, 9.3))
    screws = [
        trimesh.creation.cylinder(
            radius=2.5,
            height=25,
            sections=64,
            transform=_T(0, 0, 37) @ _R("z", a) @ _R("x", 90) @ _T(z=12),
        )
        for a in (0, 90)
    ]
    part = _diff(part, *lighten, _extrude(bore2d, 60, -10), *screws)
    part.apply_transform(_R("x", -90))
    return _finish(part, 3.0)


CATALOG = {
    "servo_motor": ("AC servo motor", "NEMA 34 frame, finned body, keyed shaft", servo_motor),
    "spur_gear": ("Involute spur gear", "m3 · z36 · hub, keyway, lightening holes", spur_gear),
    "weld_neck_flange": (
        "Weld-neck flange",
        "DN100 PN16 · raised face · 8 bolt holes",
        weld_neck_flange,
    ),
    "tslot_extrusion": ("T-slot extrusion", "40×40 B-type · slot 8 · 160 mm", tslot_extrusion),
    "pillow_block": (
        "Pillow block bearing",
        "UCP205 style · Ø25 shaft · slotted base",
        pillow_block,
    ),
    "timing_pulley": (
        "Timing pulley",
        "HTD-5M · 40 teeth · flanged, set-screw hub",
        timing_pulley,
    ),
}
