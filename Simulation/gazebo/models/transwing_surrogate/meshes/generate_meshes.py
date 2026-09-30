#!/usr/bin/env python3
"""
Generate smooth COLLADA (.dae) meshes for the P4 Transwing drone model.

Uses pure Python + numpy – no external mesh libraries required.
All meshes are centred at the origin and sized to match the existing
SDF primitive dimensions so that the existing <pose> tags keep working.

Run:  py generate_meshes.py
"""

import numpy as np
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

# ── helpers ──────────────────────────────────────────────────────────

def _naca0012(x):
    """Symmetric NACA 0012 half-thickness at normalised chord position x."""
    return 0.12 / 0.20 * (
        0.2969 * np.sqrt(x)
        - 0.1260 * x
        - 0.3516 * x**2
        + 0.2843 * x**3
        - 0.1015 * x**4
    )


def _naca_symmetric(x, t_ratio):
    """Symmetric NACA 00XX with custom thickness ratio *t_ratio*."""
    return t_ratio / 0.12 * _naca0012(x)


def _write_collada(filename, vertices, normals, triangles):
    """Write a COLLADA 1.4.1 .dae file from arrays.

    Parameters
    ----------
    vertices : (N, 3) float
    normals  : (N, 3) float  – per-vertex normals
    triangles : (M, 3) int   – index triplets
    """
    nv = len(vertices)
    nt = len(triangles)

    # --- Build XML tree ------------------------------------------------
    COLLADA = "http://www.collada.org/2005/11/COLLADASchema"
    root = ET.Element("COLLADA", xmlns=COLLADA, version="1.4.1")

    asset = ET.SubElement(root, "asset")
    up = ET.SubElement(asset, "up_axis")
    up.text = "Z_UP"

    lib_geom = ET.SubElement(root, "library_geometries")
    geom = ET.SubElement(lib_geom, "geometry", id="mesh0", name="mesh0")
    mesh = ET.SubElement(geom, "mesh")

    # positions
    pos_src = ET.SubElement(mesh, "source", id="mesh0-positions")
    pos_arr = ET.SubElement(pos_src, "float_array",
                            id="mesh0-positions-array",
                            count=str(nv * 3))
    pos_arr.text = " ".join(f"{v:.8f}" for v in vertices.ravel())
    tech = ET.SubElement(pos_src, "technique_common")
    acc = ET.SubElement(tech, "accessor",
                        source="#mesh0-positions-array",
                        count=str(nv), stride="3")
    for p in "XYZ":
        ET.SubElement(acc, "param", name=p, type="float")

    # normals
    nrm_src = ET.SubElement(mesh, "source", id="mesh0-normals")
    nrm_arr = ET.SubElement(nrm_src, "float_array",
                            id="mesh0-normals-array",
                            count=str(nv * 3))
    nrm_arr.text = " ".join(f"{n:.8f}" for n in normals.ravel())
    tech2 = ET.SubElement(nrm_src, "technique_common")
    acc2 = ET.SubElement(tech2, "accessor",
                         source="#mesh0-normals-array",
                         count=str(nv), stride="3")
    for p in "XYZ":
        ET.SubElement(acc2, "param", name=p, type="float")

    # vertices
    verts = ET.SubElement(mesh, "vertices", id="mesh0-vertices")
    ET.SubElement(verts, "input", semantic="POSITION",
                  source="#mesh0-positions")

    # triangles
    tris = ET.SubElement(mesh, "triangles", count=str(nt))
    ET.SubElement(tris, "input", semantic="VERTEX",
                  source="#mesh0-vertices", offset="0")
    ET.SubElement(tris, "input", semantic="NORMAL",
                  source="#mesh0-normals", offset="1")
    p_elem = ET.SubElement(tris, "p")
    idx_pairs = []
    for tri in triangles:
        for vi in tri:
            idx_pairs.append(f"{vi} {vi}")
    p_elem.text = " ".join(idx_pairs)

    # scene
    lib_vis = ET.SubElement(root, "library_visual_scenes")
    vis_scene = ET.SubElement(lib_vis, "visual_scene", id="Scene", name="Scene")
    node = ET.SubElement(vis_scene, "node", id="Mesh", name="Mesh", type="NODE")
    inst = ET.SubElement(node, "instance_geometry", url="#mesh0")

    scene = ET.SubElement(root, "scene")
    ET.SubElement(scene, "instance_visual_scene", url="#Scene")

    # Pretty-print
    rough = ET.tostring(root, encoding="unicode")
    dom = minidom.parseString(rough)
    pretty = dom.toprettyxml(indent="  ", encoding="utf-8")

    with open(filename, "wb") as f:
        f.write(pretty)
    print(f"  [OK] {filename}  ({nv} verts, {nt} tris)")


def compute_normals(verts, tris):
    """Compute smooth per-vertex normals by averaging face normals."""
    normals = np.zeros_like(verts)
    v0 = verts[tris[:, 0]]
    v1 = verts[tris[:, 1]]
    v2 = verts[tris[:, 2]]
    face_normals = np.cross(v1 - v0, v2 - v0)
    # Normalise face normals
    norms = np.linalg.norm(face_normals, axis=1, keepdims=True)
    norms[norms < 1e-12] = 1.0
    face_normals /= norms
    for i in range(3):
        np.add.at(normals, tris[:, i], face_normals)
    n = np.linalg.norm(normals, axis=1, keepdims=True)
    n[n < 1e-12] = 1.0
    normals /= n
    return normals


# ── Mesh generators ─────────────────────────────────────────────────

def make_fuselage(length=1.55, radius=0.105, nose_frac=0.15, tail_frac=0.25,
                  n_circ=48, n_len=80):
    """Streamlined fuselage – body of revolution with nose & tail taper.

    Origin at geometric centre (matches the cylinder visual's origin).
    Axis along +X (Gazebo FLU forward).
    """
    half = length / 2.0
    # Parametric stations along X
    xs = np.linspace(-half, half, n_len)

    def _radius(x):
        """Smooth radius profile."""
        # Normalise to [0, 1]
        t = (x + half) / length  # 0 at tail, 1 at nose
        nose_start = 1.0 - nose_frac
        tail_end = tail_frac
        if t > nose_start:
            # Nose: elliptical taper
            s = (t - nose_start) / nose_frac
            return radius * np.sqrt(max(1.0 - s * s, 0.0))
        elif t < tail_end:
            # Tail: smooth taper (cosine)
            s = t / tail_end
            return radius * (0.35 + 0.65 * (0.5 - 0.5 * np.cos(np.pi * s)))
        else:
            return radius

    thetas = np.linspace(0, 2 * np.pi, n_circ, endpoint=False)

    verts = []
    for x in xs:
        r = _radius(x)
        for th in thetas:
            verts.append([x, r * np.cos(th), r * np.sin(th)])
    # Add nose tip and tail tip
    nose_tip = [half + radius * 0.15, 0.0, 0.0]  # Slightly beyond
    tail_tip = [-half - radius * 0.08, 0.0, 0.0]
    verts.append(nose_tip)
    verts.append(tail_tip)
    verts = np.array(verts)

    nose_idx = len(verts) - 2
    tail_idx = len(verts) - 1

    tris = []
    for i in range(n_len - 1):
        for j in range(n_circ):
            j1 = (j + 1) % n_circ
            a = i * n_circ + j
            b = i * n_circ + j1
            c = (i + 1) * n_circ + j
            d = (i + 1) * n_circ + j1
            tris.append([a, c, b])
            tris.append([b, c, d])

    # Nose cap
    last_ring = (n_len - 1) * n_circ
    for j in range(n_circ):
        j1 = (j + 1) % n_circ
        tris.append([last_ring + j, last_ring + j1, nose_idx])

    # Tail cap
    for j in range(n_circ):
        j1 = (j + 1) % n_circ
        tris.append([j1, j, tail_idx])

    tris = np.array(tris)
    tris = tris[:, [0, 2, 1]]  # Fix winding: outward-facing normals
    normals = compute_normals(verts, tris)
    return verts, normals, tris


def make_tapered_cylinder(length=0.45, r_front=0.052, r_back=0.030,
                          n_circ=32, n_len=20):
    """Tail boom – tapered cylinder along +X, centred at origin."""
    half = length / 2.0
    xs = np.linspace(-half, half, n_len)
    thetas = np.linspace(0, 2 * np.pi, n_circ, endpoint=False)

    verts = []
    for x in xs:
        t = (x + half) / length  # 0 at back, 1 at front
        r = r_back + (r_front - r_back) * t
        for th in thetas:
            verts.append([x, r * np.cos(th), r * np.sin(th)])

    # Caps
    verts.append([half, 0, 0])   # front cap centre
    verts.append([-half, 0, 0])  # back cap centre
    verts = np.array(verts)
    front_cap = len(verts) - 2
    back_cap = len(verts) - 1

    tris = []
    for i in range(n_len - 1):
        for j in range(n_circ):
            j1 = (j + 1) % n_circ
            a = i * n_circ + j
            b = i * n_circ + j1
            c = (i + 1) * n_circ + j
            d = (i + 1) * n_circ + j1
            tris.append([a, c, b])
            tris.append([b, c, d])

    last_ring = (n_len - 1) * n_circ
    for j in range(n_circ):
        j1 = (j + 1) % n_circ
        tris.append([last_ring + j, last_ring + j1, front_cap])
    for j in range(n_circ):
        j1 = (j + 1) % n_circ
        tris.append([j1, j, back_cap])

    tris = np.array(tris)
    tris = tris[:, [0, 2, 1]]  # Fix winding: outward-facing normals
    normals = compute_normals(verts, tris)
    return verts, normals, tris


def make_airfoil_wing(chord, span, thickness_ratio, tip_chord_ratio=0.65,
                      n_chord=60, n_span=40, round_tip=True, anchor_te=False):
    """Wing with NACA-style airfoil cross-section.

    Origin at centre. Chord along X, span along Y, thickness along Z.
    Leading edge at +X side, trailing edge at -X side.

    Parameters
    ----------
    chord : float – root chord length
    span : float – total span
    thickness_ratio : float – max thickness / chord (e.g. 0.12)
    tip_chord_ratio : float – tip chord / root chord
    anchor_te : bool – if True, trailing edge stays at fixed X = -chord/2
                       and only the leading edge sweeps back with taper.
                       Use True when a control surface hinge must stay
                       flush with the wing trailing edge.
    """
    half_span = span / 2.0

    # Chord-wise stations with cosine spacing for detail at LE & TE
    beta = np.linspace(0, np.pi, n_chord)
    x_norm = 0.5 * (1.0 - np.cos(beta))  # 0→1

    # Span-wise stations
    y_stations = np.linspace(-half_span, half_span, n_span)

    # Fixed trailing-edge X position (used when anchor_te=True)
    te_x = -0.5 * chord

    verts = []
    # For each span station, create upper and lower airfoil profiles
    for y in y_stations:
        # Taper: chord varies linearly
        t_span = 1.0 - abs(y) / half_span  # 1 at root, 0 at tip
        local_chord = chord * (tip_chord_ratio + (1.0 - tip_chord_ratio) * t_span)

        # Tip rounding: reduce thickness near tips smoothly
        tip_factor = 1.0
        if round_tip:
            edge = 0.85  # Start rounding at 85% span
            if abs(y) / half_span > edge:
                s = (abs(y) / half_span - edge) / (1.0 - edge)
                tip_factor = np.sqrt(max(1.0 - s * s, 0.0))

        for xn in x_norm:
            yt = _naca_symmetric(xn, thickness_ratio) * local_chord * tip_factor
            if anchor_te:
                x_pos = te_x + (1.0 - xn) * local_chord
            else:
                x_pos = (0.5 - xn) * local_chord  # LE at +x
            # Upper surface
            verts.append([x_pos, y, yt])

        for xn in x_norm:
            yt = _naca_symmetric(xn, thickness_ratio) * local_chord * tip_factor
            if anchor_te:
                x_pos = te_x + (1.0 - xn) * local_chord
            else:
                x_pos = (0.5 - xn) * local_chord
            # Lower surface
            verts.append([x_pos, y, -yt])

    verts = np.array(verts)
    n_profile = 2 * n_chord  # upper + lower per span station

    tris = []
    for i in range(n_span - 1):
        for j in range(n_chord - 1):
            # Upper surface quads
            a = i * n_profile + j
            b = i * n_profile + j + 1
            c = (i + 1) * n_profile + j
            d = (i + 1) * n_profile + j + 1
            tris.append([a, b, c])
            tris.append([b, d, c])

            # Lower surface quads
            lo = n_chord
            a2 = i * n_profile + lo + j
            b2 = i * n_profile + lo + j + 1
            c2 = (i + 1) * n_profile + lo + j
            d2 = (i + 1) * n_profile + lo + j + 1
            tris.append([a2, c2, b2])
            tris.append([b2, c2, d2])

        # Connect upper TE to lower TE (trailing edge strip)
        j_te_u = n_chord - 1  # last upper
        j_te_l = n_chord + n_chord - 1  # last lower
        a = i * n_profile + j_te_u
        b = (i + 1) * n_profile + j_te_u
        c = i * n_profile + j_te_l
        d = (i + 1) * n_profile + j_te_l
        tris.append([a, b, c])
        tris.append([c, b, d])

        # Connect upper LE to lower LE (leading edge strip)
        j_le_u = 0
        j_le_l = n_chord
        a = i * n_profile + j_le_u
        b = (i + 1) * n_profile + j_le_u
        c = i * n_profile + j_le_l
        d = (i + 1) * n_profile + j_le_l
        tris.append([a, c, b])
        tris.append([b, c, d])

    # Tip caps (flat for now, at y = ±half_span)
    for end_idx in [0, n_span - 1]:
        base = end_idx * n_profile
        # Simple fan from first upper vertex
        center_upper = base
        center_lower = base + n_chord
        # Close upper surface tip
        for j in range(1, n_chord - 1):
            if end_idx == 0:
                tris.append([center_upper, base + j + 1, base + j])
            else:
                tris.append([center_upper, base + j, base + j + 1])
        # Close lower surface tip
        for j in range(1, n_chord - 1):
            if end_idx == 0:
                tris.append([center_lower, center_lower + j, center_lower + j + 1])
            else:
                tris.append([center_lower, center_lower + j + 1, center_lower + j])

    tris = np.array(tris)
    tris = tris[:, [0, 2, 1]]  # Fix winding: outward-facing normals
    normals = compute_normals(verts, tris)
    return verts, normals, tris


def make_control_surface(chord, span, thickness,
                         n_chord=30, n_span=30):
    """Thin control surface (aileron/elevator/rudder) with rounded edges.

    Origin at the hinge line (leading edge of the surface).
    Chord along -X (trailing direction), span along Y, thickness along Z.
    """
    # Wedge shape: thicker at hinge, thinner at trailing edge
    beta = np.linspace(0, np.pi, n_chord)
    x_norm = 0.5 * (1.0 - np.cos(beta))  # 0→1

    half_span = span / 2.0
    y_stations = np.linspace(-half_span, half_span, n_span)

    verts = []
    for y in y_stations:
        # Tip rounding
        tip_factor = 1.0
        edge = 0.88
        if abs(y) / half_span > edge:
            s = (abs(y) / half_span - edge) / (1.0 - edge)
            tip_factor = np.sqrt(max(1.0 - s * s, 0.0))

        for xn in x_norm:
            # Thickness distribution: elliptical taper from LE to TE
            t = thickness * 0.5 * np.sqrt(max(1.0 - xn * xn, 0.0)) * tip_factor
            x_pos = -xn * chord  # LE at x=0, TE at x=-chord
            verts.append([x_pos, y, t])  # upper

        for xn in x_norm:
            t = thickness * 0.5 * np.sqrt(max(1.0 - xn * xn, 0.0)) * tip_factor
            x_pos = -xn * chord
            verts.append([x_pos, y, -t])  # lower

    verts = np.array(verts)
    n_profile = 2 * n_chord

    tris = []
    for i in range(n_span - 1):
        for j in range(n_chord - 1):
            # Upper
            a = i * n_profile + j
            b = i * n_profile + j + 1
            c = (i + 1) * n_profile + j
            d = (i + 1) * n_profile + j + 1
            tris.append([a, b, c])
            tris.append([b, d, c])
            # Lower
            lo = n_chord
            a2 = i * n_profile + lo + j
            b2 = i * n_profile + lo + j + 1
            c2 = (i + 1) * n_profile + lo + j
            d2 = (i + 1) * n_profile + lo + j + 1
            tris.append([a2, c2, b2])
            tris.append([b2, c2, d2])

        # TE seam
        j_te_u = n_chord - 1
        j_te_l = n_chord + n_chord - 1
        a = i * n_profile + j_te_u
        b = (i + 1) * n_profile + j_te_u
        c = i * n_profile + j_te_l
        d = (i + 1) * n_profile + j_te_l
        tris.append([a, b, c])
        tris.append([c, b, d])

        # LE seam
        a = i * n_profile
        b = (i + 1) * n_profile
        c = i * n_profile + n_chord
        d = (i + 1) * n_profile + n_chord
        tris.append([a, c, b])
        tris.append([b, c, d])

    # Tip caps
    for end_idx in [0, n_span - 1]:
        base = end_idx * n_profile
        center_upper = base
        center_lower = base + n_chord
        for j in range(1, n_chord - 1):
            if end_idx == 0:
                tris.append([center_upper, base + j + 1, base + j])
            else:
                tris.append([center_upper, base + j, base + j + 1])
        for j in range(1, n_chord - 1):
            if end_idx == 0:
                tris.append([center_lower, center_lower + j, center_lower + j + 1])
            else:
                tris.append([center_lower, center_lower + j + 1, center_lower + j])

    tris = np.array(tris)
    tris = tris[:, [0, 2, 1]]  # Fix winding: outward-facing normals
    normals = compute_normals(verts, tris)
    return verts, normals, tris


def make_vertical_surface(chord, height, thickness, n_chord=30, n_height=30,
                          tip_chord_ratio=1.0, anchor_te=False):
    """Vertical tail – like a wing but span is along Z instead of Y.

    Origin at centre. Chord along X, height along Z, thickness along Y.
    Root at bottom (z = -height/2), tip at top (z = +height/2).

    Parameters
    ----------
    tip_chord_ratio : float – tip chord / root chord (1.0 = no taper)
    anchor_te : bool – if True, TE stays at fixed X = -chord/2
                       and only LE sweeps back at the tip.
    """
    half_h = height / 2.0
    beta = np.linspace(0, np.pi, n_chord)
    x_norm = 0.5 * (1.0 - np.cos(beta))

    z_stations = np.linspace(-half_h, half_h, n_height)

    # Fixed TE position (used when anchor_te=True)
    te_x = -0.5 * chord

    verts = []
    for z in z_stations:
        # Taper: root at bottom, tip at top
        t_frac = (z + half_h) / height  # 0 at bottom (root), 1 at top (tip)
        local_chord = chord * (1.0 - (1.0 - tip_chord_ratio) * t_frac)

        # Tip rounding at the TOP only (tip)
        tip_factor = 1.0
        top_edge = 0.85
        if t_frac > top_edge:
            s = (t_frac - top_edge) / (1.0 - top_edge)
            tip_factor = np.sqrt(max(1.0 - s * s, 0.0))

        # Normalise NACA thickness to peak value for scaling
        naca_peak = _naca_symmetric(0.3, 0.10)

        for xn in x_norm:
            t = thickness * 0.5 * _naca_symmetric(xn, 0.10) / naca_peak * tip_factor
            if anchor_te:
                x_pos = te_x + (1.0 - xn) * local_chord
            else:
                x_pos = (0.5 - xn) * local_chord
            verts.append([x_pos, t, z])  # +Y side

        for xn in x_norm:
            t = thickness * 0.5 * _naca_symmetric(xn, 0.10) / naca_peak * tip_factor
            if anchor_te:
                x_pos = te_x + (1.0 - xn) * local_chord
            else:
                x_pos = (0.5 - xn) * local_chord
            verts.append([x_pos, -t, z])  # -Y side

    verts = np.array(verts)
    n_profile = 2 * n_chord

    tris = []
    for i in range(n_height - 1):
        for j in range(n_chord - 1):
            a = i * n_profile + j
            b = i * n_profile + j + 1
            c = (i + 1) * n_profile + j
            d = (i + 1) * n_profile + j + 1
            tris.append([a, b, c])
            tris.append([b, d, c])

            lo = n_chord
            a2 = i * n_profile + lo + j
            b2 = i * n_profile + lo + j + 1
            c2 = (i + 1) * n_profile + lo + j
            d2 = (i + 1) * n_profile + lo + j + 1
            tris.append([a2, c2, b2])
            tris.append([b2, c2, d2])

        j_te_u = n_chord - 1
        j_te_l = n_chord + n_chord - 1
        a = i * n_profile + j_te_u
        b = (i + 1) * n_profile + j_te_u
        c = i * n_profile + j_te_l
        d = (i + 1) * n_profile + j_te_l
        tris.append([a, b, c])
        tris.append([c, b, d])

        a = i * n_profile
        b = (i + 1) * n_profile
        c = i * n_profile + n_chord
        d = (i + 1) * n_profile + n_chord
        tris.append([a, c, b])
        tris.append([b, c, d])

    for end_idx in [0, n_height - 1]:
        base = end_idx * n_profile
        center_upper = base
        center_lower = base + n_chord
        for j in range(1, n_chord - 1):
            if end_idx == 0:
                tris.append([center_upper, base + j + 1, base + j])
            else:
                tris.append([center_upper, base + j, base + j + 1])
        for j in range(1, n_chord - 1):
            if end_idx == 0:
                tris.append([center_lower, center_lower + j, center_lower + j + 1])
            else:
                tris.append([center_lower, center_lower + j + 1, center_lower + j])

    tris = np.array(tris)
    normals = compute_normals(verts, tris)
    return verts, normals, tris


def make_rudder_ctrl(chord, height, thickness, n_chord=30, n_height=30):
    """Rudder control surface – thin panel in XZ plane, thickness in Y.

    Origin at hinge line (leading edge). Chord along -X, height along Z.
    """
    half_h = height / 2.0
    beta = np.linspace(0, np.pi, n_chord)
    x_norm = 0.5 * (1.0 - np.cos(beta))

    z_stations = np.linspace(-half_h, half_h, n_height)

    verts = []
    for z in z_stations:
        tip_factor = 1.0
        edge = 0.85
        if abs(z) / half_h > edge:
            s = (abs(z) / half_h - edge) / (1.0 - edge)
            tip_factor = np.sqrt(max(1.0 - s * s, 0.0))

        for xn in x_norm:
            t = thickness * 0.5 * np.sqrt(max(1.0 - xn * xn, 0.0)) * tip_factor
            x_pos = -xn * chord
            verts.append([x_pos, t, z])

        for xn in x_norm:
            t = thickness * 0.5 * np.sqrt(max(1.0 - xn * xn, 0.0)) * tip_factor
            x_pos = -xn * chord
            verts.append([x_pos, -t, z])

    verts = np.array(verts)
    n_profile = 2 * n_chord

    tris = []
    for i in range(n_height - 1):
        for j in range(n_chord - 1):
            a = i * n_profile + j
            b = i * n_profile + j + 1
            c = (i + 1) * n_profile + j
            d = (i + 1) * n_profile + j + 1
            tris.append([a, b, c])
            tris.append([b, d, c])

            lo = n_chord
            a2 = i * n_profile + lo + j
            b2 = i * n_profile + lo + j + 1
            c2 = (i + 1) * n_profile + lo + j
            d2 = (i + 1) * n_profile + lo + j + 1
            tris.append([a2, c2, b2])
            tris.append([b2, c2, d2])

        j_te_u = n_chord - 1
        j_te_l = n_chord + n_chord - 1
        a = i * n_profile + j_te_u
        b = (i + 1) * n_profile + j_te_u
        c = i * n_profile + j_te_l
        d = (i + 1) * n_profile + j_te_l
        tris.append([a, b, c])
        tris.append([c, b, d])

        a = i * n_profile
        b = (i + 1) * n_profile
        c = i * n_profile + n_chord
        d = (i + 1) * n_profile + n_chord
        tris.append([a, c, b])
        tris.append([b, c, d])

    for end_idx in [0, n_height - 1]:
        base = end_idx * n_profile
        center_upper = base
        center_lower = base + n_chord
        for j in range(1, n_chord - 1):
            if end_idx == 0:
                tris.append([center_upper, base + j + 1, base + j])
            else:
                tris.append([center_upper, base + j, base + j + 1])
        for j in range(1, n_chord - 1):
            if end_idx == 0:
                tris.append([center_lower, center_lower + j, center_lower + j + 1])
            else:
                tris.append([center_lower, center_lower + j + 1, center_lower + j])

    tris = np.array(tris)
    normals = compute_normals(verts, tris)
    return verts, normals, tris


# ── Main ─────────────────────────────────────────────────────────────

def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    print(f"Output directory: {out_dir}\n")

    # 1. Fuselage  (cylinder was 1.55 long, r=0.105, centred at origin)
    #    The SDF has fuselage_body (cylinder) + nose (sphere) as separate visuals.
    #    We replace BOTH with a single fuselage mesh, centred at origin.
    #    The nose sphere was at x=0.775 with r=0.105 – our mesh extends to ~0.79.
    print("Generating fuselage...")
    v, n, t = make_fuselage(length=1.55, radius=0.105, nose_frac=0.18,
                            tail_frac=0.20, n_circ=48, n_len=80)
    _write_collada(os.path.join(out_dir, "fuselage.dae"), v, n, t)

    # 2. Tail boom  (box was 0.45 × 0.09 × 0.09, centred at (-0.91, 0, 0))
    #    We keep the same centre & size, just make it a tapered cylinder.
    print("Generating tail boom...")
    v, n, t = make_tapered_cylinder(length=0.45, r_front=0.052, r_back=0.032,
                                    n_circ=24, n_len=16)
    _write_collada(os.path.join(out_dir, "tail_boom.dae"), v, n, t)

    # 3. Main wing  (box was 0.30 × 0.86 × 0.04, centred at (0, ±0.43, 0))
    #    Chord=0.30, span=0.86, thickness ~0.04 → ratio ≈ 0.133
    print("Generating wing...")
    v, n, t = make_airfoil_wing(chord=0.30, span=0.86, thickness_ratio=0.12,
                                tip_chord_ratio=0.70,
                                n_chord=50, n_span=40, anchor_te=True)
    _write_collada(os.path.join(out_dir, "wing.dae"), v, n, t)

    # 4. Inner wing root  (box was 0.31 × 0.44 × 0.045, centred at (0.03, 0, 0.03))
    #    Chord=0.31, span=0.44, thickness ~0.045 → ratio ≈ 0.145
    print("Generating inner wing...")
    v, n, t = make_airfoil_wing(chord=0.31, span=0.44, thickness_ratio=0.14,
                                tip_chord_ratio=0.95,
                                n_chord=40, n_span=24, round_tip=False,
                                anchor_te=True)
    _write_collada(os.path.join(out_dir, "inner_wing.dae"), v, n, t)

    # 5. Horizontal tail  (box was 0.27 x 0.62 x 0.025)
    #    Swept back LE like real P4 – aggressive taper with fixed TE
    print("Generating horizontal tail...")
    v, n, t = make_airfoil_wing(chord=0.27, span=0.62, thickness_ratio=0.09,
                                tip_chord_ratio=0.35,
                                n_chord=40, n_span=30, anchor_te=True)
    _write_collada(os.path.join(out_dir, "htail.dae"), v, n, t)

    # 6. Vertical tail  (box was 0.30 x 0.025 x 0.27)
    #    Swept back LE like real P4 – wide root, narrow tip
    print("Generating vertical tail...")
    v, n, t = make_vertical_surface(chord=0.30, height=0.27, thickness=0.025,
                                    n_chord=36, n_height=28,
                                    tip_chord_ratio=0.40, anchor_te=True)
    _write_collada(os.path.join(out_dir, "vtail.dae"), v, n, t)

    # 7. Aileron  (box was 0.08 × 0.50 × 0.025, centred at (-0.04, 0, 0))
    #    Chord=0.08, span=0.50, thickness=0.025
    #    Hinge at x=0, surface extends to x=-0.08
    print("Generating aileron...")
    v, n, t = make_control_surface(chord=0.08, span=0.50, thickness=0.025,
                                   n_chord=20, n_span=28)
    _write_collada(os.path.join(out_dir, "aileron.dae"), v, n, t)

    # 8. Elevator  (box was 0.09 × 0.56 × 0.022, centred at (-0.045, 0, 0))
    #    Chord=0.09, span=0.56, thickness=0.022
    print("Generating elevator...")
    v, n, t = make_control_surface(chord=0.09, span=0.56, thickness=0.022,
                                   n_chord=20, n_span=28)
    _write_collada(os.path.join(out_dir, "elevator.dae"), v, n, t)

    # 9. Rudder  (box was 0.09 × 0.022 × 0.19, centred at (-0.045, 0, 0.04))
    #    Chord=0.09 (X), height=0.19 (Z), thickness=0.022 (Y)
    #    Hinge at x=0, surface extends to x=-0.09
    print("Generating rudder...")
    v, n, t = make_rudder_ctrl(chord=0.09, height=0.19, thickness=0.022,
                               n_chord=20, n_height=24)
    _write_collada(os.path.join(out_dir, "rudder.dae"), v, n, t)

    print(f"\n[DONE] All meshes generated in {out_dir}")


if __name__ == "__main__":
    main()
