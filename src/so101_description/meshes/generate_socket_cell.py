#!/usr/bin/env python3
"""Procedurally generates socket_cell_blind_hole.stl: one square "cell" of a
multi-pin socket, with a genuine cylindrical BLIND pocket (closed bottom, not
a through-hole) sized to receive one pin.

A 2x2 four-pin socket is built in the URDF by placing four copies of this
same cell mesh side by side (see the `socket` macro in
assembly_objects.urdf.xacro) -- since each cell is identical and the hole is
centered within its own cell, four untouched copies tile seamlessly into one
60x60mm block with four evenly spaced pockets, with no extra meshing work
needed for the multi-hole layout itself.

Like generate_plate_with_hole.py, this has no CSG library available, so the
blind pocket is built as explicit mesh regions:
  - a solid bottom face and plain box side walls (the pocket never reaches
    these, so they need no hole-aware lofting)
  - a top face that *is* lofted between the cell's rectangular boundary and
    the hole's circular boundary (reusing the same technique as the
    through-hole plate)
  - a pocket wall: the same inner-hole loft, but only descending from the
    top face down to the pocket floor instead of all the way to the bottom
  - a pocket floor: a flat fan-triangulated disk closing the bottom of the
    pocket, leaving CELL_Z - POCKET_DEPTH of solid material beneath it

Local frame: origin at the center of the cell's bottom face, so a joint
placed at (x, y, 0) on a surface rests the cell flush on it, with the pocket
opening upward.

Run directly to (re)generate socket_cell_blind_hole.stl in this directory:
    python3 generate_socket_cell.py
"""

import math
import struct
from pathlib import Path

CELL_X = 0.03          # m, one quadrant's footprint (full socket = 2*CELL_X by 2*CELL_Y)
CELL_Y = 0.03
CELL_Z = 0.025         # m, total block thickness
HOLE_RADIUS = 0.007    # m, pocket radius (pin radius 0.005 -> 2mm clearance)
POCKET_DEPTH = 0.022   # m, measured down from the top face
SEGMENTS = 48          # circle resolution

OUTPUT_PATH = Path(__file__).resolve().parent / "socket_cell_blind_hole.stl"


def rect_point(theta, hx, hy):
    c, s = math.cos(theta), math.sin(theta)
    t = 1.0 / max(abs(c) / hx, abs(s) / hy)
    return (t * c, t * s)


def circle_point(theta, radius):
    return (radius * math.cos(theta), radius * math.sin(theta))


def build_triangles():
    hx, hy = CELL_X / 2.0, CELL_Y / 2.0
    assert HOLE_RADIUS < min(hx, hy), "pocket must fit inside the cell footprint"
    assert 0.0 < POCKET_DEPTH < CELL_Z, "pocket depth must leave a solid floor"

    floor_z = CELL_Z - POCKET_DEPTH
    triangles = []

    def add(p0, p1, p2, hint):
        triangles.append((hint, (p0, p1, p2)))

    # --- Top face (annulus) + pocket wall + pocket floor + outer side walls ---
    # The rectangle boundary is sampled at N points (not just the 4 corners)
    # so the side walls' top edges exactly match the top annulus's outer
    # boundary edges segment-for-segment (required for the watertight check:
    # a single corner-to-corner wall edge would not match several smaller
    # collinear top-annulus edges covering the same span).
    for i in range(SEGMENTS):
        theta0 = 2.0 * math.pi * i / SEGMENTS
        theta1 = 2.0 * math.pi * (i + 1) / SEGMENTS

        rx0, ry0 = rect_point(theta0, hx, hy)
        rx1, ry1 = rect_point(theta1, hx, hy)
        cx0, cy0 = circle_point(theta0, HOLE_RADIUS)
        cx1, cy1 = circle_point(theta1, HOLE_RADIUS)

        r0b, r1b = (rx0, ry0, 0.0), (rx1, ry1, 0.0)
        r0t, r1t = (rx0, ry0, CELL_Z), (rx1, ry1, CELL_Z)
        c0t, c1t = (cx0, cy0, CELL_Z), (cx1, cy1, CELL_Z)
        c0f, c1f = (cx0, cy0, floor_z), (cx1, cy1, floor_z)

        # Top annulus (material between the pocket and the cell boundary), normal +z
        add(r0t, r1t, c1t, (0.0, 0.0, 1.0))
        add(r0t, c1t, c0t, (0.0, 0.0, 1.0))

        # Outer side wall, normal points radially outward
        mx, my = (rx0 + rx1) / 2.0, (ry0 + ry1) / 2.0
        outward = (mx, my, 0.0)
        add(r0b, r1b, r1t, outward)
        add(r0b, r1t, r0t, outward)

        # Pocket wall, normal points radially inward (into the cavity)
        mx_h, my_h = (cx0 + cx1) / 2.0, (cy0 + cy1) / 2.0
        inward = (-mx_h, -my_h, 0.0)
        add(c0t, c1t, c1f, inward)
        add(c0t, c1f, c0f, inward)

        # Pocket floor (fan from the hole's center), normal +z (up into the cavity)
        add((0.0, 0.0, floor_z), c0f, c1f, (0.0, 0.0, 1.0))

        # Solid bottom face (fan from the cell's center), normal -z
        add((0.0, 0.0, 0.0), r1b, r0b, (0.0, 0.0, -1.0))

    return triangles


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def normalize(a):
    n = math.sqrt(dot(a, a))
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def orient_triangles(raw_triangles):
    """Flip winding where needed so each triangle's normal matches its outward hint."""
    oriented = []
    for hint, (p0, p1, p2) in raw_triangles:
        n = cross(sub(p1, p0), sub(p2, p0))
        if dot(n, hint) < 0:
            p1, p2 = p2, p1
            n = cross(sub(p1, p0), sub(p2, p0))
        oriented.append((normalize(n), (p0, p1, p2)))
    return oriented


def assert_watertight(oriented_triangles):
    """Every undirected edge must be shared by exactly two triangles."""
    from collections import Counter

    def key(a, b):
        a = tuple(round(v, 9) for v in a)
        b = tuple(round(v, 9) for v in b)
        return (a, b) if a <= b else (b, a)

    edge_counts = Counter()
    for _, (p0, p1, p2) in oriented_triangles:
        edge_counts[key(p0, p1)] += 1
        edge_counts[key(p1, p2)] += 1
        edge_counts[key(p2, p0)] += 1

    bad = [e for e, c in edge_counts.items() if c != 2]
    assert not bad, f"mesh is not watertight: {len(bad)} edges not shared by exactly 2 triangles"


def write_binary_stl(oriented_triangles, path):
    with open(path, "wb") as f:
        f.write(b"\x00" * 80)
        f.write(struct.pack("<I", len(oriented_triangles)))
        for normal, (p0, p1, p2) in oriented_triangles:
            f.write(struct.pack("<3f", *normal))
            for p in (p0, p1, p2):
                f.write(struct.pack("<3f", *p))
            f.write(struct.pack("<H", 0))


def main():
    raw = build_triangles()
    oriented = orient_triangles(raw)
    assert_watertight(oriented)
    write_binary_stl(oriented, OUTPUT_PATH)
    print(f"wrote {len(oriented)} triangles to {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
