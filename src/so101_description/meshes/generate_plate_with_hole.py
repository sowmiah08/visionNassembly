#!/usr/bin/env python3
"""Procedurally generates base_plate_with_hole.stl: a rectangular plate with
a genuine, watertight cylindrical hole through it (not a visual-only marking).

URDF primitives (box/cylinder/sphere) cannot express a box with a hole, since
there is no boolean subtraction in URDF geometry. This script builds the
plate as an explicit triangle mesh by lofting between the plate's rectangular
outer boundary and the hole's circular inner boundary, for both the top and
bottom faces, plus the outer side walls and the inner (hole) cylindrical
wall. No external mesh libraries are required.

Local frame: origin at the center of the plate footprint, bottom face at
z=0, top face at z=PLATE_Z. Placing a joint's origin at a surface's (x, y, 0)
rests the plate directly on that surface.

Run directly to (re)generate base_plate_with_hole.stl in this directory:
    python3 generate_plate_with_hole.py
"""

import math
import struct
from pathlib import Path

PLATE_X = 0.10       # m, footprint length (local X)
PLATE_Y = 0.07        # m, footprint width (local Y)
PLATE_Z = 0.006       # m, thickness
HOLE_RADIUS = 0.012   # m, must fit within the footprint
SEGMENTS = 48         # circle resolution

OUTPUT_PATH = Path(__file__).resolve().parent / "base_plate_with_hole.stl"


def rect_point(theta, hx, hy):
    c, s = math.cos(theta), math.sin(theta)
    t = 1.0 / max(abs(c) / hx, abs(s) / hy)
    return (t * c, t * s)


def circle_point(theta, radius):
    return (radius * math.cos(theta), radius * math.sin(theta))


def build_triangles():
    hx, hy = PLATE_X / 2.0, PLATE_Y / 2.0
    assert HOLE_RADIUS < min(hx, hy), "hole must fit inside the plate footprint"

    triangles = []  # list of (normal_hint, (p0, p1, p2)) before orientation fix

    def add(p0, p1, p2, hint):
        triangles.append((hint, (p0, p1, p2)))

    for i in range(SEGMENTS):
        theta0 = 2.0 * math.pi * i / SEGMENTS
        theta1 = 2.0 * math.pi * (i + 1) / SEGMENTS

        rx0, ry0 = rect_point(theta0, hx, hy)
        rx1, ry1 = rect_point(theta1, hx, hy)
        cx0, cy0 = circle_point(theta0, HOLE_RADIUS)
        cx1, cy1 = circle_point(theta1, HOLE_RADIUS)

        r0b, r1b = (rx0, ry0, 0.0), (rx1, ry1, 0.0)
        r0t, r1t = (rx0, ry0, PLATE_Z), (rx1, ry1, PLATE_Z)
        c0b, c1b = (cx0, cy0, 0.0), (cx1, cy1, 0.0)
        c0t, c1t = (cx0, cy0, PLATE_Z), (cx1, cy1, PLATE_Z)

        # Top annulus (solid is between the hole and the rectangle), normal +z
        add(r0t, r1t, c1t, (0.0, 0.0, 1.0))
        add(r0t, c1t, c0t, (0.0, 0.0, 1.0))

        # Bottom annulus, normal -z
        add(r0b, r1b, c1b, (0.0, 0.0, -1.0))
        add(r0b, c1b, c0b, (0.0, 0.0, -1.0))

        # Outer side wall, normal points radially outward
        mx, my = (rx0 + rx1) / 2.0, (ry0 + ry1) / 2.0
        outward = (mx, my, 0.0)
        add(r0b, r1b, r1t, outward)
        add(r0b, r1t, r0t, outward)

        # Inner (hole) wall, normal points radially inward (into the void)
        mx_h, my_h = (cx0 + cx1) / 2.0, (cy0 + cy1) / 2.0
        inward = (-mx_h, -my_h, 0.0)
        add(c0b, c1b, c1t, inward)
        add(c0b, c1t, c0t, inward)

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
