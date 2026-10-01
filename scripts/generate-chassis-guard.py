#!/usr/bin/env python3
"""Generate the convex, D-shaped chassis collision/visual mesh."""

import math
import struct
from pathlib import Path


RADIUS = 0.145
# The CAD arm pedestal's center-facing edge is x=-0.00298 m in base_link.
CUT_X = -0.003
HEIGHT = 0.050
SEGMENTS = 48
OUTPUT = Path(__file__).resolve().parents[1] / "urdf/chassis_guard.stl"


def triangle(a, b, c):
    u = [b[i] - a[i] for i in range(3)]
    v = [c[i] - a[i] for i in range(3)]
    normal = (
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    )
    length = math.sqrt(sum(component * component for component in normal))
    return struct.pack("<12fH", *(component / length for component in normal), *a, *b, *c, 0)


def main():
    angle = math.acos(CUT_X / RADIUS)
    # Counterclockwise arc from the top of the cut around the rear to its bottom.
    outline = [
        (RADIUS * math.cos(angle + (2 * math.pi - 2 * angle) * i / SEGMENTS),
         RADIUS * math.sin(angle + (2 * math.pi - 2 * angle) * i / SEGMENTS))
        for i in range(SEGMENTS + 1)
    ]
    low = [(x, y, -HEIGHT / 2) for x, y in outline]
    high = [(x, y, HEIGHT / 2) for x, y in outline]
    faces = []
    for i in range(1, len(outline) - 1):
        faces.extend((triangle(high[0], high[i], high[i + 1]),
                      triangle(low[0], low[i + 1], low[i])))
    for i in range(len(outline)):
        j = (i + 1) % len(outline)
        faces.extend((triangle(low[i], low[j], high[j]),
                      triangle(low[i], high[j], high[i])))
    OUTPUT.write_bytes(b"LeKiwi chassis guard".ljust(80, b"\0")
                       + struct.pack("<I", len(faces)) + b"".join(faces))


if __name__ == "__main__":
    main()
