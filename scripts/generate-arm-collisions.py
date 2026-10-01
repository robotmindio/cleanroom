#!/usr/bin/env python3
"""Generate conservative per-part collision hulls without changing vendored CAD."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull


ROOT = Path(__file__).resolve().parents[1]
LINKS = ("so101_shoulder_link", "so101_lower_arm_link", "so101_wrist_link")
# Enclose every occupied 1 mm grid cell, rather than rounding vertices inward.
CELL = 0.001
STL = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])


def main():
    robot = ET.parse(ROOT / "urdf/lekiwi_cad.urdf").getroot()
    output = ROOT / "urdf/collision"
    output.mkdir(exist_ok=True)
    corners = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)])
    for name in LINKS:
        for index, visual in enumerate(robot.find(f"link[@name='{name}']").findall("visual")):
            mesh = visual.find("geometry/mesh")
            path = ROOT / mesh.get("filename").removeprefix("package://lekiwi_rmf/")
            data = path.read_bytes()
            count = int.from_bytes(data[80:84], "little")
            if len(data) != 84 + count * STL.itemsize:
                raise ValueError(f"invalid binary STL: {path}")
            points = np.frombuffer(data, offset=84, dtype=STL)["vertices"].reshape(-1, 3).astype(float)
            points *= np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            cells = np.unique(np.floor(points / CELL).astype(np.int64), axis=0)
            hull = ConvexHull(np.unique((cells[:, None, :] + corners).reshape(-1, 3), axis=0) * CELL)
            triangles = hull.points[hull.simplices].copy()
            normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
            reverse = (normals * hull.equations[:, :3]).sum(axis=1) < 0
            triangles[reverse] = triangles[reverse][:, [0, 2, 1]]
            records = np.zeros(len(triangles), dtype=STL)
            records["vertices"] = triangles
            records["normal"] = hull.equations[:, :3]
            target = output / f"{name}-{index}.stl"
            target.write_bytes(b"CAD occupied-cell hull, 1 mm".ljust(80, b"\0")
                               + len(records).to_bytes(4, "little") + records.tobytes())
            print(f"{target.relative_to(ROOT)}: {len(records)} triangles")


if __name__ == "__main__":
    main()
