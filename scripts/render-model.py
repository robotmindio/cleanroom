#!/usr/bin/env python3
"""Render the generated zero-pose calibration reference (requires matplotlib and numpy)."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def main():
    robot = ET.fromstring(subprocess.check_output(
        ["xacro", str(ROOT / "urdf/lekiwi.urdf.xacro"), "sim:=false"], text=True
    ))

    def origin(element):
        matrix = np.eye(4)
        if element is not None:
            r, p, y = map(float, element.get("rpy", "0 0 0").split())
            cr, cp, cy = np.cos([r, p, y])
            sr, sp, sy = np.sin([r, p, y])
            matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                             [-sp, cp*sr, cp*cr]]
            matrix[:3, 3] = list(map(float, element.get("xyz", "0 0 0").split()))
        return matrix

    parents = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}

    def placement(frame):
        result = np.eye(4)
        while frame != "base_link":
            joint = parents[frame]
            result = origin(joint.find("origin")) @ result
            frame = joint.find("parent").get("link")
        return result

    fig = plt.figure(figsize=(15, 6))
    axes = [fig.add_subplot(1, 3, index + 1, projection="3d") for index in range(3)]
    all_vertices = []
    for link in robot.findall("link"):
        name = link.get("name")
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            mesh = geometry.find("mesh")
            if mesh is not None:
                path = ROOT / mesh.get("filename").removeprefix("package://lekiwi_rmf/")
                data = path.read_bytes()
                count = int.from_bytes(data[80:84], "little")
                if len(data) != 84 + count * 50:
                    raise ValueError(f"expected binary STL: {path}")
                triangles = np.frombuffer(data, offset=84, dtype=np.dtype([
                    ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")
                ]))["vertices"].astype(float)
                triangles *= list(map(float, mesh.get("scale", "1 1 1").split()))
            elif geometry.find("box") is not None:
                vertices = np.array([[-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
                                     [-1,-1,1], [1,-1,1], [1,1,1], [-1,1,1]], dtype=float)
                vertices *= np.array(list(map(float, geometry.find("box").get("size").split()))) / 2
                triangles = vertices[[[0,1,2], [0,2,3], [4,6,5], [4,7,6], [0,4,5], [0,5,1],
                                      [1,5,6], [1,6,2], [2,6,7], [2,7,3], [3,7,4], [3,4,0]]]
            else:
                raise ValueError(f"unsupported visual geometry on {name}")
            transform = placement(name) @ origin(visual.find("origin"))
            triangles = triangles @ transform[:3, :3].T + transform[:3, 3]
            all_vertices.extend(triangles.reshape(-1, 3))
            color = "#ddd8c9"
            if "Servo" in name or "STS" in name or "Wheel" in name:
                color = "#444b50"
            if "lidar_mount" in name:
                color = "#70b655"
            if name == "ld06_body" or name == "astra_camera_link":
                color = "#263b48"
            if name.startswith("so101_"):
                material = visual.find("material")
                color = "#ded9c9" if material is not None and material.get("name") == "3d_printed" else "#3f4548"
            for axis in axes:
                axis.add_collection3d(Poly3DCollection(triangles, facecolor=color, edgecolor="none"))
    points = np.asarray(all_vertices)
    centre = (points.min(axis=0) + points.max(axis=0)) / 2
    radius = max(np.ptp(points, axis=0)) / 2 + 0.025
    for axis, title, view in zip(axes, ["Oblique", "Side (+X forward)", "Front"], [(25, -55), (0, -90), (0, 0)]):
        axis.set(xlim=(centre[0]-radius, centre[0]+radius), ylim=(centre[1]-radius, centre[1]+radius),
                 zlim=(centre[2]-radius, centre[2]+radius), xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title=title)
        axis.set_box_aspect((1, 1, 1))
        axis.view_init(*view)
    fig.suptitle("SO-101 reference: all six arm joints = 0 rad\nGenerated geometry only — no hardware commands")
    fig.tight_layout()
    output = ROOT / "urdf/so101-zero-pose.png"
    fig.savefig(output, dpi=160)
    print(output)


if __name__ == "__main__":
    main()
