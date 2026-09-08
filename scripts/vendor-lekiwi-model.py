#!/usr/bin/env python3
"""Vendor the complete generated LeKiwi Xacro as cleanroom's physical model."""

from __future__ import annotations

import argparse
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import xml.etree.ElementTree as ET


SOURCE_MODEL = Path("URDF/LeKiwi.urdf.xacro")
MESH_PREFIX = "${mesh_dir}/"
PACKAGE_PREFIX = "package://lekiwi_rmf/urdf/meshes/"
WHEEL_JOINTS = {"base_left_wheel", "base_back_wheel", "base_right_wheel"}
ARM_LIMITS = {
    "arm_shoulder_pan": ("-1.91986", "1.91986", "4", "2"),
    "arm_shoulder_lift": ("-1.74533", "1.74533", "4", "2"),
    "arm_elbow_flex": ("-1.69", "1.69", "4", "2"),
    "arm_wrist_flex": ("-1.65806", "1.65806", "2", "3"),
    "arm_wrist_roll": ("-2.74385", "2.84121", "2", "3"),
    "arm_gripper": ("-0.174533", "1.74533", "1", "2"),
}


def transform(path: Path) -> tuple[ET.Element, dict[Path, Path]]:
    root = ET.parse(path).getroot()
    for child in list(root):
        if child.tag.endswith("}property"):
            root.remove(child)
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            link.remove(collision)

    meshes = {}
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith(MESH_PREFIX):
            raise ValueError(f"unexpected LeKiwi mesh path: {filename!r}")
        source = PurePosixPath(filename.removeprefix(MESH_PREFIX))
        if source.is_absolute() or ".." in source.parts:
            raise ValueError(f"unsafe LeKiwi mesh path: {filename!r}")
        target = (
            PurePosixPath(*source.parts[1:])
            if source.parts[0] == "reauthored"
            else source
        )
        source_path, target_path = Path(*source.parts), Path(*target.parts)
        if target_path in meshes and meshes[target_path] != source_path:
            raise ValueError(f"two source meshes map to {target_path}")
        meshes[target_path] = source_path
        mesh.set("filename", PACKAGE_PREFIX + target.as_posix())

    for joint in root.findall("joint"):
        name = joint.get("name")
        if name in WHEEL_JOINTS:
            joint.set("type", "fixed")
        if name in ARM_LIMITS:
            joint.set(
                "type", "${roll_joint}" if name == "arm_wrist_roll" else "${arm_joint}"
            )
            lower, upper, effort, velocity = ARM_LIMITS[name]
            limit = joint.find("limit")
            if limit is None:
                limit = ET.SubElement(joint, "limit")
            limit.attrib = {
                "lower": lower,
                "upper": upper,
                "effort": effort,
                "velocity": velocity,
            }
    return root, meshes


def source_revision(source: Path) -> str:
    status = subprocess.run(
        [
            "git",
            "-C",
            str(source),
            "status",
            "--porcelain",
            "--",
            str(SOURCE_MODEL),
            "URDF/meshes",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    if status:
        raise ValueError("LeKiwi model inputs have local changes")
    return subprocess.run(
        ["git", "-C", str(source), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument(
        "--output", type=Path, default=Path(__file__).parents[1] / "urdf"
    )
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    root, meshes = transform(source / SOURCE_MODEL)
    revision = source_revision(source)

    output.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="    ")
    ET.ElementTree(root).write(
        output / "lekiwi_cad.urdf", encoding="utf-8", xml_declaration=True
    )
    mesh_dir = output / "meshes"
    expected = set()
    for target, model_source in meshes.items():
        destination = mesh_dir / target
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / "URDF/meshes" / model_source, destination)
        expected.add(destination)
    for stale in mesh_dir.rglob("*.stl"):
        if stale not in expected:
            stale.unlink()
    print(
        f"vendored LeKiwi {revision}: {len(root.findall('link'))} links, {len(meshes)} meshes"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
