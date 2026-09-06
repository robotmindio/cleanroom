#!/usr/bin/env python3
"""Vendor the complete generated LeKiwi Xacro as cleanroom's physical model."""

from __future__ import annotations

import argparse
import io
import json
from pathlib import Path, PurePosixPath
import subprocess
import sys
import xml.etree.ElementTree as ET

from lekiwi_rmf.arm_trajectory import JOINT_LIMITS, JOINT_VELOCITY_LIMITS


SOURCE_MODEL = Path("URDF/LeKiwi.urdf.xacro")
MESH_PREFIX = "${mesh_dir}/"
PACKAGE_PREFIX = "package://lekiwi_rmf/urdf/meshes/"
WHEEL_JOINTS = {"base_left_wheel", "base_back_wheel", "base_right_wheel"}
ARM_EFFORT_LIMITS = {
    "arm_shoulder_pan": "4",
    "arm_shoulder_lift": "4",
    "arm_elbow_flex": "4",
    "arm_wrist_flex": "2",
    "arm_wrist_roll": "2",
    "arm_gripper": "1",
}


def validate_xacro(path: Path) -> None:
    result = subprocess.run(
        ["xacro", str(path.resolve())], capture_output=True, text=True
    )
    if result.returncode:
        raise ValueError(f"invalid LeKiwi source Xacro: {result.stderr.strip()}")


def transform(path: Path) -> tuple[ET.Element, dict[Path, Path]]:
    validate_xacro(path)
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
        if name in JOINT_LIMITS:
            joint.set(
                "type", "${roll_joint}" if name == "arm_wrist_roll" else "${arm_joint}"
            )
            limit = joint.find("limit")
            if (
                limit is None
                or "lower" not in limit.attrib
                or "upper" not in limit.attrib
            ):
                raise ValueError(f"{name}: source joint needs lower and upper limits")
            expected = JOINT_LIMITS[name]
            actual = tuple(float(limit.get(key)) for key in ("lower", "upper"))
            if actual != expected:
                raise ValueError(
                    f"{name}: source limits {actual} do not match configured limits {expected}"
                )
            limit.attrib = {
                "lower": limit.get("lower"),
                "upper": limit.get("upper"),
                "effort": ARM_EFFORT_LIMITS[name],
                "velocity": f"{JOINT_VELOCITY_LIMITS[name]:g}",
            }
    return root, meshes


def source_revision(source: Path) -> str:
    subprocess.run(
        [sys.executable, str(source / "scripts/model_manifest.py"), "--check"],
        cwd=source, check=True,
    )
    status = subprocess.run(
        [
            "git",
            "-C",
            str(source),
            "status",
            "--porcelain",
            "--untracked-files=no",
            "--",
            str(SOURCE_MODEL),
            "URDF/meshes",
            "cad",
            "scripts",
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
    parser.add_argument("--check", action="store_true", help="fail if the vendored model or meshes differ; write nothing")
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    root, meshes = transform(source / SOURCE_MODEL)
    revision = source_revision(source)

    ET.indent(root, space="    ")
    generated = io.BytesIO()
    ET.ElementTree(root).write(
        generated, encoding="utf-8", xml_declaration=True
    )
    mesh_dir = output / "meshes"
    # Read every input before changing the snapshot. A missing upstream mesh
    # must not leave a new URDF pointing at an incomplete set of old assets.
    files = {output / "lekiwi_cad.urdf": generated.getvalue()}
    files[output / "model-source.json"] = (json.dumps({
        "repository": "https://github.com/robotmindio/LeKiwi",
        "revision": revision,
        "model": SOURCE_MODEL.as_posix(),
    }, indent=2) + "\n").encode()
    for target, model_source in meshes.items():
        files[mesh_dir / target] = (source / "URDF/meshes" / model_source).read_bytes()
    if args.check:
        different = [str(path.relative_to(output)) for path, content in files.items()
                     if not path.is_file() or path.read_bytes() != content]
        different.extend(str(path.relative_to(output)) for path in mesh_dir.rglob("*.stl")
                         if path not in files)
        if different:
            raise SystemExit("stale vendored model: " + ", ".join(different))
        print(f"vendored model matches LeKiwi {revision}")
        return 0

    for destination, content in files.items():
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
    for stale in mesh_dir.rglob("*.stl"):
        if stale not in files:
            stale.unlink()
    print(
        f"vendored LeKiwi {revision}: {len(root.findall('link'))} links, {len(meshes)} meshes"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
