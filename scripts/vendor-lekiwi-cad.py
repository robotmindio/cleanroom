#!/usr/bin/env python3
"""Refresh or verify the deployable LeKiwi CAD snapshot from a checkout."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE_URDF = Path("URDF/LeKiwi.urdf")
SOURCE_MESHES = Path("URDF/meshes")
MESH_URI_PREFIX = "package://lekiwi_rmf/urdf/"
WHEEL_JOINTS = {"base_left_wheel", "base_back_wheel", "base_right_wheel"}
ARM_LIMITS = {
    "arm_shoulder_pan": {
        "lower": "-1.92",
        "upper": "1.92",
        "effort": "4",
        "velocity": "2",
    },
    "arm_shoulder_lift": {
        "lower": "-1.75",
        "upper": "1.75",
        "effort": "4",
        "velocity": "2",
    },
    "arm_elbow_flex": {
        "lower": "-1.75",
        "upper": "1.75",
        "effort": "4",
        "velocity": "2",
    },
    "arm_wrist_flex": {
        "lower": "-1.75",
        "upper": "1.75",
        "effort": "2",
        "velocity": "3",
    },
    "arm_wrist_roll": {
        "lower": "-3.14159265359",
        "upper": "3.14159265359",
        "effort": "2",
        "velocity": "3",
    },
    "arm_gripper": {"lower": "0", "upper": "1.57", "effort": "1", "velocity": "2"},
}


def transform_urdf(path: Path) -> ET.Element:
    """Apply only the ROS integration adaptations to an upstream CAD export."""
    root = ET.parse(path).getroot()
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            link.remove(collision)
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if not filename.startswith("meshes/"):
            raise ValueError(f"unexpected upstream mesh path: {filename!r}")
        mesh.attrib["filename"] = MESH_URI_PREFIX + filename
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        if name in WHEEL_JOINTS:
            joint.attrib["type"] = "fixed"
        if name in ARM_LIMITS:
            joint.attrib["type"] = (
                "${roll_joint}" if name == "arm_wrist_roll" else "${arm_joint}"
            )
            joint.append(ET.Element("limit", ARM_LIMITS[name]))
    return root


def mesh_names(root: ET.Element) -> tuple[Path, ...]:
    names = set()
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if not filename.startswith(MESH_URI_PREFIX + "meshes/"):
            raise ValueError(f"unexpected snapshot mesh path: {filename!r}")
        name = PurePosixPath(filename.removeprefix(MESH_URI_PREFIX + "meshes/"))
        if name.is_absolute() or not name.parts or ".." in name.parts:
            raise ValueError(f"unsafe mesh path: {filename!r}")
        names.add(Path(*name.parts))
    return tuple(sorted(names))


def semantic_digest(root: ET.Element) -> str:
    def canonical(element: ET.Element):
        return (
            element.tag,
            tuple(sorted(element.attrib.items())),
            tuple(canonical(child) for child in element),
        )

    return hashlib.sha256(repr(canonical(root)).encode()).hexdigest()


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def mesh_digest(mesh_dir: Path, names: tuple[Path, ...]) -> str:
    digest = hashlib.sha256()
    for name in names:
        path = mesh_dir / name
        if not path.is_file():
            raise FileNotFoundError(f"missing mesh: {path}")
        digest.update(name.as_posix().encode() + b"\0")
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def source_revision(source: Path) -> str:
    command = [
        "git",
        "-C",
        str(source),
        "status",
        "--porcelain",
        "--untracked-files=all",
        "--",
        str(SOURCE_URDF),
        str(SOURCE_MESHES),
    ]
    status = subprocess.run(command, text=True, capture_output=True, check=True).stdout
    if status:
        raise ValueError("LeKiwi CAD inputs have local changes; use a clean checkout")
    return subprocess.run(
        ["git", "-C", str(source), "rev-parse", "HEAD"],
        text=True,
        capture_output=True,
        check=True,
    ).stdout.strip()


def expected_snapshot(source: Path):
    source_urdf = source / SOURCE_URDF
    root = transform_urdf(source_urdf)
    names = mesh_names(root)
    source_meshes = source / SOURCE_MESHES
    meshes = mesh_digest(source_meshes, names)
    semantic = semantic_digest(root)
    manifest = {
        "schema_version": 1,
        "source": {
            "repository": "https://github.com/robotmindio/LeKiwi.git",
            "revision": source_revision(source),
            "urdf_sha256": file_digest(source_urdf),
        },
        "meshes": {"count": len(names), "sha256": meshes},
        "snapshot_sha256": hashlib.sha256(f"{semantic}\0{meshes}".encode()).hexdigest(),
    }
    return root, names, manifest


def rendered_xml(root: ET.Element) -> bytes:
    ET.indent(root, space="    ")
    return ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"


def write_snapshot(
    source: Path,
    output: Path,
    root: ET.Element,
    names: tuple[Path, ...],
    manifest: dict,
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    target_urdf = output / "lekiwi_cad.urdf"
    rendered = rendered_xml(root)
    if not target_urdf.is_file() or target_urdf.read_bytes() != rendered:
        target_urdf.write_bytes(rendered)
    for name in names:
        source_mesh = source / SOURCE_MESHES / name
        target_mesh = output / "meshes" / name
        if not target_mesh.is_file() or file_digest(target_mesh) != file_digest(
            source_mesh
        ):
            target_mesh.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source_mesh, target_mesh)
    (output / "lekiwi_cad.manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def check_snapshot(
    source: Path,
    output: Path,
    root: ET.Element,
    names: tuple[Path, ...],
    manifest: dict,
) -> list[str]:
    problems = []
    target_urdf = output / "lekiwi_cad.urdf"
    if not target_urdf.is_file():
        problems.append(f"missing {target_urdf}")
    elif semantic_digest(ET.parse(target_urdf).getroot()) != semantic_digest(root):
        problems.append(f"{target_urdf} is not the expected transformed CAD export")
    source_meshes, target_meshes = source / SOURCE_MESHES, output / "meshes"
    for name in names:
        target_mesh = target_meshes / name
        if not target_mesh.is_file() or file_digest(target_mesh) != file_digest(
            source_meshes / name
        ):
            problems.append(f"mesh differs: {target_mesh}")
    manifest_path = output / "lekiwi_cad.manifest.json"
    try:
        actual_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        problems.append(f"missing or invalid {manifest_path}")
    else:
        if actual_manifest != manifest:
            problems.append(f"{manifest_path} does not match the source checkout")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", type=Path, required=True, help="clean robotmindio/LeKiwi checkout"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "urdf",
        help="snapshot directory (default: %(default)s)",
    )
    parser.add_argument(
        "--write", action="store_true", help="refresh instead of checking"
    )
    args = parser.parse_args()
    try:
        root, names, manifest = expected_snapshot(args.source.resolve())
        if args.write:
            write_snapshot(
                args.source.resolve(), args.output.resolve(), root, names, manifest
            )
            print(f"refreshed {args.output.resolve()} from {args.source.resolve()}")
            return 0
        problems = check_snapshot(
            args.source.resolve(), args.output.resolve(), root, names, manifest
        )
    except (
        FileNotFoundError,
        subprocess.CalledProcessError,
        ValueError,
        ET.ParseError,
    ) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    if problems:
        print("snapshot is stale:", *problems, sep="\n  ", file=sys.stderr)
        print("run scripts/vendor-lekiwi-cad.py --source PATH --write", file=sys.stderr)
        return 1
    print("LeKiwi CAD snapshot is current")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
