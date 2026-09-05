"""Check the deterministic physical-model adaptations."""

import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location(
    "vendor_lekiwi_model", ROOT / "scripts" / "vendor-lekiwi-model.py"
)
VENDOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VENDOR)


def test_transform_replaces_existing_limits_without_duplicate_elements(tmp_path):
    source = tmp_path / "model.xacro"
    source.write_text(
        """<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="mesh_dir" value="meshes"/>
  <link name="arm"><visual><geometry><mesh filename="${mesh_dir}/so101/arm.stl"/></geometry></visual><collision/></link>
  <joint name="arm_wrist_roll" type="revolute"><limit lower="0" upper="0"/></joint>
</robot>"""
    )

    root, meshes = VENDOR.transform(source)

    assert root.find("link/collision") is None
    assert (
        root.find(".//mesh").get("filename") == VENDOR.PACKAGE_PREFIX + "so101/arm.stl"
    )
    assert meshes == {Path("so101/arm.stl"): Path("so101/arm.stl")}
    joint = root.find("joint")
    assert joint.get("type") == "${roll_joint}"
    assert len(joint.findall("limit")) == 1
    assert joint.find("limit").get("lower") == VENDOR.ARM_LIMITS["arm_wrist_roll"][0]


def test_vendored_so101_mount_keeps_the_installed_plate_pose():
    root = ET.parse(ROOT / "urdf" / "lekiwi_cad.urdf").getroot()
    mount = root.find("joint[@name='so101_mount']")

    assert mount.find("parent").get("link") == "base_plate_layer2-v3"
    assert mount.find("child").get("link") == "so101_base_link"
    # The official base origin is 38.8353 mm from its shoulder axis. Reusing
    # the legacy base-part origin displaced that axis by almost 79 mm.
    origin = mount.find("origin")
    assert list(map(float, origin.get("xyz").split())) == pytest.approx(
        [0, 0.02831271, 0.007], abs=1e-7
    )
    assert list(map(float, origin.get("rpy").split())) == pytest.approx(
        [0, 0, 1.5707963267948966], abs=7e-6
    )


def test_vendor_preflights_meshes_and_checks_snapshot_without_writing(tmp_path, monkeypatch):
    source, output = tmp_path / "source", tmp_path / "output"
    mesh = source / "URDF/meshes/reauthored/body.stl"
    mesh.parent.mkdir(parents=True)
    (source / VENDOR.SOURCE_MODEL).write_text(
        '<robot name="test"><link name="body"><visual><geometry>'
        '<mesh filename="${mesh_dir}/reauthored/body.stl"/>'
        '</geometry></visual></link></robot>'
    )
    monkeypatch.setattr(VENDOR, "source_revision", lambda _: "test-revision")
    args = ["vendor", "--source", str(source), "--output", str(output)]
    monkeypatch.setattr(sys, "argv", args)
    with pytest.raises(FileNotFoundError):
        VENDOR.main()
    assert not output.exists()
    mesh.write_bytes(b"test mesh")
    assert VENDOR.main() == 0
    monkeypatch.setattr(sys, "argv", args + ["--check"])
    assert VENDOR.main() == 0
    vendored_mesh = output / "meshes/body.stl"
    vendored_mesh.write_bytes(b"stale mesh")
    with pytest.raises(SystemExit, match="stale vendored model: meshes/body.stl"):
        VENDOR.main()
    assert vendored_mesh.read_bytes() == b"stale mesh"
