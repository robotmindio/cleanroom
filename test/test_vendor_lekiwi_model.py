"""Check the deterministic physical-model adaptations."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET


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
    assert mount.find("origin").attrib == {"xyz": "0.04 0.08 0.007", "rpy": "0 0 0"}
