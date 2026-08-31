"""Check the deterministic transformations applied to vendored LeKiwi CAD."""

import importlib.util
import pathlib


ROOT = pathlib.Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location(
    "vendor_lekiwi_cad", ROOT / "scripts" / "vendor-lekiwi-cad.py"
)
VENDOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VENDOR)


def test_vendor_transform_removes_cad_collisions_and_applies_hardware_joints(tmp_path):
    source = tmp_path / "LeKiwi.urdf"
    source.write_text(
        """<robot name=\"LeKiwi\">
  <link name=\"base\"><visual><geometry><mesh filename=\"meshes/base.stl\"/></geometry></visual><collision/></link>
  <joint name=\"base_left_wheel\" type=\"continuous\"/>
  <joint name=\"arm_wrist_roll\" type=\"continuous\"/>
</robot>"""
    )

    transformed = VENDOR.transform_urdf(source)

    assert transformed.find("link/collision") is None
    assert (
        transformed.find(".//mesh").attrib["filename"]
        == VENDOR.MESH_URI_PREFIX + "meshes/base.stl"
    )
    assert transformed.find("joint[@name='base_left_wheel']").attrib["type"] == "fixed"
    arm = transformed.find("joint[@name='arm_wrist_roll']")
    assert arm.attrib["type"] == "${roll_joint}"
    assert arm.find("limit").attrib == VENDOR.ARM_LIMITS["arm_wrist_roll"]
