"""Render the simulation URDF as SDF and add omni-wheel contact semantics.

Modern Gazebo's URDF converter does not carry the legacy ``mu1/mu2/fdir1``
extensions into SDF.  Keeping the robot description as URDF is important for
ROS, while Gazebo needs anisotropic contact to represent passive omni rollers.
This renderer provides that small, deterministic simulation-only adaptation.
"""

from __future__ import annotations

import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


TRACTION_DIRECTIONS = {
    "sim_base_left_wheel_contact": "-0.866025 0.5 0",
    "sim_base_back_wheel_contact": "0 -1 0",
    "sim_base_right_wheel_contact": "0.866025 0.5 0",
}
GZ_XML_NAMESPACE = "http://gazebosim.org/schema"
ET.register_namespace("gz", GZ_XML_NAMESPACE)


def urdf_to_sdf(urdf: str) -> str:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as temporary:
        temporary.write(urdf)
        temporary.flush()
        result = subprocess.run(
            ["gz", "sdf", "-p", temporary.name],
            text=True,
            capture_output=True,
            check=True,
        )
    return result.stdout


def add_omni_contact(sdf: str) -> str:
    root = ET.fromstring(sdf)
    model = root.find("model")
    if model is None:
        raise ValueError("converted robot description has no SDF model")
    links = {link.attrib.get("name"): link for link in model.findall("link")}
    for link_name, direction in TRACTION_DIRECTIONS.items():
        link = links.get(link_name)
        if link is None:
            raise ValueError(f"converted robot description is missing {link_name}")
        collision = link.find("collision")
        if collision is None:
            raise ValueError(f"converted robot description has no collision for {link_name}")
        surface = ET.SubElement(collision, "surface")
        friction = ET.SubElement(surface, "friction")
        ode = ET.SubElement(friction, "ode")
        ET.SubElement(ode, "mu").text = "1.2"
        ET.SubElement(ode, "mu2").text = "0.02"
        fdir = ET.SubElement(
            ode, "fdir1", {f"{{{GZ_XML_NAMESPACE}}}expressed_in": "base_footprint"}
        )
        fdir.text = direction
        ET.SubElement(ode, "slip1").text = "0.01"
        ET.SubElement(ode, "slip2").text = "0.10"
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def render_simulation_sdf(xacro_path: str | Path) -> str:
    urdf = subprocess.run(
        ["xacro", str(xacro_path), "sim:=true"],
        text=True,
        capture_output=True,
        check=True,
    ).stdout
    return add_omni_contact(urdf_to_sdf(urdf))


def main() -> None:
    package = Path(get_package_share_directory("lekiwi_rmf"))
    print(render_simulation_sdf(package / "urdf" / "lekiwi.urdf.xacro"))


if __name__ == "__main__":
    main()
