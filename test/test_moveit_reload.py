"""The planner reload preserves calibrated poses and has no torque operation."""
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET
import yaml


def test_reload_replaces_collision_pairs_and_preserves_live_poses():
    script = Path(__file__).parents[1] / "scripts/reload-moveit.py"
    refresh = runpy.run_path(str(script))["refresh_collision_pairs"]
    current = '<robot name="lekiwi"><group_state name="closed"><joint name="jaw" value="0.321"/></group_state><disable_collisions link1="old" link2="old2"/></robot>'
    tracked = '<robot name="lekiwi"><group_state name="closed"><joint name="jaw" value="0"/></group_state><disable_collisions link1="shoulder" link2="wrist" reason="MechanicalRest"/></robot>'
    result = ET.fromstring(refresh(current, tracked))
    assert result.find("group_state/joint").get("value") == "0.321"
    assert [pair.attrib for pair in result.findall("disable_collisions")] == [
        {"link1": "shoulder", "link2": "wrist", "reason": "MechanicalRest"}
    ]
    assert "torque_request" not in script.read_text()


def test_reload_uses_the_managed_process_mount_namespace():
    script = Path(__file__).parents[1] / "scripts/reload-moveit.py"
    resolve = runpy.run_path(str(script))["planner_parameter_path"]
    assert resolve(123, ["move_group", "--params-file", "/tmp/launch_params_test"]) == Path(
        "/proc/123/root/tmp/launch_params_test"
    )


def test_reload_omits_unserializable_empty_array_defaults(tmp_path):
    script = Path(__file__).parents[1] / "scripts/reload-moveit.py"
    write = runpy.run_path(str(script))["write_parameters"]
    path = tmp_path / "parameters.yaml"
    write(path, {"empty_default": [], "limits": [0.1], "robot_description_semantic": "<robot/>"})
    assert yaml.safe_load(path.read_text())["/**"]["ros__parameters"] == {
        "limits": [0.1], "robot_description_semantic": "<robot/>"
    }
