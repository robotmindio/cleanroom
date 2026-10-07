"""Pin the parameters the safety nodes actually run with in each launch profile.

The supervisor and arm workspace monitor are built from the exact parameters
bringup resolves for them (``launch_snapshot``) and every declared parameter,
including code defaults, is compared with ``safety_parameters.json``.  Any
re-layering of the safety YAML files must leave these values unchanged.
Regenerate deliberately with ``LEKIWI_UPDATE_SNAPSHOTS=1``.
"""

import json
import os
from pathlib import Path

import pytest
import yaml

from launch_snapshot import find_node, resolve_bringup

ROOT = Path(__file__).parents[1]
GOLDEN = Path(__file__).parent / "safety_parameters.json"
PROFILES = {
    "sim": {"profile": "sim", "start_moveit": "true"},
    "wired": {"profile": "wired", "start_moveit": "true"},
    "wired_strict": {"profile": "wired", "start_moveit": "true", "disarm_on_failure": "true"},
}


def _materialize(value):
    if isinstance(value, str):
        return value.replace("$(share lekiwi_rmf)", str(ROOT))
    if isinstance(value, list):
        return [_materialize(item) for item in value]
    return value


def _running_parameters(node_class, name, parameters, tmp_path):
    rclpy = pytest.importorskip("rclpy")
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump({name: {"ros__parameters": {
        key: _materialize(value) for key, value in parameters.items()}}}))
    rclpy.init(args=["--ros-args", "--params-file", str(path)])
    node = None
    try:
        node = node_class()
        return {key: _materialize_back(parameter.value)
                for key, parameter in sorted(node._parameters.items())}
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def _materialize_back(value):
    if isinstance(value, str):
        return value.replace(str(ROOT), "$(share lekiwi_rmf)")
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return list(value)
    return value


def _profile_parameters(tmp_path):
    from lekiwi_rmf.arm_workspace_monitor import ArmWorkspaceMonitor
    from lekiwi_rmf.safety_supervisor import SafetySupervisor

    actual = {}
    for profile, arguments in PROFILES.items():
        records = resolve_bringup(**arguments)
        actual[profile] = {
            name: _running_parameters(node_class, name, find_node(records, name=name)["parameters"], tmp_path)
            for name, node_class in (("safety_supervisor", SafetySupervisor),
                                     ("arm_workspace_monitor", ArmWorkspaceMonitor))
        }
    return actual


def test_safety_nodes_run_with_unchanged_parameters_in_every_profile(tmp_path):
    actual = _profile_parameters(tmp_path)
    if os.environ.get("LEKIWI_UPDATE_SNAPSHOTS") == "1":
        GOLDEN.write_text(json.dumps(actual, indent=1, sort_keys=True) + "\n")
    assert actual == json.loads(GOLDEN.read_text())


def test_every_permission_consumer_uses_the_supervisors_tracked_lease():
    from lekiwi_rmf.arm_trajectory import ARM_JOINTS

    production = yaml.safe_load((ROOT / "config/safety_production.yaml").read_text())
    lease = production["safety_supervisor"]["ros__parameters"]["permission_timeout"]
    for profile in ("sim", "wired"):
        records = resolve_bringup(profile=profile)
        assert find_node(records, name="safety_supervisor")["parameters"]["permission_timeout"] == lease
        assert find_node(records, name="cmd_vel_mux")["parameters"]["permission_timeout"] == lease
    driver = find_node(resolve_bringup(profile="wired"), node="lekiwi_rmf/lekiwi_driver")
    assert driver["parameters"]["permission_timeout"] == lease
    (arm,) = [record["process"] for record in resolve_bringup(profile="sim")
              if "process" in record and "lekiwi_rmf.sim_arm_controller" in record["process"]]
    assert f"permission_timeout:={lease}" in arm
    # One arm joint list: the trajectory module's, mirrored by the base profile.
    assert production["safety_supervisor"]["ros__parameters"]["stow_joint_names"] == list(ARM_JOINTS)
    assert production["arm_workspace_monitor"]["ros__parameters"]["joint_names"] == list(ARM_JOINTS)
    gate = find_node(resolve_bringup(profile="sim"), name="wait_for_stable_joint_states")
    assert gate["parameters"]["joint_names"] == list(ARM_JOINTS)


@pytest.mark.parametrize("module,node_class", [
    ("lekiwi_rmf.cmd_vel_mux", "CmdVelMux"), ("lekiwi_rmf.sim_arm_controller", "SimArmController"),
])
def test_permission_consumers_refuse_to_start_without_the_lease(module, node_class):
    import importlib

    rclpy = pytest.importorskip("rclpy")
    from rclpy.exceptions import ParameterUninitializedException

    rclpy.init(args=[])
    try:
        with pytest.raises(ParameterUninitializedException):
            getattr(importlib.import_module(module), node_class)()
    finally:
        rclpy.try_shutdown()


def test_simulation_profile_only_overrides_the_physical_profile():
    production = yaml.safe_load((ROOT / "config/safety_production.yaml").read_text())
    simulation = yaml.safe_load((ROOT / "config/safety_simulation.yaml").read_text())
    for node, section in simulation.items():
        overrides = section["ros__parameters"]
        base = production[node]["ros__parameters"]
        assert set(overrides) <= set(base)
        assert all(base.get(key) != value for key, value in overrides.items()), node


def test_mapping_quota_has_one_tracked_source():
    from launch_snapshot import declared_arguments

    explorer = yaml.safe_load((ROOT / "config/exploration.yaml").read_text())["robot_explorer"]["ros__parameters"]
    arguments = declared_arguments(profile="sim")
    assert int(arguments["rtabmap_mapping_max_bytes"]["default"]) == explorer["mapping_max_bytes"]
    assert float(arguments["rtabmap_mapping_max_seconds"]["default"]) == explorer["mapping_max_seconds"]


def test_tracked_acceptance_fits_the_tracked_nav2_and_stow_configuration():
    from lekiwi_rmf.safety_acceptance import validate_tracked_acceptance

    assert validate_tracked_acceptance(ROOT / "config") == (True, "physical safety acceptance validated")
