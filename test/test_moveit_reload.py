"""The planner reload preserves calibrated poses and has no torque operation."""
from pathlib import Path
import json
import runpy
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import yaml
import pytest


def test_reload_replaces_collision_pairs_and_preserves_live_poses():
    script = Path(__file__).parents[1] / "scripts/reload-moveit.py"
    refresh = runpy.run_path(str(script))["refresh_collision_pairs"]
    current = '<robot name="lekiwi"><group_state name="closed"><joint name="jaw" value="0.321"/></group_state><disable_collisions link1="old" link2="old2"/></robot>'
    tracked = '<robot name="lekiwi"><group_state name="closed"><joint name="jaw" value="0"/></group_state><group_state name="supported_rest" group="arm"><joint name="arm_shoulder_lift" value="-1.4223"/></group_state><disable_collisions link1="shoulder" link2="wrist" reason="MechanicalRest"/></robot>'
    result = ET.fromstring(refresh(current, tracked))
    assert result.find("group_state/joint").get("value") == "0.321"
    assert [pair.attrib for pair in result.findall("disable_collisions")] == [
        {"link1": "shoulder", "link2": "wrist", "reason": "MechanicalRest"}
    ]
    assert "torque_request" not in script.read_text()
    assert result.find("group_state[@name='supported_rest']/joint").get('value') == '-1.4223'


def test_reload_refuses_missing_support_and_replaces_an_old_reference():
    script = Path(__file__).parents[1] / 'scripts/reload-moveit.py'
    refresh = runpy.run_path(str(script))['refresh_collision_pairs']
    current = '<robot name="lekiwi"><group_state name="supported_rest" group="arm"><joint name="lift" value="0"/></group_state><group_state name="travel_stow" group="arm"><joint name="lift" value="-1.8"/></group_state></robot>'
    with pytest.raises(ValueError, match='supported_rest'):
        refresh(current, '<robot name="lekiwi"/>')
    updated = ET.fromstring(refresh(current, '<robot name="lekiwi"><group_state name="supported_rest" group="arm"><joint name="lift" value="-1.4"/></group_state></robot>'))
    assert len(updated.findall("group_state[@name='supported_rest']")) == 1
    assert updated.find("group_state[@name='supported_rest']/joint").get('value') == '-1.4'
    assert updated.find("group_state[@name='travel_stow']/joint").get('value') == '-1.8'


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


@pytest.fixture
def sealed_planner_releases(tmp_path, monkeypatch):
    functions = runpy.run_path(str(Path(__file__).parents[1] / 'scripts/reload-moveit.py'))
    releases = tmp_path / 'releases'
    old, new = (releases / ('a' * 40), releases / ('b' * 40))
    for path in (old, new):
        path.mkdir(parents=True)
        (path / 'source').mkdir()
        (path / 'release.json').write_text(json.dumps({
            'dependency_overlay': {'path': 'same-overlay'}, 'settings': {'same': 'hash'},
        }))
    (tmp_path / 'current').symlink_to(old)
    (tmp_path / 'previous').symlink_to(old)
    changes, verified = [], []
    monkeypatch.setitem(functions['planner_release'].__globals__, 'subprocess', SimpleNamespace(
        run=lambda command, check: verified.append(command),
        check_output=lambda command, text: '\n'.join(changes),
    ))
    return functions, old, new, changes, verified


def test_planner_release_verifies_both_seals_and_switches_only_current(sealed_planner_releases):
    functions, old, new, changes, verified = sealed_planner_releases
    changes.extend(['src/rest_collision_env.hpp', 'config/lekiwi.srdf', 'docs/safety.md'])
    paths = functions['planner_release'](new)
    assert [command[2:] for command in verified] == [
        ['verify', str(path), path.name, 'compute'] for path in (old, new)]
    functions['select_planner_release'](*paths)
    assert (new.parent.parent / 'current').resolve() == new
    assert (new.parent.parent / 'previous').resolve() == old
    with pytest.raises(RuntimeError, match='changed'):
        functions['select_planner_release'](*paths)


@pytest.mark.parametrize('changed', ['config/safety_production.yaml', 'lekiwi_rmf/driver.py', 'urdf/lekiwi.urdf.xacro'])
def test_planner_release_refuses_other_runtime_changes(sealed_planner_releases, changed):
    functions, old, new, changes, verified = sealed_planner_releases
    changes.append(changed)
    with pytest.raises(ValueError, match='outside collision rules'):
        functions['planner_release'](new)
    assert (new.parent.parent / 'current').resolve() == old


@pytest.mark.parametrize('field', ['dependency_overlay', 'settings'])
def test_planner_release_refuses_changed_dependencies_or_settings(sealed_planner_releases, field):
    functions, old, new, changes, verified = sealed_planner_releases
    manifest = json.loads((new / 'release.json').read_text())
    manifest[field] = {'different': 'value'}
    (new / 'release.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='overlay or local settings'):
        functions['planner_release'](new)
    assert (new.parent.parent / 'current').resolve() == old
