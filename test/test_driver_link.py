"""Pure checks for the driver's stale-telemetry detector."""

import ast
import pathlib
import types

_SOURCE = (pathlib.Path(__file__).parents[1] / "lekiwi_rmf" / "driver.py").read_text()
_TREE = ast.parse(_SOURCE)
_NODE = next(node for node in _TREE.body if getattr(node, "name", None) == "LeKiwiDriver")
_NODE.bases = []
_NODE.body = [item for item in _NODE.body if getattr(item, "name", None) == "observation_is_fresh"]
driver = types.ModuleType("driver_under_test")
exec(compile(ast.Module(body=[_NODE], type_ignores=[]), "driver.py", "exec"), driver.__dict__)


def test_repeated_cached_observation_is_not_fresh():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.last_observation = None
    cached = {"arm_shoulder_pan.pos": 12.0}
    assert node.observation_is_fresh(cached)
    assert not node.observation_is_fresh(cached)
    assert node.observation_is_fresh({"arm_shoulder_pan.pos": 12.0})
