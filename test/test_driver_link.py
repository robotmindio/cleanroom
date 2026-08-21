"""Pure checks for the driver's stale-telemetry detector."""

import ast
import math
import pathlib
import threading
import types

_SOURCE = (pathlib.Path(__file__).parents[1] / "lekiwi_rmf" / "driver.py").read_text()
_TREE = ast.parse(_SOURCE)
_NODE = next(node for node in _TREE.body if getattr(node, "name", None) == "LeKiwiDriver")
_NODE.bases = []
_NODE.body = [
    item for item in _NODE.body
    if getattr(item, "name", None) in (
        "observation_is_fresh", "observation_is_valid", "set_disarmed", "update",
        "validate_motion_parameters"
    )
]
driver = types.ModuleType("driver_under_test")
exec(compile(ast.Module(body=[_NODE], type_ignores=[]), "driver.py", "exec"), driver.__dict__)
driver.math = math


def test_repeated_cached_observation_is_not_fresh():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.last_observation = None
    node.last_observation_token = None
    cached = {"arm_shoulder_pan.pos": 12.0}
    assert node.observation_is_fresh(cached)
    assert not node.observation_is_fresh(cached)
    assert not node.observation_is_fresh({"arm_shoulder_pan.pos": 12.0})


def test_mutated_cached_observation_is_fresh():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.last_observation = None
    node.last_observation_token = None
    cached = {"arm_shoulder_pan.pos": 12.0}

    assert node.observation_is_fresh(cached)
    cached["arm_shoulder_pan.pos"] = 13.0
    assert node.observation_is_fresh(cached)


def test_incomplete_or_non_finite_telemetry_is_rejected():
    driver.ARM_JOINTS = ("joint",)
    assert driver.LeKiwiDriver.observation_is_valid({"joint.pos": 0.0})
    assert not driver.LeKiwiDriver.observation_is_valid({"joint.pos": float("nan")})
    assert not driver.LeKiwiDriver.observation_is_valid({"other.pos": 0.0})


def test_invalid_motion_scale_is_rejected():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.xy_scale = 0.0
    node.yaw_scale = 1.0
    node.command_timeout = node.link_timeout = 1.0
    node.trajectory_tolerance = node.trajectory_timeout = 1.0
    node.max_linear = node.max_angular = 1.0

    try:
        node.validate_motion_parameters()
    except ValueError as error:
        assert "xy_velocity_scale" in str(error)
    else:
        raise AssertionError("zero xy_velocity_scale was accepted")


def test_disarm_queues_a_stop():
    driver.Twist = object
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.state_lock = threading.Lock()
    node.armed = True
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: object()})()
    node.cancel_trajectory = lambda outcome: setattr(node, "outcome", outcome)
    node.publish_safety = lambda state: setattr(node, "safety", state)
    node.get_logger = lambda: type("Logger", (), {"warn": lambda *_: None})()

    node.set_disarmed("DISARMED")

    assert node.stop_pending is True
    assert node.outcome == "safety disarmed"
    assert node.safety == "DISARMED"


def test_disarmed_driver_sends_zero_velocity_once():
    class Stamp:
        nanoseconds = 0

        def __sub__(self, other):
            return self

        def to_msg(self):
            return object()

    sent = []
    driver.ARM_JOINTS = ("joint",)
    driver.joint_positions = lambda *_: {"joint": 0.0}
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.last_update = Stamp()
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: Stamp()})()
    node.robot = type("Robot", (), {
        "get_observation": lambda _: {"joint.pos": 0.0},
        "send_action": lambda _, action: sent.append(action),
    })()
    node.last_observation = None
    node.last_fresh = Stamp()
    node.link_lost = False
    node.armed = False
    node.stop_pending = True
    node.arm_zero_positions = {}
    node.arm_directions = {}
    node.trajectory_lock = threading.Lock()
    node.publish_state = lambda *_: None

    node.update()

    assert sent == [{"joint.pos": 0.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}]
    assert node.stop_pending is False
