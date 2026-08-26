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
        "arm", "clamp_planar", "observation_is_fresh", "observation_is_valid",
        "arm_after_startup_telemetry", "on_command", "publish_safety", "publish_state",
        "set_disarmed", "trajectory_time",
        "twist_is_finite", "update", "validate_motion_parameters",
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
    complete = {"joint.pos": 0.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
    assert driver.LeKiwiDriver.observation_is_valid(complete)
    assert not driver.LeKiwiDriver.observation_is_valid({**complete, "joint.pos": math.nan})
    assert not driver.LeKiwiDriver.observation_is_valid({**complete, "x.vel": math.inf})
    assert not driver.LeKiwiDriver.observation_is_valid({"joint.pos": 0.0})


def test_invalid_motion_scale_is_rejected():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.xy_scale = 0.0
    node.yaw_scale = 1.0
    node.command_timeout = node.link_timeout = 1.0
    node.trajectory_tolerance = node.trajectory_timeout = 1.0
    node.odom_xy_stddev = node.odom_yaw_stddev = 1.0
    node.twist_xy_stddev = node.twist_yaw_stddev = 1.0
    node.max_linear = node.max_angular = 1.0
    node.cmd_vel_topic = "/cmd_vel_safe"

    try:
        node.validate_motion_parameters()
    except ValueError as error:
        assert "xy_velocity_scale" in str(error)
    else:
        raise AssertionError("zero xy_velocity_scale was accepted")


def test_guarded_command_topic_is_the_default_and_must_not_be_empty():
    assert 'declare_parameter("cmd_vel_topic", "/cmd_vel_safe")' in _SOURCE
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.xy_scale = node.yaw_scale = 1.0
    node.command_timeout = node.link_timeout = 1.0
    node.trajectory_tolerance = node.trajectory_timeout = 1.0
    node.odom_xy_stddev = node.odom_yaw_stddev = 1.0
    node.twist_xy_stddev = node.twist_yaw_stddev = 1.0
    node.max_linear = node.max_angular = 1.0
    node.cmd_vel_topic = ""

    try:
        node.validate_motion_parameters()
    except ValueError as error:
        assert "cmd_vel_topic" in str(error)
    else:
        raise AssertionError("empty cmd_vel_topic was accepted")


def test_initial_healthy_telemetry_arms_once_but_link_recovery_does_not():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.auto_arm_pending = True
    node.link_lost = False
    node.armed = False
    node.state_lock = threading.Lock()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.publish_safety = lambda state: setattr(node, "safety", state)
    node.get_logger = lambda: type("Logger", (), {"info": lambda *_: None})()
    driver.Twist = object

    assert node.arm_after_startup_telemetry() is True
    assert node.armed is True
    assert node.auto_arm_pending is False
    assert node.safety == "ARMED"
    assert node.arm_after_startup_telemetry() is False

    node.auto_arm_pending = True
    node.link_lost = True
    node.armed = False
    assert node.arm_after_startup_telemetry() is False
    assert node.armed is False


def test_manual_arm_rejects_telemetry_older_than_link_timeout():
    class Now:
        def __sub__(self, other):
            return types.SimpleNamespace(nanoseconds=2_000_000_000)

    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Now())
    node.last_fresh = object()
    node.link_timeout = 1.0
    node.link_lost = False
    node.last_observation = {"complete": True}
    node.state_lock = threading.Lock()
    response = types.SimpleNamespace()

    assert node.arm(None, response) is response
    assert response.success is False
    assert "fresh" in response.message


def test_planar_velocity_is_clamped_by_vector_magnitude():
    x, y = driver.LeKiwiDriver.clamp_planar(0.3, 0.4, 0.25)
    assert math.isclose(math.hypot(x, y), 0.25)
    assert math.isclose(x / y, 0.3 / 0.4)


def test_odometry_covariance_never_claims_perfect_pose_or_twist():
    def pose():
        return types.SimpleNamespace(
            position=types.SimpleNamespace(), orientation=types.SimpleNamespace()
        )

    class Odometry:
        def __init__(self):
            self.header = types.SimpleNamespace()
            self.pose = types.SimpleNamespace(pose=pose())
            self.twist = types.SimpleNamespace(twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(), angular=types.SimpleNamespace()
            ))

    driver.Odometry = Odometry
    driver.TransformStamped = lambda: types.SimpleNamespace(
        transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(), rotation=types.SimpleNamespace()
        )
    )
    driver.JointState = lambda: types.SimpleNamespace(header=types.SimpleNamespace())
    driver.ARM_JOINTS = ("joint",)
    odometry, transform, joints = [], [], []
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.pose = (1.0, 2.0, 0.0)
    node.odom_xy_stddev, node.odom_yaw_stddev = 0.05, 0.10
    node.twist_xy_stddev, node.twist_yaw_stddev = 0.10, 0.20
    node.odom_pub = types.SimpleNamespace(publish=odometry.append)
    node.tf = types.SimpleNamespace(sendTransform=transform.append)
    node.arm_positions = {"joint": 0.0}
    node.joint_pub = types.SimpleNamespace(publish=joints.append)

    node.publish_state(object(), {}, (0.0, 0.0, 0.0))

    assert odometry[0].pose.covariance[0] == 0.05 ** 2
    assert odometry[0].pose.covariance[35] == 0.10 ** 2
    assert odometry[0].twist.covariance[0] == 0.10 ** 2
    assert odometry[0].twist.covariance[35] == 0.20 ** 2
    assert odometry[0].pose.covariance[14] == 1e6


def test_non_finite_twist_is_rejected_and_disarms():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.state_lock = threading.Lock()
    node.armed = True
    node.auto_arm_pending = True
    node.get_logger = lambda: type("Logger", (), {"error": lambda *_: None})()
    node.set_disarmed = lambda state: setattr(node, "disarmed_as", state)
    vector = lambda **values: types.SimpleNamespace(x=values.get("x", 0.0), y=0.0, z=0.0)
    message = types.SimpleNamespace(linear=vector(x=math.nan), angular=vector())

    node.on_command(message)

    assert node.disarmed_as == "DISARMED"


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
    assert node.auto_arm_pending is False
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
        "get_observation": lambda _: {
            "joint.pos": 0.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0,
        },
        "send_action": lambda _, action: sent.append(action),
    })()
    node.last_observation = None
    node.last_fresh = Stamp()
    node.link_lost = False
    node.armed = False
    node.auto_arm_pending = False
    node.stop_pending = True
    node.state_lock = threading.Lock()
    node.arm_zero_positions = {}
    node.arm_directions = {}
    node.trajectory_lock = threading.Lock()
    node.publish_state = lambda *_: None

    node.update()

    assert sent == [{"joint.pos": 0.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}]
    assert node.stop_pending is False


def test_safety_marker_matches_the_safety_state():
    class Marker:
        TEXT_VIEW_FACING = 9
        ADD = 0

        def __init__(self):
            self.header = types.SimpleNamespace()
            self.pose = types.SimpleNamespace(
                position=types.SimpleNamespace(), orientation=types.SimpleNamespace()
            )
            self.scale = types.SimpleNamespace()
            self.color = types.SimpleNamespace()

    messages = []
    markers = []
    driver.String = lambda: types.SimpleNamespace()
    driver.Marker = Marker
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: type("Now", (), {"to_msg": lambda _: object()})()})()
    node.safety_pub = types.SimpleNamespace(publish=messages.append)
    node.safety_marker_pub = types.SimpleNamespace(publish=markers.append)

    node.publish_safety("LINK_LOST")

    assert messages[0].data == "LINK_LOST"
    assert markers[0].text == "LINK_LOST"
    assert (markers[0].color.r, markers[0].color.g, markers[0].color.b) == (1.0, 0.0, 0.0)
