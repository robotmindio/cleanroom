"""Pure checks for the driver's stale-telemetry detector."""

import ast
import math
import pathlib
import threading
import time
import types

import pytest

from lekiwi_rmf.motion_guards import lease_is_fresh, twist_is_finite

_SOURCE = (pathlib.Path(__file__).parents[1] / "lekiwi_rmf" / "driver.py").read_text()
_TREE = ast.parse(_SOURCE)
_NODE = next(node for node in _TREE.body if getattr(node, "name", None) == "LeKiwiDriver")
_NODE.bases = []
_NODE.body = [
    item for item in _NODE.body
    if getattr(item, "name", None) in (
        "arm", "disarm", "clamp", "clamp_planar", "observation_is_fresh", "observation_is_valid",
        "handle_host_session_change",
        "enforce_reported_torque_state",
        "arm_after_startup_telemetry", "on_command", "publish_safety", "publish_state", "publish_motor_health",
        "set_disarmed", "set_servo_torque", "cut_torque_after_failure", "_retry_rearm_soon",
        "_enable_torque_and_arm", "_arm_permission_is_current", "_run_deferred_cut",
        "_permission_is_current",
        "_capability_permission_is_current", "enforce_permission_leases",
        "on_base_permission", "on_arm_permission",
        "record_link_loss", "update", "validate_motion_parameters",
        "_poll_telemetry", "_hold_action", "_send_pending_stop", "_apply_trajectory",
        "_send_armed_command", "auto_arm_tick", "execute_trajectory", "destroy_node",
    )
]
_CONSTANTS = [
    node for node in _TREE.body
    if isinstance(node, ast.Assign)
    and getattr(node.targets[0], "id", "") == "MAX_TRAJECTORY_START_DELAY_NS"
]
driver = types.ModuleType("driver_under_test")
exec(
    compile(ast.Module(body=[*_CONSTANTS, _NODE], type_ignores=[]), "driver.py", "exec"),
    driver.__dict__,
)
driver.math = math
driver.time = time
driver.lease_is_fresh = lease_is_fresh
driver.twist_is_finite = twist_is_finite


def make_node(**overrides):
    """A driver in the state ``__init__`` leaves it, without ROS or a motor host.

    The tests below written before staying armed became the default cover the opt-in
    strict mode (disarm, cut torque and wait for an operator on every failure), so that
    is the baseline here. The default has its own tests at the end.
    """
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    state = {
        "disarm_on_failure": True,
        "operator_disarmed": False,
        "auto_arm_pending": False,
        "_next_rearm_at": 0.0,
        "armed": False,
        "torque_fault": False,
        "link_lost": False,
        "_healthy_telemetry_at": None,
        "stop_pending": True,
        "_disarm_epoch": 0,
        "_deferred_cut": False,
        "base_motion_permitted": False,
        "arm_motion_permitted": False,
        "_base_permission_received_at_ns": None,
        "_arm_permission_received_at_ns": None,
        "_arm_permission_expired": False,
        "permission_timeout_ns": 10_000_000_000,
        "link_timeout": 1.0,
        "command_timeout": 0.4,
        "robot": None,
        "context": None,
        "last_observation": None,
        "last_observation_token": None,
        "trajectory": None,
        "publish_motor_health_enabled": True,
        "safety_state": "DISARMED",
        "state_lock": threading.Lock(),
        "action_lock": threading.Lock(),
        "torque_lock": threading.Lock(),
        "trajectory_lock": threading.Lock(),
        "safety_publish_lock": threading.Lock(),
    }
    state.update(overrides)
    for name, value in state.items():
        setattr(node, name, value)
    return node


def grant_fresh_arm_permission(node, permitted=True):
    node.arm_motion_permitted = permitted
    node._arm_permission_received_at_ns = time.monotonic_ns()
    node.permission_timeout_ns = 10_000_000_000


def grant_fresh_base_permission(node, permitted=True):
    node.base_motion_permitted = permitted
    node._base_permission_received_at_ns = time.monotonic_ns()
    node.permission_timeout_ns = 10_000_000_000


def test_repeated_cached_observation_is_not_fresh():
    node = make_node()
    node.last_observation = None
    node.last_observation_token = None
    cached = {"arm_shoulder_pan.pos": 12.0}
    assert node.observation_is_fresh(cached)
    assert not node.observation_is_fresh(cached)
    assert not node.observation_is_fresh({"arm_shoulder_pan.pos": 12.0})


def test_mutated_cached_observation_is_fresh():
    node = make_node()
    node.last_observation = None
    node.last_observation_token = None
    cached = {"arm_shoulder_pan.pos": 12.0}

    assert node.observation_is_fresh(cached)
    cached["arm_shoulder_pan.pos"] = 13.0
    assert node.observation_is_fresh(cached)


def test_client_without_an_accepted_packet_is_not_fresh():
    node = make_node()
    node.robot = types.SimpleNamespace(observation_token=None, observation_sequence=0)
    node.last_observation_token = None

    assert not node.observation_is_fresh({"x.vel": 0.0})


def test_permission_lease_uses_receive_monotonic_time_and_expires():
    assert lease_is_fresh(1_000, 100, 1_100)
    assert not lease_is_fresh(1_000, 100, 1_101)
    assert not lease_is_fresh(1_000, 100, 999)
    assert not lease_is_fresh(None, 100, 1_000)


def test_arm_permission_lease_expiry_disarms_and_base_expiry_zeros_command():
    driver.Twist = object
    node = make_node()
    node.permission_timeout_ns = 100
    node.state_lock = threading.Lock()
    node.armed = True
    node.arm_motion_permitted = True
    node.base_motion_permitted = True
    node._arm_permission_received_at_ns = 1_000
    node._base_permission_received_at_ns = 1_000
    node._arm_permission_expired = False
    node.command = object()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)
    node.cancel_trajectory = lambda _outcome: None
    disarms = []
    node.set_disarmed = lambda state, **_: disarms.append(state)

    assert node.enforce_permission_leases(now_monotonic_ns=1_101)

    assert node.arm_motion_permitted is False
    assert node.base_motion_permitted is False
    assert node.command is not None
    assert disarms == ["DISARMED"]


def test_explicit_arm_accepts_fresh_base_capability_lease():
    class Now:
        def __sub__(self, _other):
            return types.SimpleNamespace(nanoseconds=0)

    driver.Twist = object
    node = make_node()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Now())
    node.last_fresh = object()
    node.link_timeout = 1.0
    node.link_lost = False
    node.last_observation = {"complete": True}
    grant_fresh_base_permission(node)
    node.arm_motion_permitted = False
    node._arm_permission_received_at_ns = None
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    node.set_servo_torque = lambda enabled: enabled
    states = []
    node.publish_safety = lambda state=None, **_: states.append(state)
    response = types.SimpleNamespace()

    node.arm(None, response)

    assert response.success is True
    assert node.armed is True
    assert states == ["ARMED"]


def test_arm_permission_withdrawal_keeps_torque_when_base_lease_is_current():
    driver.Twist = object
    node = make_node()
    node.permission_timeout_ns = 10_000_000_000
    node.state_lock = threading.Lock()
    node.armed = True
    node.arm_motion_permitted = True
    node._arm_permission_expired = False
    grant_fresh_base_permission(node)
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    messages = []
    node.get_logger = lambda: types.SimpleNamespace(error=messages.append)
    cancellations = []
    node.cancel_trajectory = cancellations.append
    disarms = []
    node.set_disarmed = lambda state, **_: disarms.append(state)

    node.on_arm_permission(types.SimpleNamespace(data=False))
    # Repeated false heartbeats are lease refreshes, not repeated withdrawal events.
    node.on_arm_permission(types.SimpleNamespace(data=False))

    assert cancellations == ["arm safety permission withdrawn"]
    assert disarms == []
    assert len(messages) == 1


def test_base_permission_lease_expiry_does_not_require_arm_disarm():
    driver.Twist = object
    node = make_node()
    node.permission_timeout_ns = 100
    node.state_lock = threading.Lock()
    node.armed = True
    node.arm_motion_permitted = True
    node.base_motion_permitted = True
    node._arm_permission_received_at_ns = 1_050
    node._base_permission_received_at_ns = 1_000
    node._arm_permission_expired = False
    node.command = object()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    disarms = []
    node.set_disarmed = lambda state, **_: disarms.append(state)

    assert not node.enforce_permission_leases(now_monotonic_ns=1_101)

    assert node.base_motion_permitted is False
    assert node.arm_motion_permitted is True
    assert disarms == []


def test_host_session_restart_forces_disarm_and_odometry_reset():
    node = make_node()
    node.robot = types.SimpleNamespace(observation_session_changed=True)
    reset = []
    node.odom_samples = types.SimpleNamespace(reset=lambda: reset.append(True))
    node.set_disarmed = lambda state, **_: setattr(node, "disarmed_as", state)
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)

    assert node.handle_host_session_change() is True
    assert reset == [True]
    assert node.disarmed_as == "DISARMED"


def test_authenticated_host_torque_cut_cannot_leave_driver_logically_armed():
    node = make_node()
    node.robot = types.SimpleNamespace(observation_torque_enabled=False)
    node.state_lock = threading.Lock()
    node.armed = True
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)
    disarms = []
    node.set_disarmed = lambda state, **_: disarms.append(state)

    assert node.enforce_reported_torque_state()
    assert disarms == ["DISARMED"]


def test_incomplete_or_non_finite_telemetry_is_rejected():
    driver.ARM_JOINTS = ("joint",)
    complete = {"joint.pos": 0.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
    assert driver.LeKiwiDriver.observation_is_valid(complete)
    assert not driver.LeKiwiDriver.observation_is_valid(complete, ("joint.pos",))
    assert not driver.LeKiwiDriver.observation_is_valid({**complete, "joint.pos": math.nan})
    assert not driver.LeKiwiDriver.observation_is_valid({**complete, "x.vel": math.inf})
    assert not driver.LeKiwiDriver.observation_is_valid({"joint.pos": 0.0})


def test_link_loss_is_logged_and_disarmed_once():
    node = make_node()
    node.link_lost = False
    logs, resets, disarms = [], [], []
    node.get_logger = lambda: types.SimpleNamespace(error=logs.append)
    node.odom_samples = types.SimpleNamespace(reset=lambda: resets.append(True))
    node.set_disarmed = lambda state, **_: disarms.append(state)

    node.record_link_loss("telemetry failed")
    node.record_link_loss("telemetry failed again")

    assert logs == ["telemetry failed"]
    assert resets == [True]
    assert disarms == ["LINK_LOST"]


def test_invalid_motion_scale_is_rejected():
    node = make_node()
    node.xy_scale = 0.0
    node.yaw_scale = 1.0
    node.command_timeout = node.link_timeout = node.permission_timeout = 1.0
    node.trajectory_path_tolerance = 1.0
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


def test_nonpositive_default_path_tolerance_is_rejected():
    node = make_node()
    node.xy_scale = node.yaw_scale = 1.0
    node.command_timeout = node.link_timeout = node.permission_timeout = 1.0
    node.trajectory_path_tolerance = 0.0
    node.trajectory_tolerance = node.trajectory_timeout = 1.0
    node.odom_xy_stddev = node.odom_yaw_stddev = 1.0
    node.twist_xy_stddev = node.twist_yaw_stddev = 1.0
    node.max_linear = node.max_angular = 1.0
    node.cmd_vel_topic = "/cmd_vel_safe"

    with pytest.raises(ValueError, match="trajectory_path_tolerance"):
        node.validate_motion_parameters()


def test_guarded_command_topic_is_the_default_and_must_not_be_empty():
    assert 'declare_parameter("cmd_vel_topic", "/cmd_vel_safe")' in _SOURCE
    node = make_node()
    node.xy_scale = node.yaw_scale = 1.0
    node.command_timeout = node.link_timeout = node.permission_timeout = 1.0
    node.trajectory_path_tolerance = 1.0
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


def test_configured_startup_arm_still_requires_supervisor_permission():
    node = make_node()
    node.auto_arm_pending = True
    node.link_lost = False
    node.armed = False
    grant_fresh_arm_permission(node, False)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.publish_safety = lambda state=None, **_: setattr(node, "safety", state)
    node.set_servo_torque = lambda enabled: enabled
    node.get_logger = lambda: type("Logger", (), {"info": lambda *_: None})()
    driver.Twist = object

    assert node.arm_after_startup_telemetry() is False
    assert node.armed is False

    grant_fresh_arm_permission(node)
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

    node = make_node()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Now())
    node.last_fresh = object()
    node.link_timeout = 1.0
    node.link_lost = False
    node.last_observation = {"complete": True}
    grant_fresh_arm_permission(node)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    node.set_servo_torque = lambda enabled: enabled
    response = types.SimpleNamespace()

    assert node.arm(None, response) is response
    assert response.success is False
    assert "fresh" in response.message


def test_unconfirmed_torque_enable_is_followed_by_fail_safe_disable():
    class Now:
        def __sub__(self, other):
            return types.SimpleNamespace(nanoseconds=0)

    node = make_node()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Now())
    node.last_fresh = object()
    node.link_timeout = 1.0
    node.link_lost = False
    node.last_observation = {"complete": True}
    grant_fresh_arm_permission(node)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    node.publish_safety = lambda _state=None, **_: None
    requests = []
    state_lock_was_free = []

    def torque(enabled):
        acquired = node.state_lock.acquire(blocking=False)
        state_lock_was_free.append(acquired)
        if acquired:
            node.state_lock.release()
        requests.append(enabled)
        return not enabled

    node.set_servo_torque = torque
    response = types.SimpleNamespace()

    assert node.arm(None, response) is response
    assert response.success is False
    assert requests == [True, False]
    assert state_lock_was_free == [True, True]


def test_manual_arm_latches_fault_when_ambiguous_enable_cannot_be_cut():
    class Now:
        def __sub__(self, _other):
            return types.SimpleNamespace(nanoseconds=0)

    node = make_node()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Now())
    node.last_fresh = object()
    node.link_timeout = 1.0
    node.link_lost = False
    node.last_observation = {"complete": True}
    grant_fresh_arm_permission(node)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    requests = []
    node.set_servo_torque = lambda enabled: requests.append(enabled) or False
    states = []
    node.publish_safety = lambda state=None, **_: states.append(state)
    response = types.SimpleNamespace()

    node.arm(None, response)

    assert response.success is False
    assert "not confirmed" in response.message
    assert requests == [True, False]
    assert node.torque_fault is True
    assert states == ["TORQUE_FAULT"]


def test_unconfirmed_startup_enable_cannot_leave_logical_arm_state_set():
    driver.Twist = object
    node = make_node()
    node.auto_arm_pending = True
    node.link_lost = False
    node.armed = False
    grant_fresh_arm_permission(node)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    requests = []
    node.set_servo_torque = lambda enabled: requests.append(enabled) or not enabled
    node.publish_safety = lambda state=None, **_: setattr(node, "safety", state)
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)

    assert node.arm_after_startup_telemetry() is False
    assert requests == [True, False]
    assert node.armed is False
    assert node.auto_arm_pending is False
    assert node.safety == "DISARMED"


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
    node = make_node()
    node.pose = (1.0, 2.0, 0.0)
    node.odom_xy_stddev, node.odom_yaw_stddev = 0.05, 0.10
    node.twist_xy_stddev, node.twist_yaw_stddev = 0.10, 0.20
    node.odom_pub = types.SimpleNamespace(publish=odometry.append)
    node.tf = types.SimpleNamespace(sendTransform=transform.append)
    node.publish_odom_tf = True
    node.arm_positions = {"joint": 0.0}
    node.joint_pub = types.SimpleNamespace(publish=joints.append)
    raw_joints = []
    node.raw_joint_pub = types.SimpleNamespace(publish=raw_joints.append)
    driver.raw_joint_positions = lambda observation: {"joint": 0.75}

    node.publish_state(object(), {}, (0.0, 0.0, 0.0))
    assert raw_joints[0].position == [0.75]
    assert joints[0].position == [0.0]

    assert odometry[0].pose.covariance[0] == 0.05 ** 2
    assert odometry[0].pose.covariance[35] == 0.10 ** 2
    assert odometry[0].twist.covariance[0] == 0.10 ** 2
    assert odometry[0].twist.covariance[35] == 0.20 ** 2
    assert odometry[0].pose.covariance[14] == 1e6


def test_validated_motor_health_is_published_as_diagnostics():
    class DiagnosticArray:
        def __init__(self):
            self.header = types.SimpleNamespace()

    class DiagnosticStatus:
        def __init__(self):
            self.values = []

    class KeyValue:
        pass

    driver.DiagnosticArray = DiagnosticArray
    driver.DiagnosticStatus = DiagnosticStatus
    driver.KeyValue = KeyValue
    published = []
    node = make_node()
    node.robot = types.SimpleNamespace(observation_motor_health=(
        types.SimpleNamespace(
            name="motor_bus", level=0, message="OK",
            values=(("torque_enabled", "false"),),
        ),
    ))
    node.motor_health_pub = types.SimpleNamespace(publish=published.append)

    node.publish_motor_health("stamp")

    assert published[0].header.stamp == "stamp"
    assert published[0].status[0].name == "motor_bus"
    assert published[0].status[0].level == b"\x00"
    assert published[0].status[0].values[0].key == "torque_enabled"


def test_non_finite_twist_is_rejected_and_disarms():
    node = make_node()
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.armed = True
    node.torque_fault = False
    node.auto_arm_pending = True
    node.get_logger = lambda: type("Logger", (), {"error": lambda *_: None})()
    node.set_disarmed = lambda state, **_: setattr(node, "disarmed_as", state)
    def vector(**values):
        return types.SimpleNamespace(x=values.get("x", 0.0), y=0.0, z=0.0)

    message = types.SimpleNamespace(linear=vector(x=math.nan), angular=vector())

    node.on_command(message)

    assert node.disarmed_as == "DISARMED"


def test_disarm_queues_a_stop():
    driver.Twist = object
    node = make_node()
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.armed = True
    node.torque_fault = False
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: object()})()
    node.cancel_trajectory = lambda outcome: setattr(node, "outcome", outcome)
    node.publish_safety = lambda state=None, **_: setattr(node, "safety", state)
    node.get_logger = lambda: type("Logger", (), {"warn": lambda *_: None})()
    torque_requests = []
    node.set_servo_torque = lambda enabled: torque_requests.append(enabled) or not enabled

    node.set_disarmed("DISARMED")

    assert node.stop_pending is True
    assert node.auto_arm_pending is False
    assert node.outcome == "safety disarmed"
    assert node.safety == "DISARMED"
    assert torque_requests == [False]


def test_unconfirmed_cut_latches_torque_fault_until_explicit_confirmed_disarm():
    class Now:
        def __sub__(self, _other):
            return types.SimpleNamespace(nanoseconds=0)

    driver.Twist = object
    node = make_node()
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.armed = True
    node.torque_fault = False
    node.auto_arm_pending = True
    node.arm_motion_permitted = True
    node.link_lost = False
    node.last_observation = {"complete": True}
    node.last_fresh = object()
    node.link_timeout = 1.0
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Now())
    node.cancel_trajectory = lambda _outcome: None
    states = []
    node.publish_safety = lambda state=None, **_: states.append(state)
    node.get_logger = lambda: types.SimpleNamespace(warn=lambda *_: None)
    cuts_confirmed = iter((False, True))
    node.set_servo_torque = lambda enabled: not enabled and next(cuts_confirmed)

    # This models an asynchronous permission/watchdog disarm, not an operator
    # acknowledgement. An unconfirmed cut becomes an externally visible latch.
    assert node.set_disarmed("LINK_LOST") is False
    assert node.torque_fault is True
    assert node.armed is False
    assert states == ["TORQUE_FAULT"]

    response = types.SimpleNamespace()
    node.arm(None, response)
    assert response.success is False
    assert "fault-latched" in response.message

    disarm_response = types.SimpleNamespace()
    node.disarm(None, disarm_response)
    assert disarm_response.success is True
    assert node.torque_fault is False
    assert states[-1] == "DISARMED"


def test_shutdown_style_disarm_latches_fault_without_publishing_on_rpc_exception():
    driver.Twist = object
    driver.rclpy = types.SimpleNamespace(ok=lambda **_kwargs: False)
    node = make_node()
    node.context = object()
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_lock = threading.Lock()
    node.armed = False
    node.torque_fault = False
    node.auto_arm_pending = False
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.cancel_trajectory = lambda _outcome: None
    node.publish_safety = lambda _state=None, **_: pytest.fail("shutdown published into a dead context")
    node.torque = types.SimpleNamespace(
        set_enabled=lambda _enabled: (_ for _ in ()).throw(RuntimeError("transport died"))
    )

    assert node.set_disarmed("DISARMED", publish=False) is False
    assert node.torque_fault is True


def test_disarm_keeps_state_observable_while_serializing_physical_actions():
    driver.Twist = object
    node = make_node()
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.armed = True
    node.torque_fault = False
    node.auto_arm_pending = True
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: object()})()
    node.cancel_trajectory = lambda _outcome: None
    node.publish_safety = lambda _state=None, **_: None
    node.get_logger = lambda: type("Logger", (), {"warn": lambda *_: None})()
    entered = threading.Event()
    release = threading.Event()
    node.set_servo_torque = lambda enabled: (entered.set(), release.wait(1), not enabled)[2]

    worker = threading.Thread(target=node.set_disarmed, args=("DISARMED",))
    worker.start()
    assert entered.wait(1)
    assert node.state_lock.acquire(blocking=False)
    node.state_lock.release()
    assert not node.action_lock.acquire(blocking=False)
    release.set()
    worker.join(1)
    assert not worker.is_alive()


def test_disarmed_driver_sends_zero_velocity_once_but_publishes_measured_motion():
    class Stamp:
        nanoseconds = 0

        def __sub__(self, other):
            return self

        def to_msg(self):
            return object()

    sent = []
    driver.ARM_JOINTS = ("joint",)
    driver.joint_positions = lambda *_: {"joint": 0.0}
    driver.integrate_pose = lambda pose, _velocity, _dt: pose
    published = []
    node = make_node()
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: Stamp()})()
    node.robot = type("Robot", (), {
        "get_observation": lambda _: {
            "joint.pos": 0.0, "x.vel": 0.2, "y.vel": 0.0, "theta.vel": 0.0,
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
    node.action_lock = threading.Lock()
    node.torque_fault = False
    node.arm_zero_positions = {}
    node.arm_directions = {}
    node.trajectory_lock = threading.Lock()
    node.odom_samples = types.SimpleNamespace(
        accept=lambda *_: None, reset=lambda: None, discontinuity=None
    )
    node.pose = (0.0, 0.0, 0.0)
    node.xy_scale = node.yaw_scale = 1.0
    node.publish_state = lambda *args: published.append(args)
    safety_refreshes = []
    node.publish_safety = lambda *args, **_: safety_refreshes.append(args)
    node.enforce_reported_torque_state = lambda: False

    state_lock_was_free = []
    def send_while_checking_lock(_robot, action):
        acquired = node.state_lock.acquire(blocking=False)
        state_lock_was_free.append(acquired)
        if acquired:
            node.state_lock.release()
        sent.append(action)
    node.robot.send_action = types.MethodType(send_while_checking_lock, node.robot)

    node.update()

    assert state_lock_was_free == [True]
    assert sent == [{"joint.pos": 0.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}]
    assert node.stop_pending is False
    assert published[0][2] == (0.2, 0.0, 0.0)
    assert safety_refreshes == [()]


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
    node = make_node()
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: type("Now", (), {"to_msg": lambda _: object()})()})()
    node.safety_pub = types.SimpleNamespace(publish=messages.append)
    node.safety_marker_pub = types.SimpleNamespace(publish=markers.append)
    node.safety_publish_lock = threading.Lock()
    node.safety_state = "DISARMED"

    node.publish_safety("LINK_LOST")

    assert messages[0].data == "LINK_LOST"
    assert markers[0].text == "LINK_LOST"
    assert (markers[0].color.r, markers[0].color.g, markers[0].color.b) == (1.0, 0.0, 0.0)

    node.publish_safety("TORQUE_FAULT")
    assert messages[-1].data == "TORQUE_FAULT"
    assert markers[-1].text == "TORQUE_FAULT"


def hold_mode_node(unreachable_host=True):
    """A driver with the default policy whose torque host never answers a cut."""
    driver.Twist = object
    node = make_node()
    node.disarm_on_failure = False
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.armed = True
    node.torque_fault = False
    node.auto_arm_pending = False
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.cancel_trajectory = lambda _outcome: None
    node.states = []
    node.publish_safety = lambda state=None, **_: node.states.append(state)
    node.get_logger = lambda: types.SimpleNamespace(warn=lambda *_: None)
    node.torque_requests = []
    node.set_servo_torque = lambda enabled: node.torque_requests.append(enabled) or not unreachable_host
    return node


def test_failure_stops_motion_but_never_cuts_or_latches_torque_by_default():
    node = hold_mode_node()

    assert node.set_disarmed("LINK_LOST") is True
    assert node.torque_requests == []
    assert node.torque_fault is False
    assert node.armed is False
    assert node.stop_pending is True
    assert node.states == ["LINK_LOST"]


def test_explicit_disarm_latches_an_unconfirmed_cut_in_every_mode():
    node = hold_mode_node()
    response = types.SimpleNamespace()

    node.disarm(None, response)

    assert node.torque_requests == [False]
    assert response.success is False
    assert node.torque_fault is True
    assert node.states == ["TORQUE_FAULT"]

    node.set_servo_torque = lambda enabled: not enabled
    node.disarm(None, response)
    assert response.success is True
    assert node.torque_fault is False
    assert node.states[-1] == "DISARMED"

    node.set_servo_torque = lambda enabled: False
    node.disarm_on_failure = True
    node.disarm(None, response)
    assert node.torque_fault is True
    assert node.states[-1] == "TORQUE_FAULT"


def test_opt_in_restores_cutting_torque_on_failure():
    node = hold_mode_node()
    node.disarm_on_failure = True

    assert node.set_disarmed("LINK_LOST") is False
    assert node.torque_requests == [False]
    assert node.torque_fault is True
    assert node.states == ["TORQUE_FAULT"]


def test_torque_held_after_a_failure_is_not_treated_as_an_outside_change():
    node = hold_mode_node()
    node.armed = False
    node.robot = types.SimpleNamespace(observation_torque_enabled=True)
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)
    disarms = []
    node.set_disarmed = lambda state, **_: disarms.append(state)

    assert node.enforce_reported_torque_state() is False
    assert disarms == []

    node.disarm_on_failure = True
    assert node.enforce_reported_torque_state() is True
    assert disarms == ["DISARMED"]


def test_unconfirmed_arm_leaves_torque_alone_by_default():
    class Now:
        def __sub__(self, other):
            return types.SimpleNamespace(nanoseconds=0)

    node = hold_mode_node()
    node.armed = False
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Now())
    node.last_fresh = object()
    node.link_timeout = 1.0
    node.link_lost = False
    node.last_observation = {"complete": True}
    grant_fresh_arm_permission(node)
    response = types.SimpleNamespace()

    assert node.arm(None, response) is response
    assert response.success is False
    assert node.torque_requests == [True]
    assert node.torque_fault is False
    assert "left unchanged" in response.message


def rearm_node(strict=False):
    """A hold-mode driver whose telemetry and supervisor permission are healthy."""
    node = hold_mode_node(unreachable_host=False)
    node.disarm_on_failure = strict
    node.armed = False
    node.auto_arm_pending = False
    node.link_lost = False
    grant_fresh_arm_permission(node)
    node.get_logger = lambda: types.SimpleNamespace(
        warn=lambda *_: None, error=lambda *_: None, info=lambda *_: None
    )
    return node


def test_a_failure_is_followed_by_an_automatic_rearm_by_default():
    node = rearm_node()
    node.armed = True

    node.set_disarmed("LINK_LOST")
    assert node.armed is False and node.auto_arm_pending is True

    assert node.arm_after_startup_telemetry() is True
    assert node.armed is True
    assert node.states[-1] == "ARMED"
    assert node.torque_requests == [True]  # never cut, then enabled again


def test_an_operator_disarm_stays_disarmed_and_only_an_explicit_arm_resumes():
    node = rearm_node()
    node.armed = True

    node.disarm(None, types.SimpleNamespace())

    assert node.armed is False and node.auto_arm_pending is False
    assert node.arm_after_startup_telemetry() is False
    assert node.armed is False


def test_a_failure_after_an_operator_disarm_never_rearms_the_robot():
    node = rearm_node()
    node.armed = True
    node.disarm(None, types.SimpleNamespace())

    node.set_disarmed("LINK_LOST")  # e.g. the link drops while the operator has it disarmed

    assert node.auto_arm_pending is False
    assert node.arm_after_startup_telemetry() is False
    node._retry_rearm_soon()
    assert node.auto_arm_pending is False


def test_an_explicit_arm_hands_recovery_back_to_the_automatic_rearm():
    class _Instant:
        nanoseconds = 0

        def __sub__(self, _other):
            return types.SimpleNamespace(nanoseconds=0)

    node = rearm_node()
    node.armed = True
    node.disarm(None, types.SimpleNamespace())
    node.get_clock = lambda: types.SimpleNamespace(now=_Instant)
    node.last_fresh = _Instant()
    node.last_observation = {"joint": 0.0}
    node.link_timeout = 1.0
    node._capability_permission_is_current = lambda: True

    response = node.arm(None, types.SimpleNamespace())

    assert response.success is True and node.operator_disarmed is False
    node.set_disarmed("LINK_LOST")
    assert node.auto_arm_pending is True


def test_strict_mode_waits_for_an_explicit_arm_after_a_failure():
    node = rearm_node(strict=True)
    node.armed = True

    node.set_disarmed("LINK_LOST")

    assert node.auto_arm_pending is False
    assert node.arm_after_startup_telemetry() is False


def test_automatic_rearm_waits_for_telemetry_and_supervisor_permission():
    node = rearm_node()
    node.auto_arm_pending = True

    node.link_lost = True
    assert node.arm_after_startup_telemetry() is False
    node.link_lost = False
    grant_fresh_arm_permission(node, permitted=False)
    assert node.arm_after_startup_telemetry() is False
    assert node.torque_requests == []

    grant_fresh_arm_permission(node)
    assert node.arm_after_startup_telemetry() is True


def test_a_failed_automatic_rearm_is_retried_at_a_bounded_rate():
    node = rearm_node()
    node.auto_arm_pending = True
    node.set_servo_torque = lambda enabled: node.torque_requests.append(enabled) or False

    assert node.arm_after_startup_telemetry() is False
    assert node.auto_arm_pending is True and node.armed is False
    attempts = len(node.torque_requests)

    assert node.arm_after_startup_telemetry() is False  # inside the retry window
    assert len(node.torque_requests) == attempts

    node._next_rearm_at = 0.0
    node.set_servo_torque = lambda enabled: node.torque_requests.append(enabled) or True
    assert node.arm_after_startup_telemetry() is True


class _Stamp:
    nanoseconds = 0

    def __sub__(self, _other):
        return self

    def to_msg(self):
        return object()


def control_loop_node(**overrides):
    """A driver whose update() runs against one healthy observation of joint ``joint``."""
    def vector():
        return types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    driver.Twist = lambda: types.SimpleNamespace(linear=vector(), angular=vector())
    driver.ARM_JOINTS = ("joint",)
    driver.joint_positions = lambda *_: {"joint": 0.0}
    driver.integrate_pose = lambda pose, _velocity, _dt: pose
    node = make_node(
        command=driver.Twist(),
        command_stamp=_Stamp(),
        last_fresh=_Stamp(),
        arm_zero_positions={},
        arm_directions={},
        arm_positions={"joint": 0.0},
        pose=(0.0, 0.0, 0.0),
        xy_scale=1.0, yaw_scale=1.0, max_linear=1.0, max_angular=1.0,
        **overrides,
    )
    node.sent = []
    node.heartbeats = []
    node.get_clock = lambda: types.SimpleNamespace(now=_Stamp)
    node.robot = types.SimpleNamespace(
        get_observation=lambda: {"joint.pos": 0.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0},
        send_action=node.sent.append,
    )
    node.odom_samples = types.SimpleNamespace(
        accept=lambda *_: None, reset=lambda: None, discontinuity=None
    )
    node.publish_state = lambda *_: None
    node.publish_safety = lambda *args, **_: node.heartbeats.append(args)
    return node


def test_arm_permission_withdrawn_at_the_final_check_holds_the_arm():
    # The active trajectory's setpoint is well away from the measured position.
    driver.sample_trajectory = lambda *_: ({"joint": 0.5}, {}, {})
    driver.action_positions = lambda names, values, *_: dict(zip(names, values))
    canceled = []
    node = control_loop_node(
        armed=True,
        trajectory={
            "start": time.monotonic(),
            "done": threading.Event(),
            "names": ("joint",),
            "start_positions": {"joint": 0.0},
            "points": [types.SimpleNamespace(time=10.0, positions={"joint": 0.5})],
            "path_tolerances": {"joint": 1.0},
            "goal_tolerances": {"joint": 0.01},
            "goal_time_tolerance": 1.0,
        },
    )
    grant_fresh_base_permission(node)
    # The withdrawal lands after this cycle's lease check but before the final
    # armed/permission check under action_lock.
    node.enforce_permission_leases = lambda: False
    node.cancel_trajectory = canceled.append

    node.update()

    assert canceled == ["arm safety permission withdrawn"]
    assert node.sent == [{"joint.pos": 0.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}]


@pytest.mark.parametrize("armed", [False, True])
def test_control_loop_keeps_its_heartbeat_while_a_torque_transition_is_in_flight(armed):
    node = control_loop_node(armed=armed, stop_pending=True)
    grant_fresh_arm_permission(node)
    grant_fresh_base_permission(node)
    node.action_lock.acquire()  # an arm/disarm RPC that has not answered yet
    try:
        worker = threading.Thread(target=node.update)
        worker.start()
        worker.join(1.0)
        assert not worker.is_alive(), "update() waited for the torque transition"
    finally:
        node.action_lock.release()

    assert node.sent == []
    assert node.heartbeats == [()]
    assert node.stop_pending is True
    assert node._healthy_telemetry_at is not None

    node.last_observation_token = None  # the next packet from the host
    node.update()  # once the transition is done, the loop sends again
    assert len(node.sent) == 1


def test_host_torque_readback_is_not_enforced_during_a_transition():
    node = make_node(armed=False)
    node.robot = types.SimpleNamespace(observation_torque_enabled=True)
    disarms = []
    node.set_disarmed = lambda state, **_: disarms.append(state)
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)

    node.action_lock.acquire()  # an arm transaction has enabled torque but not committed
    try:
        assert node.enforce_reported_torque_state() is False
    finally:
        node.action_lock.release()
    assert disarms == []

    assert node.enforce_reported_torque_state() is True
    assert disarms == ["DISARMED"]


def test_automatic_arm_tick_acts_only_on_recent_healthy_telemetry():
    class Time:
        def __init__(self, ns):
            self.nanoseconds = ns

        def __sub__(self, other):
            return Time(self.nanoseconds - other.nanoseconds)

    current = {"ns": 0}
    attempts = []
    node = make_node(auto_arm_pending=True)
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Time(current["ns"]))
    node.arm_after_startup_telemetry = lambda: attempts.append(True) or True

    assert node.auto_arm_tick() is False  # no telemetry has passed the checks yet
    node._healthy_telemetry_at = Time(0)
    current["ns"] = 2_000_000_000
    assert node.auto_arm_tick() is False  # older than link_timeout
    current["ns"] = 500_000_000
    assert node.auto_arm_tick() is True
    assert attempts == [True]

    node.auto_arm_pending = False
    node.action_lock.acquire()  # an idle tick must not even contend for the lock
    try:
        assert node.auto_arm_tick() is False
    finally:
        node.action_lock.release()
    assert attempts == [True]


def test_torque_transitions_run_outside_the_control_loop_callback_group():
    source = " ".join(_SOURCE.split())
    for registration in (
        'Trigger, "safety/arm", self.arm,',
        'Trigger, "safety/disarm", self.disarm,',
        "0.1, self.auto_arm_tick,",
    ):
        assert f"{registration} callback_group=self.transition_callback_group" in source
    # update() stays in the node's default group, apart from every torque RPC.
    assert "self.create_timer(0.05, self.update)" in source


def test_a_failed_command_send_is_a_recorded_link_loss():
    node = control_loop_node(armed=True)
    grant_fresh_arm_permission(node)
    grant_fresh_base_permission(node)
    node.robot.send_action = lambda _action: (_ for _ in ()).throw(RuntimeError("socket closed"))
    logs, resets, disarms = [], [], []
    node.get_logger = lambda: types.SimpleNamespace(error=logs.append)
    node.odom_samples.reset = lambda: resets.append(True)
    node.set_disarmed = lambda state, **_: disarms.append(state)

    node.update()

    assert node.link_lost is True
    assert logs == ["LeKiwi command failed: socket closed"]
    assert resets == [True]
    assert disarms == ["LINK_LOST"]


def test_manual_arm_measures_telemetry_age_after_waiting_for_a_transition():
    node = make_node(last_fresh=_Stamp(), last_observation={"complete": True})
    grant_fresh_arm_permission(node)
    node.set_servo_torque = lambda _enabled: True
    node.publish_safety = lambda _state=None, **_: None
    driver.Twist = object
    measured_under_lock = []

    def now():
        measured_under_lock.append(node.action_lock.locked())
        return _Stamp()

    node.get_clock = lambda: types.SimpleNamespace(now=now)

    response = node.arm(None, types.SimpleNamespace())

    assert response.success is True
    assert measured_under_lock and all(measured_under_lock)


class _Result:
    SUCCESSFUL, INVALID_GOAL, OLD_HEADER_TIMESTAMP = 0, -1, -6

    def __init__(self, error_code, error_string=""):
        self.error_code, self.error_string = error_code, error_string


@pytest.mark.parametrize("offset_ns, expected", [
    (-1_000_000_000, _Result.OLD_HEADER_TIMESTAMP),
    (60_000_000_000, _Result.INVALID_GOAL),
])
def test_trajectory_header_stamps_must_start_close_to_now(offset_ns, expected):
    now_ns = 100_000_000_000
    driver.FollowJointTrajectory = types.SimpleNamespace(Result=_Result)
    driver.trajectory_rows = lambda _trajectory: []
    driver.stamp_nanoseconds = lambda stamp: stamp
    node = make_node(armed=True)
    node.requested_tolerances = lambda *_: ({}, {}, 1.0)
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=now_ns)
    )
    aborted = []
    goal = types.SimpleNamespace(
        request=types.SimpleNamespace(trajectory=types.SimpleNamespace(
            joint_names=["joint"], header=types.SimpleNamespace(stamp=now_ns + offset_ns),
        )),
        abort=lambda: aborted.append(True),
    )

    result = node.execute_trajectory(goal)

    assert aborted == [True]
    assert result.error_code == expected
    assert node.trajectory is None


def _disarmable(node):
    driver.Twist = object
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.cancel_trajectory = lambda _outcome: None
    node.get_logger = lambda: types.SimpleNamespace(warn=lambda *_: None, error=lambda *_: None)
    node.states = []
    node.publish_safety = lambda state=None, **_: node.states.append(state)
    return node


def test_control_loop_disarm_never_waits_for_a_torque_transaction_and_defers_the_strict_cut():
    node = _disarmable(make_node(armed=True))
    cuts = []
    node.set_servo_torque = lambda enabled: cuts.append(enabled) or True
    node.action_lock.acquire()  # an arm/disarm RPC is in flight
    finished = threading.Event()
    worker = threading.Thread(
        target=lambda: (node.set_disarmed("LINK_LOST", defer_cut=True), finished.set())
    )
    worker.start()
    try:
        assert finished.wait(1), "the control loop blocked behind the torque transaction"
        assert node.armed is False and node.stop_pending is True
        assert node.states == ["LINK_LOST"] and node._deferred_cut is True
        assert cuts == [], "the control loop must not run the torque RPC"
    finally:
        node.action_lock.release()
        worker.join(1)

    node.auto_arm_tick()
    assert cuts == [False] and node._deferred_cut is False and node.torque_fault is False


def test_a_deferred_cut_that_fails_latches_the_fault_before_any_arm():
    node = _disarmable(make_node(armed=True))
    node.set_servo_torque = lambda enabled: False
    node.set_disarmed("DISARMED", defer_cut=True)
    grant_fresh_arm_permission(node)

    class Now:
        def __sub__(self, _other):
            return types.SimpleNamespace(nanoseconds=0)

    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Now())
    node.last_fresh = Now()

    response = node.arm(None, types.SimpleNamespace())
    assert response.success is False and "fault-latched" in response.message
    assert node.states[-1] == "TORQUE_FAULT"


def test_default_mode_control_loop_disarm_defers_no_cut():
    node = _disarmable(make_node(armed=True, disarm_on_failure=False))
    node.set_servo_torque = lambda _enabled: pytest.fail("default mode keeps torque on")
    node.set_disarmed("LINK_LOST", defer_cut=True)
    assert node.armed is False and node._deferred_cut is False
    node.auto_arm_tick()


def test_a_disarm_during_an_in_flight_enable_wins_and_armed_is_never_published():
    node = _disarmable(make_node())
    grant_fresh_arm_permission(node)
    requests = []

    def enable_while_the_loop_disarms(enabled):
        requests.append(enabled)
        if enabled:
            node.set_disarmed("LINK_LOST", defer_cut=True)
        return True

    node.set_servo_torque = enable_while_the_loop_disarms
    epoch = node._disarm_epoch
    outcome, _cut = node._enable_torque_and_arm(
        node._arm_permission_is_current, operator=False, disarm_epoch=epoch
    )
    assert outcome == "unsafe" and node.armed is False
    assert "ARMED" not in node.states
    assert requests == [True, False], "the rolled-back enable is followed by the strict cut"



class _NodeBase:
    def destroy_node(self):
        self.calls.append("node")


class _ShutdownDriver(driver.LeKiwiDriver, _NodeBase):
    pass


def make_shutdown_node(monkeypatch, liveness, disarm_error):
    node = make_node()
    node.__class__ = _ShutdownDriver
    node.calls = []
    monkeypatch.setattr(
        driver, "rclpy", types.SimpleNamespace(ok=lambda context=None: next(liveness)),
        raising=False,
    )

    def failing_disarm(state, publish=True):
        raise RuntimeError(disarm_error)

    node.set_disarmed = failing_disarm
    node.trajectory_server = types.SimpleNamespace(destroy=lambda: node.calls.append("server"))
    node.robot = types.SimpleNamespace(disconnect=lambda: node.calls.append("robot"))
    return node


def test_shutdown_tolerates_the_context_dying_during_the_disarm_publish(monkeypatch):
    # rclpy.ok() said the context was alive, then SIGINT invalidated it mid-publish.
    node = make_shutdown_node(monkeypatch, iter([True, False]), "publisher's context is invalid")
    node.destroy_node()
    assert node.calls == ["server", "robot", "node"]


def test_shutdown_still_raises_a_disarm_failure_while_ros_is_up(monkeypatch):
    node = make_shutdown_node(monkeypatch, iter([True, True]), "real failure")
    with pytest.raises(RuntimeError, match="real failure"):
        node.destroy_node()
    assert node.calls == ["server", "robot"]
