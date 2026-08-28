"""Pure checks for the driver's stale-telemetry detector."""

import ast
import math
import pathlib
import threading
import time
import types

import pytest

_SOURCE = (pathlib.Path(__file__).parents[1] / "lekiwi_rmf" / "driver.py").read_text()
_TREE = ast.parse(_SOURCE)
_NODE = next(node for node in _TREE.body if getattr(node, "name", None) == "LeKiwiDriver")
_NODE.bases = []
_NODE.body = [
    item for item in _NODE.body
    if getattr(item, "name", None) in (
        "arm", "disarm", "clamp_planar", "observation_is_fresh", "observation_is_valid",
        "handle_host_session_change",
        "enforce_reported_torque_state",
        "arm_after_startup_telemetry", "on_command", "publish_safety", "publish_state", "publish_motor_health",
        "set_disarmed", "set_servo_torque", "trajectory_time",
        "_permission_is_fresh", "_permission_is_current",
        "_capability_permission_is_current", "enforce_permission_leases",
        "on_base_permission", "on_arm_permission",
        "twist_is_finite", "update", "validate_motion_parameters",
    )
]
driver = types.ModuleType("driver_under_test")
exec(compile(ast.Module(body=[_NODE], type_ignores=[]), "driver.py", "exec"), driver.__dict__)
driver.math = math
driver.time = time


def grant_fresh_arm_permission(node, permitted=True):
    node.arm_motion_permitted = permitted
    node._arm_permission_received_at_ns = time.monotonic_ns()
    node.permission_timeout_ns = 10_000_000_000


def grant_fresh_base_permission(node, permitted=True):
    node.base_motion_permitted = permitted
    node._base_permission_received_at_ns = time.monotonic_ns()
    node.permission_timeout_ns = 10_000_000_000


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


def test_client_without_an_accepted_packet_is_not_fresh():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.robot = types.SimpleNamespace(observation_token=None, observation_sequence=0)
    node.last_observation_token = None

    assert not node.observation_is_fresh({"x.vel": 0.0})


def test_permission_lease_uses_receive_monotonic_time_and_expires():
    assert driver.LeKiwiDriver._permission_is_fresh(1_000, 100, 1_100)
    assert not driver.LeKiwiDriver._permission_is_fresh(1_000, 100, 1_101)
    assert not driver.LeKiwiDriver._permission_is_fresh(1_000, 100, 999)
    assert not driver.LeKiwiDriver._permission_is_fresh(None, 100, 1_000)


def test_arm_permission_lease_expiry_disarms_and_base_expiry_zeros_command():
    driver.Twist = object
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.permission_timeout_ns = 100
    node.state_lock = threading.Lock()
    node.armed = True
    node.arm_motion_permitted = True
    node.base_motion_permitted = True
    node._arm_permission_received_at_ns = 1_000
    node._base_permission_received_at_ns = 1_000
    node._arm_permission_expired = False
    node._base_permission_expired = False
    node.command = object()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)
    node.cancel_trajectory = lambda _outcome: None
    disarms = []
    node.set_disarmed = lambda state: disarms.append(state)

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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
    node.publish_safety = states.append
    response = types.SimpleNamespace()

    node.arm(None, response)

    assert response.success is True
    assert node.armed is True
    assert states == ["ARMED"]


def test_arm_permission_withdrawal_keeps_torque_when_base_lease_is_current():
    driver.Twist = object
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
    node.set_disarmed = lambda state: disarms.append(state)

    node.on_arm_permission(types.SimpleNamespace(data=False))
    # Repeated false heartbeats are lease refreshes, not repeated withdrawal events.
    node.on_arm_permission(types.SimpleNamespace(data=False))

    assert cancellations == ["arm safety permission withdrawn"]
    assert disarms == []
    assert len(messages) == 1


def test_base_permission_lease_expiry_does_not_require_arm_disarm():
    driver.Twist = object
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.permission_timeout_ns = 100
    node.state_lock = threading.Lock()
    node.armed = True
    node.arm_motion_permitted = True
    node.base_motion_permitted = True
    node._arm_permission_received_at_ns = 1_050
    node._base_permission_received_at_ns = 1_000
    node._arm_permission_expired = False
    node._base_permission_expired = False
    node.command = object()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    disarms = []
    node.set_disarmed = lambda state: disarms.append(state)

    assert not node.enforce_permission_leases(now_monotonic_ns=1_101)

    assert node.base_motion_permitted is False
    assert node.arm_motion_permitted is True
    assert disarms == []


def test_host_session_restart_forces_disarm_and_odometry_reset():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.robot = types.SimpleNamespace(observation_session_changed=True)
    reset = []
    node.odom_samples = types.SimpleNamespace(reset=lambda: reset.append(True))
    node.set_disarmed = lambda state: setattr(node, "disarmed_as", state)
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)

    assert node.handle_host_session_change() is True
    assert reset == [True]
    assert node.disarmed_as == "DISARMED"


def test_authenticated_host_torque_cut_cannot_leave_driver_logically_armed():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.robot = types.SimpleNamespace(observation_torque_enabled=False)
    node.state_lock = threading.Lock()
    node.armed = True
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)
    disarms = []
    node.set_disarmed = lambda state: disarms.append(state)

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


def test_invalid_motion_scale_is_rejected():
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.auto_arm_pending = True
    node.link_lost = False
    node.armed = False
    grant_fresh_arm_permission(node, False)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.publish_safety = lambda state: setattr(node, "safety", state)
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

    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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

    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Now())
    node.last_fresh = object()
    node.link_timeout = 1.0
    node.link_lost = False
    node.last_observation = {"complete": True}
    grant_fresh_arm_permission(node)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    node.publish_safety = lambda _state: None
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

    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
    node.publish_safety = states.append
    response = types.SimpleNamespace()

    node.arm(None, response)

    assert response.success is False
    assert "not confirmed" in response.message
    assert requests == [True, False]
    assert node.torque_fault is True
    assert states == ["TORQUE_FAULT"]


def test_unconfirmed_startup_enable_cannot_leave_logical_arm_state_set():
    driver.Twist = object
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.auto_arm_pending = True
    node.link_lost = False
    node.armed = False
    grant_fresh_arm_permission(node)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    requests = []
    node.set_servo_torque = lambda enabled: requests.append(enabled) or not enabled
    node.publish_safety = lambda state: setattr(node, "safety", state)
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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.pose = (1.0, 2.0, 0.0)
    node.odom_xy_stddev, node.odom_yaw_stddev = 0.05, 0.10
    node.twist_xy_stddev, node.twist_yaw_stddev = 0.10, 0.20
    node.odom_pub = types.SimpleNamespace(publish=odometry.append)
    node.tf = types.SimpleNamespace(sendTransform=transform.append)
    node.publish_odom_tf = True
    node.arm_positions = {"joint": 0.0}
    node.joint_pub = types.SimpleNamespace(publish=joints.append)

    node.publish_state(object(), {}, (0.0, 0.0, 0.0))

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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.armed = True
    node.torque_fault = False
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
    node.action_lock = threading.Lock()
    node.armed = True
    node.torque_fault = False
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: object()})()
    node.cancel_trajectory = lambda outcome: setattr(node, "outcome", outcome)
    node.publish_safety = lambda state: setattr(node, "safety", state)
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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
    node.publish_safety = states.append
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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.context = object()
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_lock = threading.Lock()
    node.armed = False
    node.torque_fault = False
    node.auto_arm_pending = False
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.cancel_trajectory = lambda _outcome: None
    node.publish_safety = lambda _state: pytest.fail("shutdown published into a dead context")
    node.torque = types.SimpleNamespace(
        set_enabled=lambda _enabled: (_ for _ in ()).throw(RuntimeError("transport died"))
    )

    assert node.set_disarmed("DISARMED", publish=False) is False
    assert node.torque_fault is True


def test_disarm_keeps_state_observable_while_serializing_physical_actions():
    driver.Twist = object
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.armed = True
    node.torque_fault = False
    node.auto_arm_pending = True
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: object()})()
    node.cancel_trajectory = lambda _outcome: None
    node.publish_safety = lambda _state: None
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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
    node.publish_safety = lambda *args: safety_refreshes.append(args)
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
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
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
