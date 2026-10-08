"""Driver boundary checks with a loopback host for native ROS construction."""

import asyncio
import dataclasses
import math
import pathlib
import threading
import time
import types

import pytest
import yaml

from lekiwi_rmf import driver
from lekiwi_rmf.motion_guards import lease_is_fresh


@pytest.fixture(autouse=True)
def restore_driver_imports(monkeypatch):
    # Existing boundary fakes patch imported messages/clients. Keep each check isolated.
    for name, value in vars(driver).copy().items():
        if not name.startswith("__"):
            monkeypatch.setattr(driver, name, value)


@pytest.fixture
def connected_driver():
    from lekiwi_rmf.fake_host import FakeLeKiwiHost
    import rclpy

    host = FakeLeKiwiHost()
    host.start(period_s=0.02)
    rclpy.init(args=[
        "--ros-args", "-p", f"remote_command_port:={host.command_endpoint_port}",
        "-p", f"remote_observation_port:={host.observation_endpoint_port}",
        "-p", f"torque_control_port:={host.torque_endpoint_port}",
        "-p", "arm_calibration_file:=/test/missing-calibration.json",
        "-p", f"nav2_params_file:={pathlib.Path(__file__).parents[1] / 'config/nav2_params.yaml'}",
        "-p", "permission_timeout:=0.5",
    ])
    node = None
    try:
        node = driver.LeKiwiDriver()
        yield node
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
            host.close()


def make_node(**overrides):
    """A driver in the state ``__init__`` leaves it, without ROS or a motor host.

    The tests below written before staying armed became the default cover the opt-in
    strict mode (disarm, cut torque and wait for an operator on every failure), so that
    is the baseline here. The default has its own tests at the end.
    """
    node = driver.LeKiwiDriver.__new__(driver.LeKiwiDriver)
    fields = {field.name for field in dataclasses.fields(driver.DriverSettings)}
    settings = driver.DriverSettings(**{
        "max_linear": 1.0, "max_angular": 1.0, "disarm_on_failure": True,
        "auto_arm_on_startup": False, "permission_timeout": 10.0,
        **{name: value for name, value in overrides.items() if name in fields},
    })
    node._init_state(
        settings, robot=None, torque=None, arm_calibration_file="/test/missing-calibration.json",
        initial_pose=(0.0, 0.0, 0.0), now=None,
    )
    node._context = None
    for name, value in overrides.items():
        if name not in fields:
            setattr(node, name, value)
    return node


def fake_client(**attributes):
    """The LeKiwiZmqClient surface the driver reads, after one accepted packet."""
    client = types.SimpleNamespace(
        observation_token=("host", "test-session", 1),
        missing_state_keys=(),
        observation_session_changed=False,
        observation_torque_enabled=False,
        observation_motor_health=(),
        observation_odometry={"pose": [0.0, 0.0, 0.0], "scales": [1.0, 1.0], "stamp_ns": 1},
        arm_trajectory_status=None,
        get_observation=lambda: {"joint.pos": 0.0, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0},
        send_action=lambda _action, **_lease: None,
        disconnect=lambda: None,
    )
    vars(client).update(attributes)
    return client


def grant_fresh_arm_permission(node, permitted=True):
    node.arm_permission = driver.Lease(10_000_000_000, permitted, time.monotonic_ns())


def grant_fresh_base_permission(node, permitted=True):
    node.base_permission = driver.Lease(10_000_000_000, permitted, time.monotonic_ns())


def test_only_a_newly_accepted_packet_is_fresh():
    node = make_node(robot=fake_client())
    cached = {"arm_shoulder_pan.pos": 12.0}
    assert node.observation_is_fresh(cached)
    # A stationary robot repeats identical values; only the packet identity counts.
    assert not node.observation_is_fresh(cached)
    node.robot.observation_token = ("host", "test-session", 2)
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


def test_a_lease_grants_only_its_current_value_until_it_expires():
    lease = driver.Lease(100)
    assert not lease.current(0)
    lease.grant(True, 1_000)
    assert lease.current(1_100) and not lease.current(1_101) and not lease.current(999)
    lease.grant(False, 1_050)
    assert not lease.current(1_060)


def test_arm_permission_lease_expiry_disarms_and_base_expiry_zeros_command():
    driver.Twist = object
    node = make_node()
    node.state_lock = threading.Lock()
    node.armed = True
    node.arm_permission = driver.Lease(100, True, 1_000)
    node.base_permission = driver.Lease(100, True, 1_000)
    node._arm_permission_expired = False
    node.command = object()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)
    node.cancel_trajectory = lambda _outcome: None
    disarms = []
    node.set_disarmed = lambda state, **_: disarms.append(state)

    assert node.enforce_permission_leases(now_monotonic_ns=1_101)

    assert node.arm_permission.value is False
    assert node.base_permission.value is False
    assert node.command is not None
    assert disarms == ["DISARMED"]


def test_explicit_arm_rejects_base_permission_without_arm_workspace_clear():
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
    node.state_lock = threading.Lock()
    node.action_lock = threading.Lock()
    node.torque_fault = False
    torque_requests = []
    node.set_servo_torque = lambda enabled: torque_requests.append(enabled) or enabled
    states = []
    node.publish_safety = lambda state=None, **_: states.append(state)
    response = types.SimpleNamespace()

    node.arm(None, response)

    assert response.success is False
    assert "arm permission" in response.message
    assert node.armed is False
    assert torque_requests == []
    assert states == []


def test_arm_permission_withdrawal_keeps_torque_when_base_lease_is_current():
    driver.Twist = object
    node = make_node()
    node.state_lock = threading.Lock()
    node.armed = True
    node.arm_permission.value = True
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
    node.state_lock = threading.Lock()
    node.armed = True
    node.arm_permission = driver.Lease(100, True, 1_050)
    node.base_permission = driver.Lease(100, True, 1_000)
    node._arm_permission_expired = False
    node.command = object()
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: object())
    disarms = []
    node.set_disarmed = lambda state, **_: disarms.append(state)

    assert not node.enforce_permission_leases(now_monotonic_ns=1_101)

    assert node.base_permission.value is False
    assert node.arm_permission.value is True
    assert disarms == []


def test_host_session_restart_forces_disarm():
    node = make_node()
    node.robot = fake_client(observation_session_changed=True)
    node.set_disarmed = lambda state, **_: setattr(node, "disarmed_as", state)
    node.get_logger = lambda: types.SimpleNamespace(error=lambda *_: None)

    assert node.handle_host_session_change() is True
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
    logs, disarms = [], []
    node.get_logger = lambda: types.SimpleNamespace(error=logs.append)
    node.set_disarmed = lambda state, **_: disarms.append(state)

    node.record_link_loss("telemetry failed")
    node.record_link_loss("telemetry failed again")

    assert logs == ["telemetry failed"]
    assert disarms == ["LINK_LOST"]


@pytest.mark.parametrize("field, value, parameter", [
    ("xy_scale", 0.0, "xy_velocity_scale"),
    ("trajectory_path_tolerance", 0.0, "trajectory_path_tolerance"),
    ("permission_timeout", math.inf, "permission_timeout"),
    ("cmd_vel_topic", " ", "cmd_vel_topic"),
])
def test_unsafe_or_undefined_settings_are_rejected(field, value, parameter):
    with pytest.raises(ValueError, match=parameter):
        driver.DriverSettings(max_linear=1.0, max_angular=1.0, **{"permission_timeout": 0.5, field: value})


def test_gripper_trajectory_uses_its_tighter_completion_tolerance():
    node = make_node()
    assert node.trajectory_tolerance == yaml.safe_load(
        (pathlib.Path(__file__).parents[1] / "config" / "safety_production.yaml").read_text()
    )["safety_supervisor"]["ros__parameters"]["stow_tolerance"]
    goal = types.SimpleNamespace(
        component_path_tolerance=[], component_goal_tolerance=[],
        path_tolerance=[], goal_tolerance=[],
        goal_time_tolerance=types.SimpleNamespace(sec=0, nanosec=0),
    )
    _, tolerances, _ = node.arm_trajectories.requested_tolerances(
        goal, ("arm_shoulder_lift", "arm_gripper")
    )
    assert tolerances == {"arm_shoulder_lift": 0.02, "arm_gripper": 0.005}


def test_driver_requires_the_tracked_speed_limits_and_permission_lease():
    import rclpy

    rclpy.init(args=["--ros-args", "-p", "permission_timeout:=0.5"])
    try:
        with pytest.raises(ValueError, match="nav2_params_file"):
            driver.LeKiwiDriver()
    finally:
        rclpy.try_shutdown()


def test_guarded_command_topic_is_the_default(connected_driver):
    assert connected_driver.get_parameter("cmd_vel_topic").value == "/cmd_vel_safe"
    assert connected_driver.cmd_vel_topic == "/cmd_vel_safe"


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
    from builtin_interfaces.msg import Time

    odometry = driver.odometry_message(
        Time(sec=3), (1.0, 2.0, math.pi / 2), (0.1, 0.0, 0.2), (0.05, 0.10), (0.10, 0.20),
    )
    assert (odometry.header.frame_id, odometry.child_frame_id) == ("odom", "base_footprint")
    assert odometry.pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert odometry.pose.covariance[0] == pytest.approx(0.05 ** 2)
    assert odometry.pose.covariance[35] == pytest.approx(0.10 ** 2)
    assert odometry.twist.covariance[0] == pytest.approx(0.10 ** 2)
    assert odometry.twist.covariance[35] == pytest.approx(0.20 ** 2)
    assert odometry.pose.covariance[14] == 1e6
    transform = driver.odometry_transform(odometry)
    assert transform.header.stamp.sec == 3
    assert (transform.transform.translation.x, transform.transform.translation.y) == (1.0, 2.0)
    assert transform.transform.rotation.w == odometry.pose.pose.orientation.w


def test_published_state_reports_calibrated_and_raw_joints():
    from builtin_interfaces.msg import Time

    published = {name: [] for name in ("odom", "tf", "joints", "raw")}
    node = make_node(publish_odom_tf=True)
    node.odom_pub = types.SimpleNamespace(publish=published["odom"].append)
    node.tf = types.SimpleNamespace(sendTransform=published["tf"].append)
    node.joint_pub = types.SimpleNamespace(publish=published["joints"].append)
    node.raw_joint_pub = types.SimpleNamespace(publish=published["raw"].append)
    node.arm_positions["arm_shoulder_pan"] = 0.25

    node.publish_state(Time(sec=5), {"arm_shoulder_pan.pos": 90.0}, (0.0, 0.0, 0.0))

    assert all(len(messages) == 1 for messages in published.values())
    joints, raw = published["joints"][0], published["raw"][0]
    assert joints.position[joints.name.index("arm_shoulder_pan")] == 0.25
    assert raw.position[raw.name.index("arm_shoulder_pan")] == pytest.approx(math.pi / 2)
    assert raw.header.stamp.sec == 5


def test_validated_motor_health_is_published_as_diagnostics():
    from builtin_interfaces.msg import Time

    message = driver.diagnostics_message(Time(sec=7), (
        types.SimpleNamespace(
            name="motor_bus", level=0, message="OK",
            values=(("torque_enabled", "false"),),
        ),
    ))

    assert message.header.stamp.sec == 7
    assert message.status[0].name == "motor_bus"
    assert message.status[0].level == b"\x00"
    assert message.status[0].hardware_id == "lekiwi_servo_bus"
    assert message.status[0].values[0].key == "torque_enabled"


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
    node.arm_permission.value = True
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
    node._context = object()
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
    published = []
    node = make_node(xy_scale=1.0, yaw_scale=1.0)
    node.get_clock = lambda: type("Clock", (), {"now": lambda _: Stamp()})()
    node.robot = fake_client(get_observation=lambda: {
        "joint.pos": 0.0, "x.vel": 0.2, "y.vel": 0.0, "theta.vel": 0.0,
    })
    node.publish_motor_health = lambda _stamp: None
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
    node.publish_state = lambda *args: published.append(args)
    safety_refreshes = []
    node.publish_safety = lambda *args, **_: safety_refreshes.append(args)
    node.enforce_reported_torque_state = lambda: False

    state_lock_was_free = []
    def send_while_checking_lock(action):
        acquired = node.state_lock.acquire(blocking=False)
        state_lock_was_free.append(acquired)
        if acquired:
            node.state_lock.release()
        sent.append(action)
    node.robot.send_action = send_while_checking_lock

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
    node._arm_permission_is_current = lambda: True

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
    node = make_node(
        command_stamp=_Stamp(),
        last_fresh=_Stamp(),
        arm_zero_positions={},
        arm_directions={},
        xy_scale=1.0, yaw_scale=1.0,
        **overrides,
    )
    node.sent = []
    node._base_qualification = None
    node._base_test_radius = .20
    node.heartbeats = []
    node.get_clock = lambda: types.SimpleNamespace(now=_Stamp)
    node.robot = fake_client(send_action=node.sent.append, observation_torque_enabled=node.armed)
    node.publish_motor_health = lambda _stamp: None
    node.publish_state = lambda *_: None
    node.publish_safety = lambda *args, **_: node.heartbeats.append(args)
    return node


def test_idle_arm_hold_keeps_its_goal_when_feedback_sags():
    node = control_loop_node(armed=True)
    grant_fresh_arm_permission(node)
    grant_fresh_base_permission(node)
    observation = {"joint.pos": 10.0}

    node._send_armed_command(_Stamp(), observation, (0.0, 0.0, 0.0))
    observation["joint.pos"] = 7.0
    node._send_armed_command(_Stamp(), observation, (0.0, 0.0, 0.0))

    assert [action["joint.pos"] for action in node.sent] == [10.0, 10.0]


def test_arm_permission_withdrawn_at_the_final_check_holds_the_arm():
    canceled = []
    node = control_loop_node(armed=True, trajectory=driver.ArmGoal(host_id=1, host_session="s", start=0.0))
    leases = []
    node.robot.send_action = lambda action, **kwargs: (node.sent.append(action), leases.append(kwargs))
    grant_fresh_base_permission(node)
    # The withdrawal lands after this cycle's lease check but before the final
    # armed/permission check under action_lock.
    node.enforce_permission_leases = lambda: False
    node.cancel_trajectory = canceled.append

    node.update()

    assert canceled == ["arm safety permission withdrawn"]
    assert leases == [{"arm_goal_id": 1, "arm_permitted": False}]
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


def test_torque_transitions_run_outside_the_control_loop_callback_group(connected_driver):
    node = connected_driver
    for service in node.services:
        if service.srv_name in ("/safety/arm", "/safety/disarm"):
            assert service.callback_group is node.transition_callback_group
    callbacks = {timer.callback: timer.callback_group for timer in node.timers}
    assert callbacks[node.auto_arm_tick] is node.transition_callback_group
    assert callbacks[node.update] is node.default_callback_group
    assert node.default_callback_group is not node.transition_callback_group


def test_a_failed_command_send_is_a_recorded_link_loss():
    node = control_loop_node(armed=True)
    grant_fresh_arm_permission(node)
    grant_fresh_base_permission(node)
    node.robot.send_action = lambda _action: (_ for _ in ()).throw(RuntimeError("socket closed"))
    logs, disarms = [], []
    node.get_logger = lambda: types.SimpleNamespace(error=logs.append)
    node.set_disarmed = lambda state, **_: disarms.append(state)

    node.update()

    assert node.link_lost is True
    assert logs == ["LeKiwi command failed: socket closed"]
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
    node.arm_trajectories.requested_tolerances = lambda *_: ({}, {}, 1.0)
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

    result = asyncio.run(node.arm_trajectories.execute(goal))

    assert aborted == [True]
    assert result.error_code == expected
    assert node.trajectory is None


def test_goal_waiting_for_joint_recovery_starts_its_clock_after_wait(monkeypatch):
    clock = [10.0]
    allowed = [False]
    monkeypatch.setattr(driver.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(driver, "FollowJointTrajectory", types.SimpleNamespace(Result=_Result))
    monkeypatch.setattr(driver, "Twist", object)
    monkeypatch.setattr(driver, "threading", threading, raising=False)
    point = types.SimpleNamespace(time=1.0, positions={"joint": 0.5})
    monkeypatch.setattr(driver, "trajectory_rows", lambda *_: [point])
    monkeypatch.setattr(driver, "stamp_nanoseconds", lambda *_: 0)
    monkeypatch.setattr(driver, "prepare_trajectory", lambda _names, points, *_: points, raising=False)
    node = make_node(
        armed=True, arm_positions={"joint": 0.0},
        arm_zero_positions={"joint": 0.0}, arm_directions={"joint": 1.0},
        last_observation_token=("host", "test-session", 1),
        arm_hold_action={"joint.pos": 10.0},
    )
    node.arm_trajectories.requested_tolerances = lambda *_: ({}, {}, 1.0)
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=0)
    )
    node._arm_permission_is_current = lambda: allowed[0]
    node._hold_feedback_gap = lambda: True
    node.arm_trajectories.publish_feedback = lambda *_: None
    uploaded = []
    started = []

    def upload(command, **request):
        assert command == "trajectory_start"
        uploaded.append(request)
        started.append(node.trajectory.start)
        node.robot.arm_trajectory_status = {
            "id": request["trajectory"]["id"], "state": "succeeded",
            "elapsed": 1.0, "code": 0, "detail": "",
        }
        return {"trajectory": node.robot.arm_trajectory_status}

    node.robot = types.SimpleNamespace(arm_trajectory_status=None)
    node.torque = types.SimpleNamespace(trajectory_request=upload)
    monkeypatch.setattr(driver, "action_positions", lambda *_: {"joint": 20.0})

    async def yield_control(_delay):
        clock[0] += 0.5
        allowed[0] = True

    node.arm_trajectories._yield_for_control = yield_control
    succeeded = []
    goal = types.SimpleNamespace(
        request=types.SimpleNamespace(trajectory=types.SimpleNamespace(
            joint_names=["joint"], header=types.SimpleNamespace(stamp=0),
        )),
        is_cancel_requested=False,
        succeed=lambda: succeeded.append(True),
    )

    result = asyncio.run(node.arm_trajectories.execute(goal))

    assert result.error_code == _Result.SUCCESSFUL
    assert succeeded == [True]
    assert started == [pytest.approx(10.5)]
    assert len(uploaded) == 1 and uploaded[0]["session"] == "test-session"
    assert uploaded[0]["trajectory"]["delay"] == 0.0
    assert node.trajectory is None
    assert node.arm_hold_action == {"joint.pos": 20.0}


def test_local_goal_survives_telemetry_silence_beyond_link_timeout(monkeypatch):
    node = make_node(armed=True, disarm_on_failure=False)
    node.trajectory = driver.ArmGoal(host_id=1, host_session="s", start=0.0)
    node.get_logger = lambda: types.SimpleNamespace(warning=lambda *_: None, error=lambda *_: None)
    states, disarmed = [], []
    node.publish_safety = states.append
    node.set_disarmed = lambda *args, **kwargs: disarmed.append(args)
    monkeypatch.setattr(driver.time, "monotonic", lambda: 22.0)
    node.record_link_loss("No fresh LeKiwi telemetry for 12.0s; waiting for recovery")
    assert node.link_lost and node.armed
    assert node._hold_feedback_gap()
    assert not node.trajectory.done.is_set()
    assert states == ["LINK_LOST"] and not disarmed


def test_local_goal_waits_for_collision_check_after_motor_feedback_recovers(monkeypatch):
    node = make_node(armed=True, disarm_on_failure=False)
    node.arm_permission.value = True
    node.trajectory = driver.ArmGoal(host_id=1, host_session="s", start=0.0)
    node.get_logger = lambda: types.SimpleNamespace(warning=lambda *_: None, error=lambda *_: None)
    canceled, disarmed = [], []
    node.cancel_trajectory = canceled.append
    node.set_disarmed = disarmed.append
    monkeypatch.setattr(driver.time, "monotonic", lambda: 10.01)
    monkeypatch.setattr(driver, "Bool", types.SimpleNamespace, raising=False)

    node.on_arm_permission(types.SimpleNamespace(data=False))
    assert node.armed and not canceled and not disarmed
    assert not node._arm_permission_is_current()
    node.on_arm_permission(types.SimpleNamespace(data=True))
    assert node._arm_permission_is_current()
    node.on_arm_collision(types.SimpleNamespace(data=True))
    assert canceled == ["arm safety permission withdrawn"]
    assert disarmed == ["DISARMED"]
    node.on_arm_permission(types.SimpleNamespace(data=True))
    assert not node._arm_permission_is_current()


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



def make_shutdown_node(monkeypatch, liveness, disarm_error):
    node = make_node()
    node.calls = []
    monkeypatch.setattr(driver.Node, "destroy_node", lambda self: self.calls.append("node"))
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


def test_bounded_base_test_zeros_all_axes_at_boundary_without_cutting_torque():
    node = control_loop_node(armed=True, bounded_base_test=True)
    grant_fresh_arm_permission(node)
    grant_fresh_base_permission(node)
    node._base_test_center = (0.0, 0.0)
    node.pose = (0.21, 0.0, 0.0)
    node.command.linear.x = node.command.linear.y = 0.02
    node.command.angular.z = 0.15
    node._send_armed_command(_Stamp(), {'joint.pos': 10.0}, (0.0, 0.0, 0.0))
    assert node.armed
    assert all(node.sent[-1][axis] == 0.0 for axis in ('x.vel', 'y.vel', 'theta.vel'))
    assert node.sent[-1]['joint.pos'] == 10.0


def test_qualification_caps_coupled_motion_and_retains_the_folded_hold():
    node=control_loop_node(armed=True,bounded_base_test=True,max_linear=.3,max_angular=.6)
    grant_fresh_arm_permission(node)
    grant_fresh_base_permission(node)
    node._base_test_center=(0.,0.)
    node._base_test_radius=.45
    node._base_qualification={'return_linear_speed_m_s':.1,'return_angular_speed_rad_s':.06}
    node.pose=(0.,0.,0.)
    node.command.linear.x=.3
    node.command.angular.z=.6
    node._send_armed_command(_Stamp(),{'joint.pos':10.},(0.,0.,0.))
    assert node.sent[-1]['x.vel']==.1
    assert node.sent[-1]['theta.vel']==pytest.approx(math.degrees(.06))
    assert node.sent[-1]['joint.pos']==10.
    node.command.angular.z=0.
    node._send_armed_command(_Stamp(),{'joint.pos':10.},(0.,0.,0.))
    assert node.sent[-1]['x.vel']==.3
    node.pose=(.451,0.,0.)
    node._send_armed_command(_Stamp(),{'joint.pos':10.},(0.,0.,0.))
    assert node.sent[-1]['x.vel']==node.sent[-1]['y.vel']==node.sent[-1]['theta.vel']==0.
    assert node.armed and node.sent[-1]['joint.pos']==10.
