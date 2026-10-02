"""Pure checks for the execution-time MoveIt collision gate."""

import math
import types

from sensor_msgs.msg import JointState

from lekiwi_rmf.arm_workspace_monitor import (
    ArmWorkspaceMonitor, ArmWorkspaceState, complete_joint_snapshot,
)


SECOND = 1_000_000_000


def _joints(names=("joint_a", "joint_b"), positions=(0.1, -0.2)):
    message = JointState()
    message.header.stamp.sec = 1
    message.name = list(names)
    message.position = list(positions)
    return message


def test_joint_snapshot_requires_one_complete_finite_stamped_message():
    snapshot = complete_joint_snapshot(_joints(), ("joint_b", "joint_a"))
    assert snapshot is not None
    assert list(snapshot.name) == ["joint_b", "joint_a"]
    assert list(snapshot.position) == [-0.2, 0.1]

    assert complete_joint_snapshot(_joints(("joint_a",), (0.1,)), ("joint_a", "joint_b")) is None
    assert complete_joint_snapshot(_joints(("joint_a", "joint_a"), (0.1, 0.2)), ("joint_a",)) is None
    assert complete_joint_snapshot(_joints(("joint_a",), (0.1, 0.2)), ("joint_a",)) is None
    assert complete_joint_snapshot(_joints(positions=(0.1, math.nan)), ("joint_a", "joint_b")) is None
    stampless = _joints()
    stampless.header.stamp.sec = 0
    assert complete_joint_snapshot(stampless, ("joint_a", "joint_b")) is None


def test_gate_requires_fresh_joint_perception_and_collision_free_service_result():
    state = ArmWorkspaceState(
        joint_received_ns=SECOND,
        perception_received_ns=SECOND,
        checked_ns=SECOND,
        collision_free=True,
    )
    assert state.decision(SECOND, SECOND, SECOND, SECOND)[0]

    state.collision_free = False
    state.detail = "MoveIt reports collision"
    assert state.decision(SECOND, SECOND, SECOND, SECOND) == (
        False, "MoveIt reports collision"
    )
    state.collision_free = True
    assert not state.decision(SECOND * 3, SECOND, SECOND, SECOND)[0]


def test_gate_rejects_future_receive_timestamps():
    state = ArmWorkspaceState(
        joint_received_ns=SECOND + 1,
        perception_received_ns=SECOND,
        checked_ns=SECOND,
        collision_free=True,
    )
    clear, detail = state.decision(SECOND, SECOND, SECOND, SECOND)
    assert not clear
    assert "joint state stale" == detail


def test_joint_capture_clock_skew_is_bounded_and_transport_age_is_preserved():
    now = [SECOND]
    node = types.SimpleNamespace(_joint_names=('joint_a','joint_b'),
        _joint_timeout_ns=450_000_000, _state=ArmWorkspaceState(),
        _monotonic_ns=lambda:10*SECOND,
        get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=now[0])))
    message = _joints()
    for ahead in (7_000_000,50_000_000):
        message.header.stamp.nanosec = ahead
        ArmWorkspaceMonitor._on_joint_state(node,message)
        assert node._joint_snapshot is not None
        assert node._state.joint_received_ns==10*SECOND
    message.header.stamp.nanosec = 50_000_001
    ArmWorkspaceMonitor._on_joint_state(node,message)
    assert node._joint_snapshot is None and node._state.joint_received_ns is None
    message.header.stamp.nanosec = 0
    now[0] = SECOND+100_000_000
    ArmWorkspaceMonitor._on_joint_state(node,message)
    assert node._state.joint_received_ns==10*SECOND-100_000_000
    now[0] = SECOND+450_000_001
    ArmWorkspaceMonitor._on_joint_state(node,message)
    assert node._joint_snapshot is None


def test_gate_blocks_until_the_perception_cloud_arrives_and_when_it_goes_stale():
    state = ArmWorkspaceState(
        joint_received_ns=SECOND,
        checked_ns=SECOND,
        collision_free=True,
    )
    assert state.decision(SECOND, SECOND, SECOND, SECOND) == (
        False, "perception cloud missing"
    )
    state.perception_received_ns = SECOND
    assert state.decision(SECOND, SECOND, SECOND, SECOND)[0]
    assert state.decision(SECOND * 2 + 1, SECOND * 3, SECOND, SECOND * 3) == (
        False, "perception cloud stale"
    )
