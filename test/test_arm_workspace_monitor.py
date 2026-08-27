"""Pure checks for the execution-time MoveIt collision gate."""

import math

from sensor_msgs.msg import JointState

from lekiwi_rmf.arm_workspace_monitor import ArmWorkspaceState, complete_joint_snapshot


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


def test_gate_requires_fresh_joint_scene_and_collision_free_service_result():
    state = ArmWorkspaceState(
        joint_received_ns=SECOND,
        scene_received_ns=SECOND,
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
        scene_received_ns=SECOND,
        checked_ns=SECOND,
        collision_free=True,
    )
    clear, detail = state.decision(SECOND, SECOND, SECOND, SECOND)
    assert not clear
    assert "joint state stale" == detail
