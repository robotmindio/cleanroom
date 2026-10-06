"""Local motion, indefinite network pause, and idempotent control requests."""
import copy
import math

import pytest

from lekiwi_rmf.arm_trajectory import ARM_JOINTS, JOINT_LIMITS
from lekiwi_rmf.local_arm_executor import LocalArmExecutor, validate_status


def setup_executor():
    now = [0.0]
    executor = LocalArmExecutor(lambda: now[0], 0.5)
    observation = {f"{name}.pos": 0.0 for name in ARM_JOINTS}
    request = {
        "id": 123, "names": [ARM_JOINTS[0]],
        "points": [[2.0, [0.1], [], [], []]],
        "zeros": dict.fromkeys(ARM_JOINTS, 0.0),
        "directions": dict.fromkeys(ARM_JOINTS, 1.0),
        "path": {ARM_JOINTS[0]: 0.2}, "goal": {ARM_JOINTS[0]: 0.02},
        "settling": 5.0, "delay": 0.0,
    }
    executor.start(request, observation)
    return executor, now, observation, request


def test_local_clock_retains_goal_through_a_long_network_gap():
    executor, now, observation, request = setup_executor()
    executor.renew(123, True)
    executor.step(observation, now[0])
    for _ in range(10):
        now[0] += 0.033
        executor.renew(123, True)
        observation.update(executor.step(observation, now[0]))
    elapsed = executor.status["elapsed"]
    now[0] += 12.0
    hold = executor.step(observation, now[0])
    assert executor.status["state"] == "paused"
    assert executor.status["elapsed"] == elapsed
    assert hold["x.vel"] == hold["y.vel"] == hold["theta.vel"] == 0
    assert executor.start(copy.deepcopy(request), observation)["elapsed"] == elapsed
    executor.renew(999, True)  # A different goal cannot restart this arm.
    now[0] += 0.033
    executor.step(observation, now[0])
    assert executor.status["state"] == "paused"
    executor.renew(123, True)
    executor.step(observation, now[0])
    assert executor.status["elapsed"] == elapsed  # No catch-up jump.
    for _ in range(90):
        now[0] += 0.033
        executor.renew(123, True)
        action = executor.step(observation, now[0])
        if action:
            observation.update(action)
    assert executor.status["state"] == "succeeded"
    assert observation[f"{ARM_JOINTS[0]}.pos"] == pytest.approx(math.degrees(0.1))
    validate_status(executor.status)


def test_revoked_permission_holds_and_cancel_never_restarts():
    executor, now, observation, request = setup_executor()
    executor.renew(123, True)
    executor.step(observation, now[0])
    executor.renew(123, False)
    now[0] = 0.1
    action = executor.step(observation, now[0])
    assert action[f"{ARM_JOINTS[0]}.pos"] == observation[f"{ARM_JOINTS[0]}.pos"]
    observation[f"{ARM_JOINTS[0]}.pos"] += 0.5
    now[0] += 0.033
    assert executor.step(observation, now[0]) == action  # Hold cannot follow gravity drift.
    executor.cancel(999)
    assert executor.active
    executor.cancel(123)
    executor.start(copy.deepcopy(request), observation)
    executor.renew(123, True)
    assert executor.step(observation, now[0]) is None
    assert executor.status["state"] == "canceled"
    request["points"][0][1] = [0.12]
    with pytest.raises(ValueError, match="reused"):
        executor.start(request, observation)


def test_local_tracking_fault_uses_measured_hold():
    executor, now, observation, _request = setup_executor()
    executor.renew(123, True)
    executor.step(observation, now[0])
    observation[f"{ARM_JOINTS[0]}.pos"] = math.degrees(0.5)
    now[0] = 0.033
    action = executor.step(observation, now[0])
    assert executor.status["code"] == -4
    assert executor.status["state"] == "aborted"
    assert "error=" in executor.status["detail"] and "limit=" in executor.status["detail"]
    assert action[f"{ARM_JOINTS[0]}.pos"] == observation[f"{ARM_JOINTS[0]}.pos"]


def test_invalid_local_feedback_cannot_advance_motion():
    executor, now, observation, _request = setup_executor()
    executor.renew(123, True)
    observation[f"{ARM_JOINTS[0]}.pos"] = float("nan")
    with pytest.raises(ValueError, match="non-finite"):
        executor.step(observation, now[0])
    assert executor.status["elapsed"] == 0


def test_recovery_setpoints_stay_inside_the_joint_bound():
    executor, now, observation, request = setup_executor()
    joint = ARM_JOINTS[0]
    upper = JOINT_LIMITS[joint][1]
    observation[f"{joint}.pos"] = math.degrees(upper + 0.07)
    request["id"] += 1
    request["points"] = [[2.0, [upper - 0.02], [], [], []]]
    executor.start(request, observation)
    executor.renew(request["id"], True)
    action = executor.step(observation, now[0])
    assert action[f"{joint}.pos"] == pytest.approx(math.degrees(upper))


def test_goal_timeout_reports_the_joint_and_holds_measured_feedback():
    executor, now, observation, request = setup_executor()
    for _ in range(220):
        now[0] += 0.033
        executor.renew(request["id"], True)
        executor.step(observation, now[0])
    assert executor.status["state"] == "aborted" and executor.status["code"] == -5
    assert executor.status["detail"] == (
        f"goal tolerance exceeded for {ARM_JOINTS[0]}: error=0.1000 rad, limit=0.0200 rad"
    )
    assert executor.hold[f"{ARM_JOINTS[0]}.pos"] == observation[f"{ARM_JOINTS[0]}.pos"]
