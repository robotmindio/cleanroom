"""A timed-out arm jog must ask the controller to cancel its active goal."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location("arm_jog", ROOT / "scripts/arm_jog.py")
arm_jog = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(arm_jog)


class Future:
    def __init__(self, result=None, done=False):
        self.value = result
        self.finished = done

    def done(self):
        return self.finished

    def result(self):
        return self.value


def test_result_timeout_requests_and_confirms_goal_cancel(monkeypatch):
    result_future = Future()
    cancel_future = Future()

    class Handle:
        def cancel_goal_async(self):
            return cancel_future

    def spin(_node, future, timeout_sec):
        if future is cancel_future:
            cancel_future.value = SimpleNamespace(goals_canceling=[object()])
            cancel_future.finished = True

    monkeypatch.setattr(arm_jog.rclpy, "spin_until_future_complete", spin)
    with pytest.raises(RuntimeError, match="cancellation was acknowledged"):
        arm_jog.wait_for_result(object(), Handle(), result_future, 9.0)
