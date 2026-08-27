"""Live DDS proof that a visible duplicate adapter blocks the preflight."""

from __future__ import annotations

import pathlib
import sys
import unittest

import launch
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest

from lekiwi_rmf.rmf_owner_guard import CONFLICT_EXIT


ROOT = pathlib.Path(__file__).parents[1]


@pytest.mark.rostest
def generate_test_description():
    peer = ExecuteProcess(
        cmd=[sys.executable, str(ROOT / "test" / "rmf_owner_guard_peer.py")],
        output="screen",
    )
    guard = launch_ros.actions.Node(
        package="lekiwi_rmf",
        executable="rmf_owner_guard",
        name="rmf_owner_guard_launch_test",
        parameters=[{
            "fleet_name": "lekiwi",
            # Give peer discovery a full second before and during the scan.
            "settle_seconds": 1.0,
            "poll_period_seconds": 0.05,
        }],
        output="screen",
    )
    return launch.LaunchDescription([
        peer,
        TimerAction(period=1.0, actions=[guard]),
        launch_testing.actions.ReadyToTest(),
    ]), {"guard": guard}


class TestRMFOwnerGuardGraph(unittest.TestCase):
    def test_detected_peer_returns_the_dedicated_conflict_code(self, proc_info, guard):
        proc_info.assertWaitForShutdown(process=guard, timeout=8)
        launch_testing.asserts.assertExitCodes(
            proc_info, process=guard, allowable_exit_codes=[CONFLICT_EXIT]
        )
