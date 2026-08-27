"""Unit checks for the command-source boundary before collision monitoring."""

from math import nan
import time

from geometry_msgs.msg import Twist

from lekiwi_rmf.cmd_vel_mux import CmdVelMux, _Command, _finite_twist


def _twist(x: float) -> Twist:
    message = Twist()
    message.linear.x = x
    return message


def _bare_mux() -> CmdVelMux:
    node = CmdVelMux.__new__(CmdVelMux)
    node._manual_timeout_ns = 250
    node._navigation_timeout_ns = 500
    node._permission_timeout_ns = 500_000_000
    node._manual = None
    node._navigation = None
    node._motion_permitted = True
    node._permission_received_at_ns = time.monotonic_ns()
    return node


def test_rejects_non_finite_values_in_unused_axes_too():
    message = _twist(0.1)
    message.angular.z = nan
    assert not _finite_twist(message)


def test_fresh_manual_command_preempts_navigation():
    node = _bare_mux()
    node._navigation = _Command(_twist(0.1), 600)
    node._manual = _Command(_twist(0.2), 800)

    selected, source = node.selected_command(now=1_000)

    assert source == "manual"
    assert selected.linear.x == 0.2


def test_stale_manual_yields_to_fresh_navigation_then_stops():
    node = _bare_mux()
    node._manual = _Command(_twist(0.2), 600)
    node._navigation = _Command(_twist(0.1), 800)

    selected, source = node.selected_command(now=1_000)
    assert source == "navigation"
    assert selected.linear.x == 0.1

    selected, source = node.selected_command(now=1_301)
    assert source == "none"
    assert selected.linear.x == 0.0
    assert selected.linear.y == 0.0
    assert selected.angular.z == 0.0


def test_clock_rewind_never_revives_a_future_dated_command():
    node = _bare_mux()
    node._manual = _Command(_twist(0.2), 1_001)

    selected, source = node.selected_command(now=1_000)

    assert source == "none"
    assert selected.linear.x == 0.0


def test_motion_interlock_overrides_fresh_commands():
    node = _bare_mux()
    node._manual = _Command(_twist(0.2), 900)
    node._motion_permitted = False

    selected, source = node.selected_command(now=1_000)

    assert source == "interlock"
    assert selected.linear.x == 0.0


def test_transient_local_true_permission_expires_without_a_new_message():
    node = _bare_mux()
    received_at = 10_000
    node._permission_received_at_ns = received_at
    node._manual = _Command(_twist(0.2), 900)

    selected, source = node.selected_command(
        now=1_000, permission_now=received_at + 500_000_001
    )

    assert source == "interlock"
    assert selected.linear.x == 0.0


def test_permission_callback_refreshes_receive_time():
    node = _bare_mux()
    node._permission_callback(type("BoolMessage", (), {"data": True})())

    assert node._motion_permitted is True
    assert node._permission_is_fresh(
        node._permission_received_at_ns, node._permission_timeout_ns
    )
