"""Unit checks for the command-source boundary before collision monitoring."""

from math import nan

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
    node._manual = None
    node._navigation = None
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
