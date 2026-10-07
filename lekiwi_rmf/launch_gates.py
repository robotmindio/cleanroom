"""Readiness-gate wiring shared by the bringup launch files.

Dependency gates replace fixed launch delays. Each gate exits only after a
real message/action server is available, then starts its dependent stage.
A failed camera, driver, or mapper therefore leaves downstream motion and
fleet components stopped instead of launching a noisy degraded stack.
"""

from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def after_success(stage, actions):
    """Start dependent launch actions only when a readiness gate succeeded.

    A gate normally waits forever for an unavailable dependency.  This explicit
    exit-status check also keeps an invalid parameter, import error, or other
    gate crash from being treated as readiness by ``OnProcessExit``.
    """
    def on_exit(event, context):
        # SIGINT makes the lightweight gates leave their spin loops cleanly.
        # Do not mistake that clean exit for readiness and start RTAB-Map/Nav2
        # after launch has already begun tearing the stack down.
        if context.is_shutdown or event.returncode == 130:
            return []
        if event.returncode == 0:
            return actions
        return [LogInfo(msg=f"ERROR: {stage} readiness gate exited with {event.returncode}; dependents remain stopped")]

    return on_exit


def gated(gate, stage, actions):
    """Start ``gate`` now and ``actions`` only after it reports ``stage`` ready."""
    return [gate, RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=after_success(stage, actions)))]
