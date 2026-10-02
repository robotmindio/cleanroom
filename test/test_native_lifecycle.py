"""Exercise the deployed native manager with delayed and refused ROS services."""
from __future__ import annotations

import signal
import subprocess
import threading
import time
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


@pytest.mark.parametrize(
    ("delay", "timeout", "refuse", "expected"),
    [(2.5, 4, False, True), (1.5, 1, False, False), (0.0, 4, True, False),
     (-1.0, 1, False, False)],
)
def test_native_lifecycle_response_deadline_and_refusal(tmp_path, delay, timeout, refuse, expected):
    context = Context()
    rclpy.init(context=context)
    node = Node("native_lifecycle_fixture", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    state = State.PRIMARY_STATE_UNCONFIGURED
    state_requests = 0

    def change_state(request, response):
        nonlocal state
        if delay < 0 and request.transition.id == Transition.TRANSITION_CONFIGURE:
            node.destroy_service(state_service)
        state = {
            Transition.TRANSITION_CONFIGURE: State.PRIMARY_STATE_INACTIVE,
            Transition.TRANSITION_ACTIVATE: State.PRIMARY_STATE_ACTIVE,
            Transition.TRANSITION_DEACTIVATE: State.PRIMARY_STATE_INACTIVE,
            Transition.TRANSITION_CLEANUP: State.PRIMARY_STATE_UNCONFIGURED,
        }.get(request.transition.id, State.PRIMARY_STATE_FINALIZED)
        response.success = not refuse
        return response

    def get_state(_request, response):
        nonlocal state_requests
        state_requests += 1
        if state_requests == 1:
            time.sleep(delay)
        response.current_state.id = state
        return response

    node.create_service(ChangeState, "/native_fixture/change_state", change_state)
    state_service = node.create_service(GetState, "/native_fixture/get_state", get_state)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    binary = Path(get_package_prefix("nav2_lifecycle_manager")) / \
        "lib/nav2_lifecycle_manager/lifecycle_manager"
    process = None
    log_path = tmp_path / "manager.log"
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen([
                str(binary), "--ros-args", "-r", "__node:=native_manager_probe",
                "-p", "node_names:=[native_fixture]", "-p", "autostart:=true",
                "-p", "bond_timeout:=0.0", "-p", f"service_timeout:={timeout}",
            ], stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                output = log_path.read_text()
                if "Managed nodes are active" in output or "Aborting bringup" in output:
                    break
                assert process.poll() is None, output
                time.sleep(0.05)
            else:
                pytest.fail("native lifecycle startup did not finish: " + output)
            assert ("Managed nodes are active" in output) is expected, output
            if refuse:
                assert state_requests == 0, "refused transition must not be treated as successful"
            elif not expected:
                error = "response deadline exceeded" if delay >= 0 else "service unavailable"
                assert error in output, output
    finally:
        try:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
                    pytest.fail("native lifecycle manager did not shut down")
        finally:
            executor.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            node.destroy_node()
            context.shutdown()
    assert process.returncode == 0, log_path.read_text()
