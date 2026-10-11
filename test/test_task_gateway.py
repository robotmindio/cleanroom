"""Native ROS actions behind durable, reconnectable robot task services; no motors."""

import json
import threading
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from lekiwi_rmf.action import Explore
from lekiwi_rmf.exploration_owner_guard import ExplorationOwnerGuard
from lekiwi_rmf.srv import CancelTask, GetTask, SubmitTask
from lekiwi_rmf.task_gateway import TaskGateway, task_goal


def wait(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('ROS task operation did not complete')


@pytest.mark.parametrize('capability,parameters', [
    ('navigate', '{}'), ('navigate', '{"x":true,"y":0}'), ('navigate', '{"x":NaN,"y":0}'),
    ('navigate', '{"x":0,"y":0,"behavior_tree":"unsafe"}'), ('explore', '{"revisit_known":1}'),
    ('explore', '{"max_radius_m":-1}'), ('explore', '{"resume_task_id":"arbitrary"}'),
    ('explore', '[]'), ('drive', '{}'), ('navigate', ' ' * 4097),
])
def test_invalid_goal(capability, parameters):
    with pytest.raises(ValueError):
        task_goal(capability, parameters)


class Peers(Node):
    def __init__(self, context):
        super().__init__('task_peers', context=context)
        self.finish = False
        self.abort = False
        self.paused = False
        self.received = []
        self.acceptance_delay = 0.0
        group = ReentrantCallbackGroup()
        self.navigation = ActionServer(self, NavigateToPose, '/navigate_to_pose', self.navigate,
                                       goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                       callback_group=group)
        self.exploration = ActionServer(self, Explore, '/robot/explore', self.explore,
                                        goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                        callback_group=group)

    def accept(self, _request):
        from rclpy.action import GoalResponse
        time.sleep(self.acceptance_delay)
        return GoalResponse.ACCEPT

    def execute(self, goal, result, feedback):
        self.received.append(bytes(goal.goal_id.uuid))
        while not self.finish and not goal.is_cancel_requested:
            goal.publish_feedback(feedback())
            time.sleep(0.05)
        if goal.is_cancel_requested:
            goal.canceled()
        elif self.abort:
            goal.abort()
        else:
            goal.succeed()
        return result

    def navigate(self, goal):
        return self.execute(goal, NavigateToPose.Result(), lambda: NavigateToPose.Feedback(current_pose=PoseStamped()))

    def explore(self, goal):
        return self.execute(goal, Explore.Result(complete=not self.abort, message='finished'),
                            lambda: Explore.Feedback(stage='paused: StopZone' if self.paused else 'exploring_frontiers'))


@pytest.fixture
def system(tmp_path):
    context = Context()
    rclpy.init(context=context)
    peers = Peers(context)
    guard = ExplorationOwnerGuard(context=context)
    path = str(tmp_path / 'tasks.sqlite3')
    gateway = TaskGateway(context=context, parameter_overrides=[Parameter('database_path', value=path)])
    client = Node('task_client', context=context)
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for node in (peers, guard, client):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    gateway_executor = SingleThreadedExecutor(context=context)
    gateway_executor.add_node(gateway)
    gateway_thread = threading.Thread(target=gateway_executor.spin, daemon=True)
    gateway_thread.start()
    submit = client.create_client(SubmitTask, '/robot/tasks/submit')
    get = client.create_client(GetTask, '/robot/tasks/get')
    cancel = client.create_client(CancelTask, '/robot/tasks/cancel')
    wait(lambda: submit.service_is_ready() and gateway._action_clients['explore'].server_is_ready())

    def call(service, request):
        future = service.call_async(request)
        wait(future.done)
        return future.result()

    def status(identifier='task-1'):
        return call(get, GetTask.Request(task_id=identifier))

    def start(capability='explore', parameters='{}', identifier='task-1'):
        return call(submit, SubmitTask.Request(task_id=identifier, capability=capability, parameters_json=parameters))

    def stop(identifier='task-1'):
        return call(cancel, CancelTask.Request(task_id=identifier))

    def restart():
        nonlocal gateway, gateway_executor, gateway_thread
        gateway_executor.shutdown(timeout_sec=5)
        gateway_thread.join(timeout=5)
        gateway.destroy_node()
        gateway = TaskGateway(context=context, parameter_overrides=[Parameter('database_path', value=path)])
        gateway_executor = SingleThreadedExecutor(context=context)
        gateway_executor.add_node(gateway)
        gateway_thread = threading.Thread(target=gateway_executor.spin, daemon=True)
        gateway_thread.start()
        wait(submit.service_is_ready)

    yield peers, start, status, stop, restart
    peers.finish = True
    gateway_executor.shutdown(timeout_sec=5)
    gateway_thread.join(timeout=5)
    executor.shutdown(timeout_sec=5)
    thread.join(timeout=5)
    for node in (gateway, client, peers, guard):
        node.destroy_node()
    context.try_shutdown()


def test_admission_deduplicates_and_rejects_conflicting_payload(system):
    peers, start, status, stop, _ = system
    assert start().accepted
    wait(lambda: status().state == 'running')
    assert start().accepted
    assert not start(parameters='{"revisit_known":true}').accepted
    assert not start(identifier='task-2').accepted
    assert len(peers.received) == 1
    assert stop().state == 'canceling'
    wait(lambda: status().state == 'canceled')
    assert start().state == 'canceled'
    assert len(peers.received) == 1


def test_pause_is_observable_and_does_not_finish_task(system):
    peers, start, status, stop, _ = system
    assert start().accepted
    peers.paused = True
    wait(lambda: status().state == 'paused')
    assert 'StopZone' in status().detail
    peers.paused = False
    wait(lambda: status().state == 'running')
    stop('')
    wait(lambda: status().state == 'canceled')


@pytest.mark.parametrize('capability', ['navigate', 'explore'])
@pytest.mark.parametrize('abort', [False, True])
def test_terminal_state_comes_from_ros_result(system, capability, abort):
    peers, start, status, _, _ = system
    peers.abort = abort
    assert start(capability, '{"x":0,"y":0}' if capability == 'navigate' else '{}').accepted
    wait(lambda: len(peers.received) == 1)
    peers.finish = True
    wait(lambda: status().state in ('succeeded', 'failed'))
    assert status().state == ('failed' if abort else 'succeeded')
    assert isinstance(json.loads(status().result_json), dict)


def test_restart_recovers_live_exploration_without_resubmitting(system):
    peers, start, status, stop, restart = system
    assert start().accepted
    wait(lambda: status().state == 'running')
    restart()
    assert start().accepted
    assert len(peers.received) == 1
    stop()
    wait(lambda: status().state == 'canceled')
    restart()
    assert status().state == 'canceled'


def test_late_acceptance_still_gets_canceled(system):
    peers, start, status, stop, _ = system
    peers.acceptance_delay = 0.4
    assert start().accepted
    time.sleep(0.25)
    assert stop().accepted
    wait(lambda: status().state == 'canceled')
    assert len(peers.received) <= 1


def test_restart_does_not_revive_navigation_lease(system):
    peers, start, status, _, restart = system
    assert start('navigate', '{"x":0,"y":0}').accepted
    wait(lambda: status().state == 'running')
    restart()
    wait(lambda: status().state == 'canceled')
    assert len(peers.received) == 1


def test_other_ros_clients_keep_their_ownership(system):
    from rclpy.action import ActionClient
    peers, start, status, stop, _ = system
    native = ActionClient(peers, Explore, '/robot/explore')
    wait(native.server_is_ready)
    accepted = native.send_goal_async(Explore.Goal())
    wait(accepted.done)
    handle = accepted.result()
    assert handle.accepted
    wait(lambda: len(peers.received) == 1)
    time.sleep(0.2)
    assert not start().accepted
    assert not stop('').accepted
    result = handle.get_result_async()
    handle.cancel_goal_async()
    wait(result.done)
    assert not status().found
    native.destroy()


def test_duplicate_gateway_cannot_own_the_same_journal(tmp_path):
    context = Context()
    rclpy.init(context=context)
    parameter = Parameter('database_path', value=str(tmp_path / 'tasks.sqlite3'))
    gateway = TaskGateway(context=context, parameter_overrides=[parameter])
    try:
        with pytest.raises(BlockingIOError):
            TaskGateway(context=context, parameter_overrides=[parameter], namespace='/duplicate')
    finally:
        gateway.destroy_node()
        context.try_shutdown()


def test_lost_cancel_reply_is_retried_without_claiming_termination():
    from queue import SimpleQueue
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.task import Future

    old, retry = Future(), Future()
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.return_value = retry
    row = {'id': 'task-1', 'capability': 'explore', 'goal_id': bytes(16), 'cancel_requested': 1}
    gateway = SimpleNamespace(
        _replies=SimpleQueue(), _active=lambda: [row], _recovering={'task-1'},
        _pending={('cancel', 'task-1'): old},
        _queried={('cancel', 'task-1'): time.monotonic() - 4},
        _terminal_seen={}, _cancel_clients={'explore': client},
        _lookup_result=Mock(), _result=Mock(), _accepted=Mock(), _canceled=Mock(),
    )
    TaskGateway._tick(gateway)
    assert old.cancelled()
    client.remove_pending_request.assert_called_once_with(old)
    client.call_async.assert_called_once()
    assert gateway._pending[('cancel', 'task-1')] is retry
    TaskGateway._canceled(gateway, 'task-1', old)
    assert gateway._pending[('cancel', 'task-1')] is retry
