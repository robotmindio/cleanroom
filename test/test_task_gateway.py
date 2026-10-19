"""Native ROS actions behind durable, reconnectable robot task services; no motors."""

import json
import sqlite3
import threading
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile

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
    ('explore', '{"resume_task_id":"' + 'a' * 32 + '","max_radius_m":1}'),
    ('explore', '{"resume_task_id":"' + 'a' * 32 + '","max_duration_sec":1}'),
    ('explore', '[' * 1100 + '0' + ']' * 1100),
])
def test_invalid_goal(capability, parameters):
    with pytest.raises(ValueError):
        task_goal(capability, parameters)


class Peers(Node):
    def __init__(self, context):
        super().__init__('task_peers', context=context)
        self.finish = False
        self.abort = False
        self.hold_cancel = False
        self.completed = []
        self.paused = False
        self.received = []
        self.requests = []
        self.acceptance_delay = 0.0
        self.permission = True
        self.collision_stop = False
        self.permission_pub = self.create_publisher(Bool, '/safety/base_motion_permitted',
                                                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.collision_pub = self.create_publisher(CollisionMonitorState, '/collision_monitor_state', 10)
        self.create_timer(0.05, self.safety)
        group = ReentrantCallbackGroup()
        self.navigation = ActionServer(self, NavigateToPose, '/navigate_to_pose', self.navigate,
                                       goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                       callback_group=group)
        self.exploration = ActionServer(self, Explore, '/robot/explore', self.explore,
                                        goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                        callback_group=group)

    def destroy_node(self):
        self.navigation.destroy()
        self.exploration.destroy()
        return super().destroy_node()

    def safety(self):
        self.permission_pub.publish(Bool(data=self.permission))
        self.collision_pub.publish(CollisionMonitorState(
            action_type=CollisionMonitorState.STOP if self.collision_stop else CollisionMonitorState.DO_NOTHING,
            polygon_name='StopZone'))

    def accept(self, _request):
        from rclpy.action import GoalResponse
        time.sleep(self.acceptance_delay)
        return GoalResponse.ACCEPT

    def execute(self, goal, result, feedback):
        self.received.append(bytes(goal.goal_id.uuid))
        self.requests.append(goal.request)
        while not self.finish and (not goal.is_cancel_requested or self.hold_cancel):
            goal.publish_feedback(feedback())
            time.sleep(0.05)
        if goal.is_cancel_requested:
            goal.canceled()
        elif self.abort:
            goal.abort()
        else:
            goal.succeed()
        self.completed.append(bytes(goal.goal_id.uuid))
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
    peers.gateway = gateway
    peers.guard = guard
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

    def restart(allow_navigation=True):
        nonlocal gateway, gateway_executor, gateway_thread
        gateway_executor.shutdown(timeout_sec=5)
        gateway_thread.join(timeout=5)
        gateway.destroy_node()
        gateway = TaskGateway(context=context, parameter_overrides=[
            Parameter('database_path', value=path), Parameter('allow_navigation', value=allow_navigation)])
        peers.gateway = gateway
        gateway_executor = SingleThreadedExecutor(context=context)
        gateway_executor.add_node(gateway)
        gateway_thread = threading.Thread(target=gateway_executor.spin, daemon=True)
        gateway_thread.start()
        wait(submit.service_is_ready)

    def journal_row(identifier):
        with sqlite3.connect(path) as database:
            database.row_factory = sqlite3.Row
            return database.execute('SELECT * FROM tasks WHERE id=?', (identifier,)).fetchone()

    def on_gateway(function, *args):
        future = gateway_executor.create_task(function, *args)
        wait(future.done)
        return future.result()

    peers.journal_row = journal_row
    peers.on_gateway = on_gateway
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
    wait(lambda: bool(peers.gateway._observed['explore']))
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


@pytest.mark.parametrize('completed_late', [False, True])
def test_lost_cancel_reply_is_retried_without_claiming_termination(completed_late):
    from queue import SimpleQueue
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.task import Future

    old, retry = Future(), Future()
    if completed_late:
        old.set_result(SimpleNamespace(return_code=0))
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.return_value = retry
    row = {'id': 'task-1', 'capability': 'explore', 'goal_id': bytes(16), 'cancel_requested': 1, 'state': 'canceling', 'navigation_stopped': 0}
    gateway = SimpleNamespace(
        _replies=SimpleQueue(), _active=lambda: [row], _row=lambda _: row, _recovering={'task-1'},
        _pending={('cancel', 'task-1'): old},
        _queried={('cancel', 'task-1'): time.monotonic() - 4},
        _terminal_seen={}, _cancel_clients={'explore': client},
        _reconcile=Mock(return_value=False), _unknown_results={}, _pausing=set(), _server_id=lambda _: 'server',
        _queue_reply=Mock(), _lookup_result=Mock(), _result=Mock(), _accepted=Mock(), _canceled=Mock(),
    )
    TaskGateway._tick(gateway)
    assert old.cancelled() == (not completed_late)
    client.remove_pending_request.assert_called_once_with(old)
    client.call_async.assert_called_once()
    assert gateway._pending[('cancel', 'task-1')] is retry
    TaskGateway._canceled(gateway, 'task-1', old)
    assert gateway._pending[('cancel', 'task-1')] is retry


def test_late_acceptance_error_cannot_replace_terminal_result():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.task import Future

    future = Future()
    future.set_exception(RuntimeError('lost acceptance reply'))
    gateway = SimpleNamespace(_pending={}, _row=lambda _: {'state': 'succeeded'},
                              _update=Mock(), get_logger=Mock(), _pausing=set())
    TaskGateway._accepted(gateway, 'task-1', future)
    gateway._update.assert_not_called()


@pytest.mark.parametrize('cancel_before_restart,allow_navigation', [(False, True), (True, True), (False, False)])
def test_restart_handles_proven_unsent_task(system, cancel_before_restart, allow_navigation):
    peers, start, status, stop, restart = system
    # Hold the dispatch loop while service callbacks still admit and cancel work.
    for timer in peers.gateway.timers:
        timer.cancel()
    assert start('navigate', '{"x":0,"y":0}').accepted
    assert not peers.received
    if cancel_before_restart:
        assert stop().accepted
    restart(allow_navigation=allow_navigation)
    if not allow_navigation:
        wait(lambda: status().state == 'failed')
        assert not peers.received
    elif cancel_before_restart:
        wait(lambda: status().state == 'canceled')
        assert not peers.received
    else:
        wait(lambda: status().state == 'running')
        assert len(peers.received) == 1
        stop()
        wait(lambda: status().state == 'canceled')


def test_failed_submission_is_durable_and_never_replayed(tmp_path):
    from types import SimpleNamespace
    from unittest.mock import Mock

    from lekiwi_rmf.task_journal import TaskJournal

    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        journal.admit('task', 'explore', '{}', bytes(16))
        action = Mock()
        action.server_is_ready.return_value = True
        action.send_goal_async.side_effect = RuntimeError('transport lost')
        gateway = SimpleNamespace(_journal=journal, _enabled={'explore': True}, _observed={'explore': set()},
                                  _action_clients={'explore': action}, _children=Mock(busy=lambda: False), _sent=set(),
                                  _started={}, _recovering=set(), _update=journal.update,
                                  _server_id=lambda _: 'server', get_logger=Mock())
        gateway._delivery_failed = lambda task, error: TaskGateway._delivery_failed(gateway, task, error)
        TaskGateway._dispatch(gateway, journal.row('task'), time.monotonic())
        assert journal.row('task')['dispatched']
        assert journal.row('task')['state'] == 'unknown'
        assert gateway._recovering == {'task'}
    finally:
        journal.close()


def test_dispatch_waits_for_observed_foreign_goal(tmp_path):
    from types import SimpleNamespace
    from unittest.mock import Mock

    from lekiwi_rmf.task_journal import TaskJournal

    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        journal.admit('task', 'explore', '{}', bytes(16))
        action = Mock()
        gateway = SimpleNamespace(_journal=journal, _enabled={'explore': True}, _observed={'navigate': {bytes(range(16))}},
                                  _action_clients={'explore': action})
        TaskGateway._dispatch(gateway, journal.row('task'), time.monotonic())
        assert not journal.row('task')['dispatched']
        action.send_goal_async.assert_not_called()
    finally:
        journal.close()


def test_user_cancel_preserves_navigation_deadline_failure(tmp_path):
    from types import SimpleNamespace
    from rclpy.task import Future
    from action_msgs.msg import GoalStatus

    from lekiwi_rmf.task_journal import TaskJournal

    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        journal.admit('task', 'navigate', '{}', bytes(16))
        journal.request_cancel('task', reason=2)
        gateway = SimpleNamespace(_journal=journal, _active=journal.active,
                                  _row=journal.row, _update=journal.update, _pending={}, _pausing=set())
        response = TaskGateway._cancel(gateway, CancelTask.Request(task_id='task'), CancelTask.Response())
        assert response.accepted
        future = Future()
        future.set_result(NavigateToPose.Impl.GetResultService.Response(
            status=GoalStatus.STATUS_CANCELED, result=NavigateToPose.Result()))
        TaskGateway._result(gateway, 'task', future)
        assert journal.row('task')['state'] == 'failed'
        assert 'navigation deadline reached' in journal.row('task')['detail']
    finally:
        journal.close()


def test_invalid_native_result_does_not_crash_gateway(tmp_path):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.task import Future
    from action_msgs.msg import GoalStatus

    from lekiwi_rmf.task_journal import TaskJournal

    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        journal.admit('task', 'explore', '{}', bytes(16))
        gateway = SimpleNamespace(_row=journal.row, _update=journal.update,
                                  _pending={}, get_logger=Mock(), _pausing=set())
        future = Future()
        future.set_result(Explore.Impl.GetResultService.Response(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=Explore.Result(complete=True, mapped_area_m2=float('nan'))))
        TaskGateway._result(gateway, 'task', future)
        assert journal.row('task')['state'] == 'unknown'
        assert journal.row('task')['result'] == '{}'
    finally:
        journal.close()


def test_canceled_acceptance_request_records_uncertain_delivery():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.task import Future

    future = Future()
    future.cancel()
    gateway = SimpleNamespace(_pending={}, _recovering=set(),
                              _row=lambda _: {'state': 'starting'}, _update=Mock(), get_logger=Mock())
    gateway._delivery_failed = lambda task, error: TaskGateway._delivery_failed(gateway, task, error)
    TaskGateway._accepted(gateway, 'task', future)
    assert gateway._recovering == {'task'}
    gateway._update.assert_called_once_with('task', 'unknown', 'action acceptance unconfirmed; not resubmitted')


def test_terminal_task_releases_lost_transport_requests(tmp_path):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.task import Future

    from lekiwi_rmf.task_journal import TaskJournal

    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        journal.admit('task', 'explore', '{}', bytes(16))
        futures = {kind: Future() for kind in ('send', 'result', 'cancel')}
        result_client, cancel_client = Mock(), Mock()
        gateway = SimpleNamespace(
            _journal=journal, _row=journal.row, _events=Mock(),
            _pending={(kind, 'task'): future for kind, future in futures.items()},
            _queried={(kind, 'task'): 0 for kind in futures},
            _result_clients={'explore': result_client}, _cancel_clients={'explore': cancel_client},
            _recovering={'task'}, _sent={'task'}, _started={'task': 0}, _terminal_seen={'task': 0},
            _pausing=set(), _pause_started={}, _unknown_results={}, _unknown_cancels={}, _unknown_since={})
        gateway._release_requests = lambda task, row: TaskGateway._release_requests(gateway, task, row)
        TaskGateway._update(gateway, 'task', 'succeeded', 'finished', {'complete': True})
        assert journal.row('task')['state'] == 'succeeded'
        assert not gateway._pending and not gateway._queried
        assert all(future.cancelled() for future in futures.values())
        result_client.remove_pending_request.assert_called_once_with(futures['result'])
        cancel_client.remove_pending_request.assert_called_once_with(futures['cancel'])
        assert not gateway._recovering and not gateway._sent
    finally:
        journal.close()


def test_late_result_reply_preserves_current_lookup(tmp_path):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.task import Future
    from action_msgs.msg import GoalStatus

    from lekiwi_rmf.task_journal import TaskJournal

    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        journal.admit('task', 'explore', '{}', bytes(16))
        old, current = Future(), Future()
        gateway = SimpleNamespace(_row=journal.row, _update=journal.update,
                                  _pending={('result', 'task'): current}, _recovering=set(), get_logger=Mock())
        old.set_result(Explore.Impl.GetResultService.Response(status=GoalStatus.STATUS_UNKNOWN))
        TaskGateway._result(gateway, 'task', old)
        assert gateway._pending[('result', 'task')] is current
        current.set_result(Explore.Impl.GetResultService.Response(
            status=GoalStatus.STATUS_SUCCEEDED, result=Explore.Result(complete=True)))
        TaskGateway._result(gateway, 'task', current)
        assert journal.row('task')['state'] == 'succeeded'
        assert not gateway._pending
    finally:
        journal.close()


@pytest.mark.parametrize('fault', ['collision', 'permission'])
def test_navigation_pause_waits_for_terminal_proof_then_resumes_same_destination(system, fault):
    peers, start, status, stop, _ = system
    assert start('navigate', '{"x":1,"y":2,"yaw":0.5}').accepted
    wait(lambda: len(peers.received) == 1 and status().state == 'running')
    original = peers.received[0]
    old_row = peers.journal_row('task-1')
    peers.hold_cancel = True
    if fault == 'collision':
        peers.collision_stop = True
    else:
        peers.permission = False
    wait(lambda: status().state == 'paused')
    budget = peers.journal_row('task-1')['remaining_sec']
    time.sleep(1.2)
    assert peers.journal_row('task-1')['remaining_sec'] == budget
    assert peers.received == [original]
    peers.collision_stop, peers.permission = False, True
    time.sleep(1.2)
    assert peers.received == [original]  # Clear inputs alone cannot authorize a second attempt.
    peers.hold_cancel = False
    wait(lambda: len(peers.received) == 2)
    assert peers.received[1] != original
    from rclpy.task import Future
    from action_msgs.msg import GoalStatus
    late = Future()
    late.set_result(NavigateToPose.Impl.GetResultService.Response(status=GoalStatus.STATUS_SUCCEEDED))
    peers.gateway._queue_reply('result', old_row, late)
    time.sleep(0.3)
    assert status().state == 'running'
    assert peers.journal_row('task-1')['parameters'] == '{"x": 1, "y": 2, "yaw": 0.5}'
    assert stop().accepted
    wait(lambda: status().state == 'canceled')


@pytest.mark.parametrize('cancel', [False, True])
def test_cancel_and_restart_while_navigation_is_safely_paused(system, cancel):
    peers, start, status, stop, restart = system
    assert start('navigate', '{"x":1,"y":2}').accepted
    wait(lambda: len(peers.received) == 1 and status().state == 'running')
    peers.collision_stop = True
    wait(lambda: peers.journal_row('task-1')['navigation_stopped'])
    restart()
    time.sleep(0.3)
    assert len(peers.received) == 1
    if cancel:
        assert stop().accepted
        wait(lambda: status().state == 'canceled')
        peers.collision_stop = False
        time.sleep(1.2)
        assert len(peers.received) == 1
    else:
        peers.collision_stop = False
        wait(lambda: len(peers.received) == 2)
        stop()
        wait(lambda: status().state == 'canceled')


@pytest.mark.parametrize('capability,child_active', [('explore', False), ('navigate', False), ('navigate', True)])
def test_replacement_server_reconciles_forgotten_task_without_replay(system, capability, child_active):
    peers, start, status, _, _ = system
    gateway = peers.gateway
    lookup = gateway._lookup_result
    gateway._lookup_result = lambda _: None  # Simulate unavailable result transport.
    assert start(capability, '{"x":1,"y":2}' if capability == 'navigate' else '{}').accepted
    wait(lambda: len(peers.received) == 1 and status().state == 'running')
    if capability == 'navigate':
        wait(lambda: bool(peers.guard._active))
        peers.guard._status = lambda *_: None  # Lose terminal status, as with a crashed Nav2 server.
    server_id = peers.journal_row('task-1')['server_id']
    assert server_id
    child = None
    if child_active:
        from action_msgs.msg import GoalStatus, GoalStatusArray
        child = peers.create_publisher(GoalStatusArray, '/follow_path/_action/status',
                                       QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        child.publish(GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)]))
        wait(gateway._children.busy)
    peers.finish = True
    wait(lambda: bool(peers.completed))
    server = 'navigation' if capability == 'navigate' else 'exploration'
    getattr(peers, server).destroy()
    setattr(peers, server, ActionServer(
        peers, NavigateToPose if capability == 'navigate' else Explore,
        '/navigate_to_pose' if capability == 'navigate' else '/robot/explore',
        peers.navigate if capability == 'navigate' else peers.explore, goal_callback=peers.accept,
        cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup()))
    wait(lambda: gateway._server_id(capability) not in ('', server_id))
    gateway._recovering.add('task-1')
    gateway._lookup_result = lookup
    if child_active:
        wait(lambda: status().state == 'unknown')
        time.sleep(3.5)
        assert status().state == 'unknown'
        assert not start(identifier='blocked').accepted
        child.publish(GoalStatusArray())
    wait(lambda: status().state == 'failed', timeout=10)
    result = json.loads(status().result_json)
    assert result['outcome_known'] is False
    assert result['termination_confirmed'] is True
    assert '_task' in result
    assert len(peers.received) == 1
    assert start(identifier='task-2').accepted


@pytest.mark.parametrize('original,current,guard,foreign,expected', [
    ('old', 'new', True, False, True), ('same', 'same', True, False, False),
    ('', 'new', True, False, False), ('old', 'new', False, False, False),
    ('old', 'new', True, True, False),
])
@pytest.mark.parametrize('deadline', [False, True])
def test_reconciliation_requires_replacement_server_and_clear_owner_guard(original, current, guard, foreign, expected, deadline):
    from types import SimpleNamespace
    from unittest.mock import Mock
    row = {'id': 'task', 'capability': 'explore', 'server_id': original, 'goal_id': bytes(16), 'cancel_requested': 2 if deadline else 0}
    gateway = SimpleNamespace(
        _unknown_results={'task': current}, _unknown_cancels={'task': current}, _unknown_since={'task': 5.0},
        _server_id=lambda _: current, _observed={'explore': {bytes(range(16))} if foreign else set()},
        _guard=(guard, 10.0), _children=Mock(busy=lambda: False), _update=Mock())
    assert TaskGateway._reconcile(gateway, row, 10.1) == expected
    assert gateway._update.called == expected
    if expected and deadline:
        assert gateway._update.call_args.args[2].startswith('navigation deadline reached;')


def test_navigation_pause_with_unconfirmed_stop_expires_owner_lease(system):
    peers, start, status, _, _ = system
    assert start('navigate', '{"x":1,"y":2}').accepted
    wait(lambda: len(peers.received) == 1 and status().state == 'running')
    peers.hold_cancel = True
    peers.collision_stop = True
    wait(lambda: 'task-1' in peers.gateway._pausing)
    peers.gateway._pause_started['task-1'] -= 4.0
    wait(lambda: status().state == 'unknown')
    assert 'task-1' in peers.gateway._recovering
    assert len(peers.received) == 1
    peers.hold_cancel = False
    wait(lambda: peers.journal_row('task-1')['navigation_stopped'])
    assert status().state == 'paused'


def test_navigation_active_budget_reaches_deadline(system):
    peers, start, status, _, _ = system
    assert start('navigate', '{"x":1,"y":2}').accepted
    wait(lambda: len(peers.received) == 1 and status().state == 'running')
    peers.on_gateway(peers.gateway._journal.navigation_budget, 'task-1', 0.01)
    wait(lambda: status().state == 'failed')
    assert 'navigation deadline reached' in status().detail
    assert len(peers.received) == 1


def test_owner_guard_terminal_cleanup_releases_pending_requests():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.task import Future
    goal = bytes(16)
    cancel, result = Future(), Future()
    guard = SimpleNamespace(_active={goal}, _seen={goal: 1}, _instances={goal: 'old'},
                            _forgotten={goal: ('new', 1)}, _cancels={goal: (cancel, 1)},
                            _results={goal: (result, 1)}, _cancel=Mock(), _result=Mock())
    ExplorationOwnerGuard._finished(guard, goal)
    assert not guard._active and not guard._instances and not guard._forgotten
    assert not guard._cancels and not guard._results
    assert cancel.cancelled() and result.cancelled()
    guard._cancel.remove_pending_request.assert_called_once_with(cancel)
    guard._result.remove_pending_request.assert_called_once_with(result)


def test_owner_guard_recovers_terminal_proof_when_status_was_lost(system):
    peers, start, status, _, _ = system
    assert start('navigate', '{"x":1,"y":2}').accepted
    wait(lambda: peers.guard._active and status().state == 'running')
    peers.guard._status = lambda *_: None
    peers.finish = True
    wait(lambda: status().state == 'succeeded')
    wait(lambda: not peers.guard._active)
    assert len(peers.received) == 1


def test_unknown_native_result_stops_lease_without_a_server_identity(tmp_path):
    from types import SimpleNamespace
    from rclpy.task import Future
    from action_msgs.msg import GoalStatus
    from lekiwi_rmf.task_journal import TaskJournal
    journal = TaskJournal(tmp_path / 'tasks.db')
    try:
        journal.admit('task', 'navigate', '{"x":1,"y":2}', bytes(16))
        journal.mark_dispatched('task')
        gateway = SimpleNamespace(_row=journal.row, _update=journal.update, _pending={}, _recovering=set())
        future = Future()
        future.set_result(NavigateToPose.Impl.GetResultService.Response(status=GoalStatus.STATUS_UNKNOWN))
        TaskGateway._result(gateway, 'task', future)
        assert gateway._recovering == {'task'}
        assert journal.row('task')['state'] == 'unknown'
    finally:
        journal.close()


@pytest.mark.parametrize('stop_unconfirmed', [False, True])
def test_restart_during_navigation_pause_preserves_intent_until_terminal_proof(system, stop_unconfirmed):
    peers, start, status, stop, restart = system
    assert start('navigate', '{"x":1,"y":2}').accepted
    wait(lambda: len(peers.received) == 1 and status().state == 'running')
    peers.hold_cancel = True
    peers.collision_stop = True
    wait(lambda: status().state == 'paused')
    budget = peers.journal_row('task-1')['remaining_sec']
    if stop_unconfirmed:
        peers.gateway._pause_started['task-1'] -= 4.0
        wait(lambda: status().state == 'unknown')
    restart()
    peers.collision_stop = False
    time.sleep(1.2)
    assert len(peers.received) == 1
    assert peers.journal_row('task-1')['remaining_sec'] == budget
    peers.hold_cancel = False
    wait(lambda: len(peers.received) == 2)
    assert stop().accepted
    wait(lambda: status().state == 'canceled')


def test_exploration_defaults_are_valid_without_any_parameters():
    goal, parameters = task_goal('explore', '{}')
    assert parameters == '{}'
    assert goal.revisit_known is False
    assert goal.max_duration_sec == goal.max_radius_m == 0.0
    assert goal.resume_task_id == ''


def test_capability_defaults_and_task_metadata_come_from_explorer(system):
    from std_msgs.msg import String

    peers, start, status, _, restart = system
    gateway = peers.gateway
    publisher = peers.create_publisher(String, '/robot/explore/defaults',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(String(data=json.dumps({'max_duration_sec': 60.0, 'max_radius_m': 3.0})))
    wait(lambda: gateway._exploration_defaults is not None)
    catalog = json.loads(gateway._capabilities(None, Trigger.Response()).message)
    explore = next(item for item in catalog['capabilities'] if item['name'] == 'explore')
    assert explore['required_parameters'] == []
    assert explore['defaults'] == {'revisit_known': False, 'max_duration_sec': 0.0,
                                   'max_radius_m': 0.0, 'resume_task_id': ''}
    assert explore['effective_defaults']['max_duration_sec'] == 60.0
    assert explore['effective_defaults']['max_radius_m'] == 3.0
    assert catalog['terminal_states'] == ['canceled', 'failed', 'succeeded']
    assert start(parameters='{"max_radius_m":2,"revisit_known":true}').accepted
    wait(lambda: status().state == 'running')
    metadata = json.loads(status().result_json)['_task']
    assert metadata['effective_parameters'] == {
        'max_duration_sec': 60.0, 'max_radius_m': 2,
        'revisit_known': True, 'resume_task_id': ''}
    restart()
    assert json.loads(status().result_json)['_task'] == metadata
    peers.finish = True
    wait(lambda: status().state == 'succeeded')
    result = json.loads(status().result_json)
    assert result['complete'] is True
    assert result['_task'] == metadata


def test_missing_explorer_defaults_are_explicit_and_do_not_block_submission(system):
    peers, start, status, _, _ = system
    catalog = json.loads(peers.gateway._capabilities(None, Trigger.Response()).message)
    assert catalog['observations']['services']['query'] == '/robot/observations/query'
    assert not catalog['observations']['available']
    query = catalog['observations']['query']
    assert query['service'] == '/robot/perception/set_query' and query['type'] == 'lekiwi_rmf/srv/SetPerceptionQuery'
    assert not query['available'] and (query['max_labels'], query['max_label_chars']) == (16, 48)
    explore = next(item for item in catalog['capabilities'] if item['name'] == 'explore')
    assert explore['effective_defaults'] is None
    assert start().accepted
    assert json.loads(status().result_json)['_task']['effective_parameters'] is None


@pytest.mark.parametrize('text', ['invalid', '[]', '{}', '{"max_duration_sec":true,"max_radius_m":3}',
                                 '{"max_duration_sec":NaN,"max_radius_m":3}',
                                 '{"max_duration_sec":60,"max_radius_m":-1}',
                                 '[' * 1100 + '0' + ']' * 1100, ' ' * 4097,
                                 '{"max_duration_sec":' + '9' * 400 + ',"max_radius_m":3}'])
def test_invalid_explorer_defaults_are_not_advertised(system, text):
    from std_msgs.msg import String

    peers, _, _, _, _ = system
    peers.gateway._explore_defaults(String(data=text))
    assert peers.gateway._exploration_defaults is None


@pytest.mark.parametrize('parameters', ['{"max_duration_sec":61}', '{"max_radius_m":4}'])
def test_exploration_overrides_above_advertised_limits_are_rejected_before_admission(system, parameters):
    from std_msgs.msg import String

    peers, start, status, _, _ = system
    peers.on_gateway(peers.gateway._explore_defaults,
                     String(data='{"max_duration_sec":60,"max_radius_m":3}'))
    response = start(parameters=parameters)
    assert not response.accepted
    assert 'must be zero or within' in response.detail
    assert not status().found
    assert not peers.received


def test_unsent_exploration_preserves_admitted_limits_after_restart(system):
    from std_msgs.msg import String

    peers, start, status, _, restart = system
    for timer in peers.gateway.timers:
        timer.cancel()
    publisher = peers.create_publisher(String, '/robot/explore/defaults',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(String(data='{"max_duration_sec":60,"max_radius_m":3}'))
    wait(lambda: peers.gateway._exploration_defaults is not None)
    assert start().accepted
    assert not peers.received
    publisher.publish(String(data='{"max_duration_sec":120,"max_radius_m":6}'))
    wait(lambda: peers.gateway._exploration_defaults['max_radius_m'] == 6)
    restart()
    wait(lambda: status().state == 'running')
    assert len(peers.requests) == 1
    assert peers.requests[0].max_duration_sec == 60
    assert peers.requests[0].max_radius_m == 3
    assert start().accepted  # Original omitted arguments still deduplicate.
    assert len(peers.requests) == 1


def test_navigation_deadline_is_enforced_locally_and_retries_keep_identity(system):
    peers, start, status, _, _ = system
    parameters = json.dumps({'x': 1., 'y': 0., 'deadline_unix_sec': time.time() + .5})
    assert start(capability='navigate', parameters=parameters).accepted
    wait(lambda: status().state in ('succeeded', 'canceled', 'failed'))
    assert status().state == 'canceled'
    assert start(capability='navigate', parameters=parameters).accepted


def test_travel_bound_requires_odometry_before_admission(system):
    _, start, _, _, _ = system
    response = start(capability='navigate', parameters='{"x":1,"y":0,"max_travel_m":1}')
    assert not response.accepted and 'odometry' in response.detail
