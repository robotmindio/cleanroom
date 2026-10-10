"""Operator and session recovery on an isolated graph, without motors."""

import threading
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.msg import CollisionMonitorState
import pytest
from std_srvs.srv import Empty

from lekiwi_rmf.action import Explore
from test_exploration_ros import graph as graph, response, start, wait


def test_operator_waits_for_recovering_startup_inputs_without_a_rejected_goal(graph):
    from lekiwi_rmf.explore_client import ExploreClient

    peers, explorer, _ = graph
    peers.publish_slam = False
    wait(lambda: 'slam' in explorer._readiness_reason())
    interrupted = threading.Event()
    node = ExploreClient(context=explorer.context)

    def recover():
        time.sleep(0.5)
        peers.publish_slam = True
        wait(lambda: peers.nav_active)
        interrupted.set()

    worker = threading.Thread(target=recover)
    worker.start()
    try:
        assert node.run(interrupted) == 130
        assert not explorer._rejection
        assert peers.nav_count == 1 and peers.nav_canceled == 1
    finally:
        peers.publish_slam = True
        worker.join(timeout=5)
        assert not worker.is_alive()
        node.destroy_node()


@pytest.mark.parametrize('success', [False, True])
def test_operator_recovers_a_lost_result_reply_from_an_idle_server(graph, monkeypatch, success):
    from lekiwi_rmf.explore_client import ExploreClient
    from rclpy.task import Future

    peers, explorer, _ = graph
    if success:
        peers.grid.data = [0 if 33 <= i % 80 < 47 and 33 <= i // 80 < 47 else 100
                           for i in range(6400)]
        wait(lambda: explorer._map[0][0, 0] == 100)
    else:
        explorer.config['max_duration_sec'] = 0.7
    node = ExploreClient(context=explorer.context)
    # Drop the first result RPC; the real server still completes and retains the result.
    monkeypatch.setattr(node.action, '_get_result_async', lambda _goal: Future())
    try:
        assert node.run(threading.Event()) == (0 if success else 1)
        assert not peers.nav_active and not explorer._busy
    finally:
        node.destroy_node()


@pytest.mark.parametrize('failure', ['overlap', 'invalid_source', 'missing_status'])
def test_collision_stop_pauses_without_discarding_targets_and_clearing_resumes(graph, failure):
    peers, explorer, client = graph
    feedback = []
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5),
                      feedback_callback=lambda msg: feedback.append(msg.feedback)))
    assert handle.accepted
    wait(lambda: peers.nav_active)
    if failure == 'missing_status':
        peers.publish_collision = False
    else:
        peers.collision = CollisionMonitorState(
            action_type=CollisionMonitorState.STOP,
            polygon_name='StopZone' if failure == 'overlap' else 'invalid source')
    wait(lambda: 'paused: collision monitor' in explorer._stage)
    wait(lambda: not peers.nav_active and peers.nav_canceled == 1)
    count = peers.nav_count
    time.sleep(0.3)
    assert peers.nav_count == count
    assert feedback and all(message.unreachable_targets == 0 for message in feedback)
    assert not handle.get_result_async().done()
    peers.collision = CollisionMonitorState()
    peers.publish_collision = True
    wait(lambda: peers.nav_active and peers.nav_count == count + 1)
    response(handle.cancel_goal_async())
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.unreachable_targets == 0


def test_collision_stop_refuses_a_new_goal_but_does_not_poison_readiness_after_clear(graph):
    peers, explorer, client = graph
    peers.collision = CollisionMonitorState(action_type=CollisionMonitorState.STOP, polygon_name='StopZone')
    wait(lambda: 'collision monitor stop: StopZone' in explorer._readiness_reason())
    assert not response(client.send_goal_async(Explore.Goal())).accepted
    assert explorer._readiness_reason(require_permission=False) == ''
    peers.collision = CollisionMonitorState()
    wait(lambda: not explorer._readiness_reason())
    handle = start(graph)
    response(handle.cancel_goal_async())
    assert response(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED


def test_completed_duration_quota_does_not_poison_the_next_explore_goal(graph):
    peers, explorer, client = graph
    explorer.config['mapping_max_seconds'] = 0.6
    peers.set_mode(True, Empty.Response())
    wait(lambda: explorer._mapping is False and explorer._quota_reason
         and explorer._freeze_future is not None and explorer._freeze_future.done())
    explorer.config['mapping_max_seconds'] = 10.0
    handle = start(graph)
    assert not explorer._quota_reason
    response(handle.cancel_goal_async())
    assert response(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED
    (explorer.database).write_bytes(b'x' * explorer.config['mapping_max_bytes'])
    assert not response(client.send_goal_async(Explore.Goal())).accepted


def test_pause_reports_a_changed_blocking_reason(graph):
    peers, explorer, client = graph
    feedback = []
    handle = response(client.send_goal_async(Explore.Goal(max_radius_m=2.5),
                      feedback_callback=lambda msg: feedback.append(msg.feedback.stage)))
    wait(lambda: peers.nav_active)
    peers.permitted = False
    wait(lambda: any('base_motion_permitted' in stage for stage in feedback))
    peers.publish_slam = False
    wait(lambda: 'paused:' in explorer._stage)
    # Restoring permission exposes the next still-active cause.
    time.sleep(explorer.config['slam_timeout_sec'] + 0.2)
    peers.permitted = True
    wait(lambda: any('slam' in stage.lower() for stage in feedback))
    peers.publish_slam = True
    wait(lambda: peers.nav_active)
    response(handle.cancel_goal_async())
    assert response(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED


@pytest.mark.parametrize('accepted', [False, True])
def test_operator_client_returns_failure_for_rejected_or_aborted_goals(graph, accepted, capsys):
    from lekiwi_rmf.explore_client import ExploreClient

    peers, explorer, _ = graph
    if accepted:
        explorer.config['max_duration_sec'] = 0.7
    else:
        peers.permitted = False
        wait(lambda: not explorer._inputs['base_motion_permitted'][0])
    node = ExploreClient(context=explorer.context)
    try:
        code = node.run(threading.Event())
        assert code == (1 if accepted else 2)
        output = capsys.readouterr().out
        assert ('duration reached' if accepted else 'base_motion_permitted') in output
    finally:
        node.destroy_node()


def test_readonly_deployment_check_requires_active_nav2_and_fresh_permission(graph):
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from lekiwi_rmf.stack_readiness import StackReadiness
    from std_msgs.msg import String
    from rclpy.qos import DurabilityPolicy, QoSProfile

    peers, explorer, _ = graph
    state = [State.PRIMARY_STATE_INACTIVE]
    def get_state(_request, reply):
        reply.current_state.id = state[0]
        return reply
    services = [peers.create_service(GetState, f'/{name}/get_state', get_state)
                for name in ('controller_server', 'planner_server', 'bt_navigator')]
    checker = StackReadiness(context=peers.context)
    driver = peers.create_publisher(String, "/safety/driver_state",
                                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    timer = peers.create_timer(0.05, lambda: driver.publish(String(data="ARMED")))
    explorer.executor.add_node(checker)
    try:
        try:
            wait(lambda: checker.inputs.get('explorer', (False,))[0])
        except AssertionError:
            raise AssertionError((checker.inputs, explorer._readiness_reason(require_permission=False)))
        assert 'not confirmed active' in checker.reason(armed=False, exploration=True)
        state[0] = State.PRIMARY_STATE_ACTIVE
        wait(lambda: checker.reason(armed=True, exploration=True) == '')
        peers.permitted = False
        wait(lambda: not checker.inputs['base_motion_permitted'][0])
        wait(lambda: checker.reason(armed=False, exploration=True) == '')
        assert 'base_motion_permitted' in checker.reason(armed=True, exploration=True)
        peers.publish_slam = False
        checker.inputs['slam'] = (True, time.monotonic() - 5)
        assert 'slam' in checker.reason(armed=False, exploration=True)
    finally:
        explorer.executor.remove_node(checker)
        checker.destroy_node()
        peers.destroy_timer(timer)
        peers.destroy_publisher(driver)
        for service in services:
            peers.destroy_service(service)


@pytest.mark.parametrize('outcome', ['success', 'cancel', 'late_acceptance'])
def test_operator_client_success_and_interrupt_confirm_its_owned_stop(graph, outcome):
    from lekiwi_rmf.explore_client import ExploreClient

    peers, explorer, _ = graph
    interrupted = threading.Event()
    node = ExploreClient(context=explorer.context)
    if outcome == 'success':
        # No accessible observation targets: success must not race a revisit budget.
        peers.grid.data = [0 if 33 <= i % 80 < 47 and 33 <= i // 80 < 47 else 100
                           for i in range(6400)]
        wait(lambda: explorer._map[0][0, 0] == 100)
    else:
        if outcome == 'late_acceptance':
            def delayed(request):
                time.sleep(0.6)
                return explorer._accept(request)
            explorer._action.register_goal_callback(delayed)
        def cancel():
            if outcome == 'cancel':
                wait(lambda: peers.nav_active)
            else:
                wait(lambda: node._sent_id is not None)
            interrupted.set()
        worker = threading.Thread(target=cancel)
        worker.start()
    try:
        assert node.run(interrupted) == (0 if outcome == 'success' else 130)
        assert not peers.nav_active and not explorer._busy
        if outcome == 'cancel':
            assert peers.nav_canceled == 1
    finally:
        node.destroy_node()
        if outcome != 'success':
            worker.join(timeout=5)
            assert not worker.is_alive()


def test_lost_freeze_reply_is_retried_even_after_localization_is_observed(graph):
    from rclpy.task import Future

    peers, explorer, client = graph
    explorer._timer.cancel()
    wait(lambda: explorer._mode_future.done())
    stalled = Future()
    explorer._freeze_future = stalled
    explorer._freeze_unconfirmed = True
    explorer._freeze_requested_at = time.monotonic() - explorer.config['service_timeout_sec'] - 1
    explorer._quota_reason = 'mapping session duration reached'
    assert explorer._mapping is False and explorer._mapping_started is None
    assert not response(client.send_goal_async(Explore.Goal())).accepted
    explorer._monitor()
    assert stalled.cancelled()
    wait(lambda: not explorer._freeze_unconfirmed and explorer._freeze_future.done())
    explorer._timer.reset()
    handle = start(graph)
    assert not explorer._quota_reason and peers.mapping_requests[:2] == [False, True]
    response(handle.cancel_goal_async())
    assert response(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED
