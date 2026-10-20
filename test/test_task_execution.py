"""Durable physical execution through the negotiated task API (spec T-40..T-42, T-47..T-52); no motors.

Real ROS services, action servers, owner guard and SQLite journal; the action
servers are test doubles, so these prove the gateway's contract, not robot motion.
"""

import json
import sqlite3
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger

from lekiwi_rmf.action import Explore, RunSkill
from lekiwi_rmf.exploration_owner_guard import ExplorationOwnerGuard
from lekiwi_rmf.object_detector import ObjectDetector
from lekiwi_rmf.perception_query import make_query
from lekiwi_rmf.srv import GetTask, SetPerceptionQuery, SubmitTask, TaskApi
from lekiwi_rmf.task_gateway import BODY_PROFILE, TASK_API, TaskGateway
from ros_test_utils import release_graph
from test_task_gateway import Peers, wait as gateway_wait


def wait(predicate, timeout=20):
    # Each step here chains several service round trips; CI's single-process coverage run is slower.
    gateway_wait(predicate, timeout)


SKILL = {'skill_id': 'insert_peg', 'parameters': {}}


class ExecutionPeers(Peers):
    """The gateway test doubles plus a skill runner."""

    def __init__(self, context):
        super().__init__(context)
        self.skilling = ActionServer(self, RunSkill, '/robot/skills/run', self.skill,
                                     goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                     callback_group=ReentrantCallbackGroup())

    def skill(self, goal):
        return self.execute(goal, RunSkill.Result(outcome='finished', detail='ran to its end', result_json='{}'),
                            lambda: RunSkill.Feedback(stage='chunk'))

    def destroy_node(self):
        self.skilling.destroy()
        return super().destroy_node()


class Robot:
    def __init__(self, tmp_path):
        self.context = Context()
        rclpy.init(context=self.context)
        self.path = str(tmp_path / 'tasks.sqlite3')
        self.peers = ExecutionPeers(self.context)
        self.guard = ExplorationOwnerGuard(context=self.context)
        self.client = Node('execution_client', context=self.context)
        self.resources = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.client.create_subscription(String, '/robot/resources',
                                        lambda message: self.resources.append(json.loads(message.data)), latched)
        self.executor = MultiThreadedExecutor(num_threads=6, context=self.context)
        for node in (self.peers, self.guard, self.client):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.api_clients = {name: self.client.create_client(TaskApi, service) for name, service in TASK_API.items()}
        self.submit_v1 = self.client.create_client(SubmitTask, '/robot/tasks/submit')
        self.get_v1 = self.client.create_client(GetTask, '/robot/tasks/get')
        self.capabilities = self.client.create_client(Trigger, '/robot/capabilities')
        self.gateway = None
        self.start_gateway()
        wait(lambda: all(self.gateway._action_clients[name].server_is_ready() for name in ('explore', 'skill')))

    def start_gateway(self):
        self.gateway = TaskGateway(context=self.context, parameter_overrides=[
            Parameter('database_path', value=self.path), Parameter('allow_skills', value=True)])
        self.gateway_executor = SingleThreadedExecutor(context=self.context)
        self.gateway_executor.add_node(self.gateway)
        self.gateway_thread = threading.Thread(target=self.gateway_executor.spin, daemon=True)
        self.gateway_thread.start()
        wait(lambda: all(client.service_is_ready() for client in self.api_clients.values()))

    def stop_gateway(self):
        release_graph(executor=self.gateway_executor, thread=self.gateway_thread, nodes=(self.gateway,))

    def restart(self):
        self.stop_gateway()
        self.start_gateway()

    def close(self):
        self.peers.finish = True
        # Both executors stop before any node is destroyed, so no callback meets a destroyed peer.
        try:
            release_graph(executor=self.gateway_executor, thread=self.gateway_thread)
        finally:
            release_graph(executor=self.executor, thread=self.thread,
                          nodes=(self.gateway, self.client, self.peers, self.guard), context=self.context)

    def call(self, client, request):
        future = client.call_async(request)
        wait(future.done)
        return future.result()

    def api(self, name, schema=2, **payload):
        reply = self.call(self.api_clients[name], TaskApi.Request(schema=schema, request_json=json.dumps(payload)))
        return SimpleNamespace(ok=reply.ok, error=reply.error, detail=reply.detail, schema=reply.schema,
                               data=json.loads(reply.response_json or '{}'))

    def submit(self, task_id='task-1', capability='explore', parameters=None, owner='agent-a', **extra):
        return self.api('submit', task_id=task_id, capability=capability, parameters=parameters or {},
                        owner=owner, **extra)

    def status(self, task_id='task-1', **extra):
        reply = self.api('status', task_id=task_id, **extra)
        assert reply.ok, reply.detail
        return reply.data['task']

    def legacy_submit(self, task_id, capability, parameters):
        return self.call(self.submit_v1, SubmitTask.Request(task_id=task_id, capability=capability,
                                                            parameters_json=json.dumps(parameters)))

    def journal_rows(self):
        with sqlite3.connect(self.path) as database:
            return [row[0] for row in database.execute('SELECT id FROM tasks')]

    def capability_catalog(self):
        return json.loads(self.call(self.capabilities, Trigger.Request()).message)


@pytest.fixture
def robot(tmp_path):
    robot = Robot(tmp_path)
    yield robot
    robot.close()


def running(robot, task_id='task-1'):
    wait(lambda: robot.status(task_id)['execution']['phase'] == 'running')


def test_capabilities_declare_resources_effects_and_the_negotiated_schema(robot):
    catalog = robot.capability_catalog()
    assert catalog['schema'] == 1 and catalog['task_api']['schemas'] == [1, 2]
    assert catalog['task_api']['body_profile']['id'] == BODY_PROFILE
    entries = {entry['name']: entry for entry in catalog['capabilities']}
    assert entries['search']['resources'] == ['base', 'perception_query']
    assert entries['skill']['resources'] == ['arm', 'base']
    assert entries['skill']['implementation_kinds'] == ['lerobot_policy']
    assert entries['navigate']['effects']['reversibility'] == 'reversible'
    assert entries['skill']['effects']['undeclared']['reversibility'] == 'irreversible'
    assert entries['navigate']['units']['yaw'] == 'rad' and entries['navigate']['frame'] == 'map'
    assert catalog['task_api']['resources'] == {'base': None, 'arm': None, 'perception_query': None}


@pytest.mark.parametrize('capability,parameters,extra,code', [
    ('navigate', {'x': 1, 'y': 0}, {'body_profile': 'lekiwi:0000000000000000'}, 'body_profile_mismatch'),
    ('navigate', {'x': 1, 'y': 0}, {'frame': 'odom'}, 'map_unaligned'),
    ('navigate', {'x': 1, 'y': 0}, {'units': 'imperial'}, 'unit_mismatch'),
    ('navigate', {'x': 1, 'y': 0}, {'resources': ['arm']}, 'resource_mismatch'),
    ('navigate', {'x': 1, 'y': 0, 'max_travel_m': 2}, {}, 'precondition_failed'),
    ('navigate', {'x': 'one', 'y': 0}, {}, 'invalid_request'),
    ('explore', {}, {'authority': 'Bearer ' + 'x' * 200}, 'invalid_request'),
])
def test_t40_incompatible_requests_are_refused_before_dispatch(robot, capability, parameters, extra, code):
    for operation in ('validate', 'submit'):
        reply = robot.api(operation, task_id='bad', capability=capability, parameters=parameters,
                          owner='agent-a', **extra)
        assert not reply.ok and reply.error == code, reply.detail
    assert robot.journal_rows() == [] and robot.peers.received == []


def test_t40_validate_does_not_admit_and_schema_is_negotiated(robot):
    reply = robot.api('validate', task_id='probe', capability='search', parameters={'query': 'cup'},
                      owner='agent-a', resources=['perception_query', 'base'], frame='map', units='si',
                      body_profile=BODY_PROFILE)
    assert reply.ok and reply.data['valid'] and reply.data['existing'] is None
    assert reply.data['request']['resources'] == ['base', 'perception_query']
    assert robot.api('status', task_id='probe').error == 'not_found'
    # A client speaking another schema is told so; the payload is never guessed at.
    for schema in (0, 1, 3):
        refused = robot.api('submit', schema=schema, task_id='probe', capability='explore', owner='agent-a')
        assert not refused.ok and refused.error == 'schema_incompatible' and refused.data['supported'] == [1, 2]
    assert robot.journal_rows() == []


def test_t41_t49_contending_capabilities_are_serialized_by_resource(robot):
    assert robot.submit(owner='agent-a').ok
    running(robot)
    for task_id, capability, parameters in (('skill', 'skill', SKILL), ('search', 'search', {'query': 'cup'})):
        for operation in ('validate', 'submit'):
            refused = robot.api(operation, task_id=task_id, capability=capability, parameters=parameters, owner='agent-b')
            assert refused.error == 'resource_busy', refused.detail
            assert refused.data['holders'][0] == {'resource': 'base', 'task_id': 'task-1', 'owner': 'agent-a',
                                                  'generation': 1, 'state': 'running'}
    legacy = robot.legacy_submit('legacy', 'navigate', {'x': 0, 'y': 0})
    assert not legacy.accepted and 'base by task task-1' in legacy.detail
    wait(lambda: any(state['resources']['base'] for state in robot.resources))
    robot.peers.finish = True
    wait(lambda: robot.status()['state'] == 'succeeded')
    robot.peers.finish = False
    # The released base can now serve a search, which also reserves the perception query.
    assert robot.submit('search', 'search', {'query': 'cup'}, owner='agent-b').ok
    wait(lambda: (robot.resources[-1]['resources']['perception_query'] or {}).get('task_id') == 'search')
    wait(lambda: len(robot.peers.received) == 2)


def test_t49_two_agents_racing_for_overlapping_resources_get_one_grant(robot):
    barrier, replies = threading.Barrier(2), {}

    def race(owner, task_id, capability, parameters):
        barrier.wait()
        replies[owner] = robot.submit(task_id, capability, parameters, owner=owner)

    threads = [threading.Thread(target=race, args=args) for args in (
        ('agent-a', 'skill', 'skill', SKILL), ('agent-b', 'search', 'search', {'query': 'cup'}))]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=10)
    granted = [owner for owner, reply in replies.items() if reply.ok]
    refused = [reply for reply in replies.values() if not reply.ok]
    assert len(granted) == 1 and len(refused) == 1
    assert refused[0].error == 'resource_busy' and refused[0].data['holders'][0]['owner'] == granted[0]
    wait(lambda: len(robot.peers.received) == 1)
    time.sleep(0.5)
    assert len(robot.peers.received) == 1


def test_t47_lost_reply_after_admission_reconciles_without_a_second_command(robot):
    first = robot.submit(goal={'id': 'goal-7', 'revision': 2})
    assert first.ok and first.data['admitted']
    running(robot)
    # The client never saw the reply; it retries with the same identity.
    retry = robot.submit(goal={'id': 'goal-7', 'revision': 2}, owner='agent-a')
    assert retry.ok and retry.data['existing'] and retry.data['task']['generation'] == first.data['task']['generation']
    assert robot.legacy_submit('task-1', 'explore', {}).accepted
    conflict = robot.submit(goal={'id': 'goal-7', 'revision': 3})
    assert not conflict.ok and conflict.error == 'identity_conflict'
    events = robot.api('events', task_id='task-1').data['events']
    assert [event['kind'] for event in events].count('dispatched') == 1
    time.sleep(0.5)
    assert len(robot.peers.received) == 1


def test_t48_restart_between_journal_and_dispatch_never_invents_termination(robot):
    for timer in robot.gateway.timers:
        timer.cancel()
    assert robot.submit().ok
    assert robot.status()['execution']['phase'] == 'admitted'
    # Crash after the dispatch marker committed but before the goal left the process.
    robot.gateway._journal.mark_dispatched('task-1', robot.gateway._server_id('explore'))
    robot.restart()
    wait(lambda: robot.status()['state'] == 'unknown')
    view = robot.status()
    assert view['execution'] == {'phase': 'uncertain', 'dispatched': True, 'outcome': None}
    assert view['stop'] == 'uncertain' and view['holding_resources']
    blocked = robot.submit('task-2', 'navigate', {'x': 0, 'y': 0})
    assert blocked.error == 'execution_uncertain'
    time.sleep(1.5)
    assert robot.status()['state'] == 'unknown' and robot.peers.received == []


def test_t50_stale_generation_and_late_callback_cannot_overwrite_the_new_execution(robot):
    first = robot.submit().data['task']
    running(robot)
    handed = robot.api('transfer', task_id='task-1', generation=first['generation'], owner='agent-a',
                       new_owner='agent-b', checkpoint={'summary': 'agent-b continues the exploration'})
    assert handed.ok and handed.data['generation'] > first['generation']
    stale = robot.api('cancel', task_id='', generation=first['generation'], requester='agent-a')
    assert stale.error == 'stale_generation' and stale.data['owner'] == 'agent-b'
    assert robot.api('transfer', task_id='task-1', generation=first['generation'], owner='agent-a',
                     new_owner='agent-a', checkpoint={'summary': 'take it back'}).error == 'stale_generation'
    # A late result for a superseded attempt UUID arrives after the current attempt started.
    late = Future()
    late.set_result(Explore.Impl.GetResultService.Response(status=GoalStatus.STATUS_SUCCEEDED,
                                                           result=Explore.Result(complete=True)))
    robot.gateway._replies.put(('result', 'task-1', bytes(16), '', late))
    wait(lambda: any(event['kind'] == 'late_reply' for event in robot.api('events', task_id='task-1').data['events']))
    view = robot.status()
    assert view['state'] == 'running' and view['cancel'] is None and view['owner'] == 'agent-b'
    assert robot.api('cancel', task_id='task-1', generation=handed.data['generation'], requester='agent-b').data['accepted']
    wait(lambda: robot.status()['state'] == 'canceled')


def test_t51_cancel_receipt_is_queryable_and_the_stop_is_confirmed_separately(robot):
    robot.peers.hold_cancel = True
    assert robot.submit().ok
    running(robot)
    cancel = robot.api('cancel', task_id='task-1', requester='user')
    assert cancel.ok and cancel.data['accepted'] and cancel.data['stop'] == 'stopping'
    receipt = cancel.data['receipt']
    assert receipt['receipt_id'] and receipt['requester'] == 'user'
    assert robot.api('cancel', task_id='task-1').data['receipt'] == receipt
    time.sleep(0.5)
    view = robot.status()
    # Requested is not stopped: the executor has not confirmed termination yet.
    assert view['stop'] == 'stopping' and view['state'] == 'canceling' and view['cancel'] == receipt
    robot.peers.hold_cancel = False
    wait(lambda: robot.status()['stop'] == 'confirmed')
    view = robot.status()
    assert view['execution']['phase'] == 'stopped' and view['cancel'] == receipt and not view['holding_resources']
    assert view['times']['finished_at'] >= receipt['requested_at']
    assert robot.call(robot.get_v1, GetTask.Request(task_id='task-1')).state == 'canceled'
    kinds = [event['kind'] for event in robot.api('events', task_id='task-1').data['events']]
    assert kinds.count('cancel_requested') == 1 and len(robot.peers.received) == 1


def test_t42_t52_handoff_reconciles_then_compensation_does_not_hide_failure(robot):
    legacy = robot.legacy_submit('weld', 'skill', SKILL)
    assert not legacy.accepted and 'schema 2' in legacy.detail
    first = robot.submit('weld', 'skill', SKILL, owner='agent-a', goal={'id': 'join-parts', 'revision': 1}).data['task']
    assert first['effects']['reversibility'] == 'irreversible'
    running(robot, 'weld')
    checkpoint = {'summary': 'weld started on part 3; verification pending', 'progress': {'chunks': 1},
                  'pending_verifications': ['joint_strength'], 'resources': first['resources']}
    handed = robot.api('transfer', task_id='weld', generation=first['generation'], owner='agent-a',
                       new_owner='agent-b', checkpoint=checkpoint)
    generation = handed.data['generation']
    robot.peers.abort = True
    robot.peers.finish = True
    wait(lambda: robot.status('weld')['state'] == 'failed')
    plan = {'intent': 'isolate', 'steps': [{'capability': 'navigate', 'parameters': {'x': 0, 'y': 0}}],
            'note': 'back away and leave the part for inspection'}
    # The successor has not read the journal yet, so it may not act.
    early = robot.api('compensate', task_id='weld', generation=generation, owner='agent-b', plan=plan)
    assert early.error == 'execution_uncertain'
    events = robot.api('events', task_id='weld', cursor=0, generation=generation).data
    assert 'handoff' in [event['kind'] for event in events['events']]
    view = robot.status('weld')
    assert view['reconciled'] and view['checkpoint'] == checkpoint and view['compensation_required']
    # A material change cannot be rolled back by running the trajectory in reverse.
    restore = robot.api('compensate', task_id='weld', generation=generation, owner='agent-b',
                        plan={'intent': 'restore', 'steps': []})
    assert restore.error == 'irreversible_effect'
    assert robot.api('compensate', task_id='weld', generation=first['generation'], owner='agent-a',
                     plan=plan).error == 'stale_generation'
    recorded = robot.api('compensate', task_id='weld', generation=generation, owner='agent-b', plan=plan)
    assert recorded.ok and recorded.data['task']['compensation']['material_rollback'] is False
    robot.peers.abort = robot.peers.finish = False
    away = robot.submit('away', 'navigate', {'x': 0, 'y': 0}, owner='agent-b',
                        compensates={'task_id': 'weld', 'generation': generation})
    assert away.ok, away.detail
    assert robot.submit('again', 'navigate', {'x': 0, 'y': 0}, owner='agent-b',
                        compensates={'task_id': 'weld', 'generation': generation}).error in (
        'resource_busy', 'precondition_failed')
    running(robot, 'away')
    robot.peers.finish = True
    wait(lambda: robot.status('weld')['compensation']['status'] == 'succeeded')
    view = robot.status('weld')
    assert view['state'] == 'failed' and view['execution']['outcome'] == 'failed'
    assert view['goal'] == {'id': 'join-parts', 'revision': 1, 'verified': None, 'verifier': 'recuriant'}
    assert view['compensation']['counts_as_goal_success'] is False and not view['compensation_required']
    assert robot.call(robot.get_v1, GetTask.Request(task_id='weld')).state == 'failed'


def test_reserved_perception_query_refuses_direct_changes():
    holder = {'task_id': 'search', 'owner': 'agent-a', 'generation': 3, 'state': 'running'}
    query = make_query(['cup'])
    detector = SimpleNamespace(_query_lock=threading.Lock(), _query=query, _query_holder=None,
                               get_logger=lambda: SimpleNamespace(warning=print, info=print, error=print),
                               _detector=SimpleNamespace(default_query=query), _publish_profile=lambda: None)
    message = String(data=json.dumps({'schema': 1, 'resources': {'base': holder, 'perception_query': holder}}))
    ObjectDetector._resources_message(detector, message)
    response = ObjectDetector._set_query(detector, SetPerceptionQuery.Request(labels=['mug']),
                                         SetPerceptionQuery.Response())
    assert not response.success and 'resource_busy' in response.detail and detector._query is query
    ObjectDetector._resources_message(detector, String(data='not json'))
    assert detector._query_holder == {'task_id': 'unknown'}
    ObjectDetector._resources_message(detector, String(data=json.dumps(
        {'schema': 1, 'resources': {'base': None, 'perception_query': None}})))
    response = ObjectDetector._set_query(detector, SetPerceptionQuery.Request(labels=['mug']),
                                         SetPerceptionQuery.Response())
    assert response.success and list(response.labels) == ['mug']
