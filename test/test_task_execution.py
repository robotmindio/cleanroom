"""Durable physical execution through the negotiated task API (spec T-40..T-42, T-47..T-52); no motors.

Real ROS services, action servers, owner guard and SQLite journal; the action
servers are test doubles, so these prove the gateway's contract, not robot motion.
"""

import json
import math
import sqlite3
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rtabmap_msgs.msg import MapGraph
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import yaml

from lekiwi_rmf.action import Explore, RunSkill
from lekiwi_rmf.exploration_owner_guard import ExplorationOwnerGuard
from lekiwi_rmf import object_map
from lekiwi_rmf.object_detector import ObjectDetector
from lekiwi_rmf.perception_query import make_query
from lekiwi_rmf.spatial_memory_node import SpatialMemory
from lekiwi_rmf.srv import GetTask, QuerySpatial, SetPerceptionQuery, SubmitTask, TaskApi
from lekiwi_rmf.task_gateway import BODY_PROFILE, OBJECT_MAP_SCHEMA, TASK_API, TaskGateway
from lekiwi_rmf.task_journal import Refused
from ros_test_utils import release_graph
from test_spatial_memory_ros import graph, rtabmap_database
from test_task_gateway import Peers, wait as gateway_wait


def wait(predicate, timeout=20):
    # Each step here chains several service round trips; CI's single-process coverage run is slower.
    gateway_wait(predicate, timeout)


SKILL = {'skill_id': 'insert_peg', 'parameters': {}}
ARTIFACT = 'a' * 64
REGISTRY = {name: {
    'version': 2, 'description': 'test double', 'status': status, 'kind': 'lerobot_policy',
    'checkpoint': f'{name}/2', 'artifact_sha256': ARTIFACT, 'action_names': ['arm_shoulder_pan'], 'cameras': [],
    'fps': 10, 'max_duration_sec': 5.0, 'max_joint_step_rad': 0.1,
    'parameters': {'depth_m': {'type': 'number', 'min': 0.0, 'max': 0.05}}}
    for name, status in (('insert_peg', 'qualified'), ('trial', 'experimental'))}


class ExecutionPeers(Peers):
    """The gateway test doubles plus a skill runner."""

    def __init__(self, context):
        super().__init__(context)
        self.reject_skill = False
        self.skill_report = '{}'
        self.skilling = ActionServer(self, RunSkill, '/robot/skills/run', self.skill,
                                     goal_callback=lambda _: GoalResponse.REJECT if self.reject_skill
                                     else GoalResponse.ACCEPT,
                                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                                     callback_group=ReentrantCallbackGroup())

    def skill(self, goal):
        return self.execute(goal, RunSkill.Result(outcome='finished', detail='ran to its end',
                                                  result_json=self.skill_report),
                            lambda: RunSkill.Feedback(stage='chunk'))

    def destroy_node(self):
        self.skilling.destroy()
        return super().destroy_node()


class Robot:
    def __init__(self, tmp_path):
        self.context = Context()
        rclpy.init(context=self.context)
        self.path = str(tmp_path / 'tasks.sqlite3')
        self.registry = tmp_path / 'skills.yaml'
        self.registry.write_text(yaml.safe_dump({'skills': REGISTRY}))
        self.peers = ExecutionPeers(self.context)
        self.guard = ExplorationOwnerGuard(context=self.context)
        self.client = Node('execution_client', context=self.context)
        self.extra_nodes = []
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
            Parameter('database_path', value=self.path), Parameter('allow_skills', value=True),
            Parameter('registry_file', value=str(self.registry))])
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
                          nodes=(self.gateway, self.client, self.peers, self.guard, *self.extra_nodes),
                          context=self.context)

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
    # The execution phase includes dispatch before the action acceptance reply.
    wait(lambda: robot.status(task_id)['state'] == 'running')


def test_capabilities_declare_resources_effects_and_the_negotiated_schema(robot):
    catalog = robot.capability_catalog()
    assert catalog['schema'] == 1 and catalog['task_api']['schemas'] == [1, 2]
    assert catalog['task_api']['body_profile']['id'] == BODY_PROFILE
    entries = {entry['name']: entry for entry in catalog['capabilities']}
    assert entries['search']['resources'] == ['base', 'perception_query']
    assert entries['skill']['resources'] == ['arm', 'base']
    assert entries['skill']['implementation_kinds'] == ['lerobot_policy']
    # Only what admission accepts is advertised: the qualified entry, with the version a plan can pin.
    assert [(skill['skill_id'], skill['version']) for skill in entries['skill']['skills']] == [('insert_peg', 2)]
    assert catalog['observations']['services']['prepare'] == '/robot/observations/prepare'
    # The gateway states what it can observe: each observation carries its own grounding, and the
    # object map and the robot's pose are advertised with the availability of their services.
    assert catalog['observations']['grounding'] == 'per_observation'
    assert OBJECT_MAP_SCHEMA == object_map.SCHEMA
    assert {name: catalog['objects'][name] for name in catalog['objects'] if name != 'description'} == {
        'available': False, 'schema': 'lekiwi.object_map/1', 'service': '/robot/objects/query',
        'type': 'lekiwi_rmf/srv/QuerySpatial', 'estimates': 'local_landmark', 'verification': 'last_observed',
        'qualified': 'per_object', 'navigate_target': 'object'}
    assert catalog['spatial']['service'] == '/robot/spatial/query' and not catalog['spatial']['available']
    assert catalog['spatial']['pose'] == {'request': {'op': 'pose'}}
    assert entries['navigate']['targets']['object'] == {
        'available': False, 'service': '/robot/objects/query', 'map': '/map', 'max_sigma_ratio': 0.5}
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


@pytest.mark.parametrize('parameters,code', [
    ({'skill_id': 'unknown_skill'}, 'competence_insufficient'),
    ({'skill_id': 'trial'}, 'competence_insufficient'),
    ({'skill_id': 'insert_peg', 'version': 1}, 'competence_insufficient'),
    ({'skill_id': 'insert_peg', 'artifact_sha256': 'b' * 64}, 'competence_insufficient'),
    ({'skill_id': 'insert_peg', 'parameters': {'depth_m': 0.5}}, 'invalid_request'),
    ({'skill_id': 'insert_peg', 'parameters': {'speed': 1}}, 'invalid_request'),
    ({'skill_id': 'insert_peg', 'version': '2'}, 'invalid_request'),
])
def test_a_skill_the_runner_would_reject_is_refused_before_anything_is_held(robot, parameters, code):
    for operation in ('validate', 'submit'):
        reply = robot.api(operation, task_id='bad', capability='skill', parameters=parameters, owner='agent-a')
        assert not reply.ok and reply.error == code, reply.detail
    assert robot.journal_rows() == [] and robot.peers.received == []
    assert robot.capability_catalog()['task_api']['resources']['arm'] is None


def test_a_qualified_skill_is_admitted_with_the_pins_its_plan_was_made_for(robot):
    pinned = {'skill_id': 'insert_peg', 'parameters': {'depth_m': 0.01}, 'version': 2, 'artifact_sha256': ARTIFACT}
    reply = robot.api('validate', task_id='peg', capability='skill', parameters=pinned, owner='agent-a')
    assert reply.ok and reply.data['valid'], reply.detail
    assert robot.submit('peg', 'skill', pinned).ok
    running(robot, 'peg')
    assert json.loads(robot.peers.requests[0].parameters_json) == {'depth_m': 0.01}
    assert robot.status('peg')['detail'] == 'chunk'


def test_a_goal_the_runner_rejects_leaves_nothing_to_compensate(robot):
    robot.peers.reject_skill = True
    assert robot.submit('weld', 'skill', SKILL).ok
    wait(lambda: robot.status('weld')['state'] == 'failed')
    view = robot.status('weld')
    assert view['result'] == {'goal_rejected': True, 'reason_code': 'goal_rejected'}
    assert view['reason_code'] == 'goal_rejected' and not view['compensation_required']
    assert not view['holding_resources'] and robot.peers.received == []


@pytest.mark.parametrize('report,abort', [('{"arm_stopped": false}', False), ('{"arm_stopped": false}', True),
                                          ('not json', False)])
def test_a_skill_that_ends_without_a_confirmed_arm_stop_keeps_its_resources(robot, report, abort):
    robot.peers.skill_report, robot.peers.abort, robot.peers.finish = report, abort, True
    assert robot.submit('weld', 'skill', SKILL).ok
    wait(lambda: robot.status('weld')['state'] == 'unknown')
    view = robot.status('weld')
    assert 'did not confirm the arm stopped' in view['detail'] and view['result']['arm_stopped'] is False
    assert view['reason_code'] == 'arm_stop_unconfirmed'
    assert view['stop'] == 'uncertain' and view['execution']['phase'] == 'uncertain' and view['holding_resources']
    assert robot.capability_catalog()['task_api']['resources']['arm']['task_id'] == 'weld'
    for capability, parameters in (('skill', SKILL), ('navigate', {'x': 0, 'y': 0})):
        assert robot.submit('next', capability, parameters).error == 'execution_uncertain'
    # Neither a cancel, the server's repeated answer nor a gateway restart turns it into a stop.
    assert robot.api('cancel', task_id='weld').data['stop'] in ('stopping', 'uncertain')
    robot.restart()
    time.sleep(4.5)
    view = robot.status('weld')
    assert view['state'] in ('unknown', 'canceling') and view['stop'] != 'confirmed' and view['holding_resources']
    assert robot.submit('next', 'navigate', {'x': 0, 'y': 0}).error in ('execution_uncertain', 'resource_busy')


def test_a_skill_whose_runner_confirms_or_omits_the_arm_stop_releases_on_its_terminal_result(robot):
    for task_id, report in (('old-runner', '{}'), ('confirmed', '{"arm_stopped": true}')):
        robot.peers.skill_report, robot.peers.finish = report, True
        assert robot.submit(task_id, 'skill', SKILL).ok
        wait(lambda: robot.status(task_id)['state'] == 'succeeded')
        view = robot.status(task_id)
        assert view['stop'] == 'confirmed' and not view['holding_resources']


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


@pytest.mark.parametrize('capability,parameters', [('explore', {}), ('skill', SKILL)])
def test_t48_restart_between_journal_and_dispatch_never_invents_an_outcome(robot, capability, parameters):
    for timer in robot.gateway.timers:
        timer.cancel()
    assert robot.submit(capability=capability, parameters=parameters).ok
    assert robot.status()['execution']['phase'] == 'admitted'
    # Crash after the dispatch marker committed but before the goal left the process.
    robot.gateway._journal.mark_dispatched('task-1', robot.gateway._server_id(capability))
    robot.restart()
    wait(lambda: robot.status()['state'] == 'unknown')
    view = robot.status()
    assert view['execution'] == {'phase': 'uncertain', 'dispatched': True, 'outcome': None}
    assert view['stop'] == 'uncertain' and view['holding_resources']
    blocked = robot.submit('task-2', 'navigate', {'x': 0, 'y': 0})
    assert blocked.error == 'execution_uncertain'
    # The server that would have run it denies the goal to its result and cancel services, and the
    # process that might have sent it is gone: termination is proven, the outcome is not, nothing is replayed.
    wait(lambda: robot.status()['state'] == 'failed')
    view = robot.status()
    assert view['result'] == {'outcome_known': False, 'termination_confirmed': True,
                              'reason_code': 'outcome_unknown'}
    assert view['execution']['outcome'] == 'unknown' and not view['holding_resources']
    assert view['times']['finished_at'] - view['times']['dispatched_at'] >= 3.0
    assert robot.peers.received == []
    assert robot.submit('task-2', 'navigate', {'x': 0, 'y': 0}).ok


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
    # A lost reply is retried with the same plan; another plan never replaces the recorded one.
    assert robot.api('compensate', task_id='weld', generation=generation, owner='agent-b', plan=plan).ok
    assert robot.api('compensate', task_id='weld', generation=generation, owner='agent-b',
                     plan={**plan, 'steps': []}).error == 'identity_conflict'
    compensates = {'task_id': 'weld', 'generation': generation}
    # A step is the plan's next step with exactly its parameters, not any task of that capability.
    for capability, parameters in (('navigate', {'x': 5, 'y': 0}), ('explore', {})):
        for operation in ('validate', 'submit'):
            other = robot.api(operation, task_id='elsewhere', capability=capability, parameters=parameters,
                              owner='agent-b', compensates=compensates)
            assert other.error == 'precondition_failed' and other.data == {
                'step': 0, 'retryable': False, 'next_condition': 'fix_request'}, other.detail
    # A failed step leaves the plan open: a new task runs the same step again.
    assert robot.submit('away-1', 'navigate', {'x': 0, 'y': 0}, owner='agent-b', compensates=compensates).ok
    wait(lambda: robot.status('weld')['compensation']['status'] == 'failed')
    robot.peers.abort = robot.peers.finish = False
    away = robot.submit('away', 'navigate', {'x': 0, 'y': 0}, owner='agent-b', compensates=compensates)
    assert away.ok, away.detail
    assert robot.status('weld')['compensation']['status'] == 'in_progress'
    assert robot.submit('again', 'navigate', {'x': 0, 'y': 0}, owner='agent-b',
                        compensates=compensates).error == 'precondition_failed'
    running(robot, 'away')
    robot.peers.finish = True
    wait(lambda: robot.status('weld')['compensation']['status'] == 'succeeded')
    assert robot.status('weld')['compensation']['tasks'] == {'away-1': 'failed', 'away': 'succeeded'}
    assert robot.submit('more', 'navigate', {'x': 0, 'y': 0}, owner='agent-b',
                        compensates=compensates).error == 'precondition_failed'
    started = [event['data'] for event in robot.api('events', task_id='weld').data['events']
               if event['kind'] == 'compensation_started']
    assert started == [{'task_id': 'away-1', 'step': 0}, {'task_id': 'away', 'step': 0}]
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


# Requests that state every parameter of every capability; the manifest decides what happens at each bound.
EXAMPLES = {
    'navigate': [{'x': 1.0, 'y': 2.0, 'yaw': 0.5, 'deadline_unix_sec': 60, 'max_travel_m': 5},
                 {'target': {'pose': {'x': 1, 'y': 2, 'yaw': 0.1}}},
                 {'target': {'anchored': {'map_pose': {'x': 1, 'y': 2, 'yaw': 0.1}, 'site_ref': 'home',
                                          'map_ref': 'map-1', 'revision': 'revision-1'}}},
                 {'target': {'point': {'x': 1, 'y': 2}, 'standoff_m': 0.6}},
                 {'target': {'object': {'object_id': 'c' * 32, 'standoff_m': 0.6}}}],
    'explore': [{'revisit_known': True, 'max_duration_sec': 5, 'max_radius_m': 2}, {'resume_task_id': 'a' * 32}],
    'search': [{'query': 'cup', 'region': {'x': 0, 'y': 0, 'radius_m': 1}, 'max_duration_sec': 5,
                'max_travel_m': 2, 'target_coverage': 0.5}],
    'skill': [{'skill_id': 'insert_peg', 'parameters': {}, 'version': 2, 'artifact_sha256': ARTIFACT}],
    'reach': [{'joints': {'arm_shoulder_pan': 0.2, 'arm_shoulder_lift': 0.5, 'arm_elbow_flex': -0.4,
                          'arm_wrist_flex': 0.3, 'arm_wrist_roll': 0.0}}, {'named': 'home'}],
    'gripper': [{'state': 'open'}, {'position_rad': 0.5}],
}
ABSENT = object()


def stated(schema, values, path=()):
    """Every (path, descriptor, enclosing descriptor, enclosing values) of a request."""
    for name, value in values.items():
        item = schema['properties'][name]
        yield (*path, name), item, schema, values
        if 'properties' in item:
            yield from stated(item, value, (*path, name))


def declared(schema, path=()):
    for name, item in schema['properties'].items():
        yield (*path, name)
        if 'properties' in item:
            yield from declared(item, (*path, name))


def changed(values, path, value):
    values = json.loads(json.dumps(values))
    holder = values
    for name in path[:-1]:
        holder = holder[name]
    if value is ABSENT:
        del holder[path[-1]]
    else:
        holder[path[-1]] = value
    return values


def boundaries(item):
    """What a parameter descriptor promises: (value, accepted) at and just past each bound it states."""
    cases = [(wrong, False) for wrong in {
        'number': ('1', True, None, float('nan')), 'integer': (1.5, True, '1'), 'boolean': (1, 'true'),
        'string': (7, None), 'object': ([], 'x')}[item['type']]]
    ahead = item.get('max_ahead_sec')
    if 'min' in item:
        cases += [(item['min'], True), (math.nextafter(item['min'], -math.inf), False)]
    if 'exclusive_min' in item:
        cases.append((item['exclusive_min'], False))
        if not ahead:
            cases.append((math.nextafter(item['exclusive_min'], math.inf), True))
    if 'max' in item:
        cases += [(item['max'], True), (math.nextafter(item['max'], math.inf), False)]
    if 'enum' in item:
        cases += [*((member, True) for member in item['enum']), ('no such member', False)]
    if 'pattern' in item:
        cases.append(('!', False))
    if 'max_length' in item:
        cases += [('x' * item['max_length'], True), ('x' * (item['max_length'] + 1), False)]
    if item.get('min_length'):
        cases.append(('   ', False))
    if ahead:
        cases += [(time.time() + ahead - 5, True), (time.time() + ahead + 5, False), (time.time() - 5, False)]
    return cases


def test_validation_enforces_exactly_the_schema_the_manifest_publishes(tmp_path):
    context = Context()
    rclpy.init(context=context)
    gateway = TaskGateway(context=context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'tasks.sqlite3'))])
    # Tracked limits of this deployment are part of the published schema.
    gateway._exploration_defaults = {'max_duration_sec': 60.0, 'max_radius_m': 3.0}
    gateway._search_defaults = {'max_duration_sec': 60.0, 'max_travel_m': 20.0, 'target_coverage': 0.9}

    def refusal(capability, values):
        # Nothing can run here, so a request the schema accepts ends at `capability_unavailable`.
        with pytest.raises(Refused) as caught:
            gateway._admit('probe', capability, json.dumps(values), request={'owner': 'agent-a'}, dry_run=True)
        return caught.value

    try:
        manifest = json.loads(gateway._capabilities(None, Trigger.Response()).message)
        entries = {entry['name']: entry for entry in manifest['capabilities']}
        assert set(entries) == set(EXAMPLES)
        assert entries['explore']['schema']['properties']['max_radius_m']['max'] == 3.0
        assert entries['search']['schema']['properties']['max_travel_m']['max'] == 20.0
        for capability, examples in EXAMPLES.items():
            schema, seen = entries[capability]['schema'], set()
            for example in examples:
                if 'deadline_unix_sec' in example:
                    example = {**example, 'deadline_unix_sec': time.time() + example['deadline_unix_sec']}
                assert refusal(capability, example).code == 'capability_unavailable', (capability, example)
                for path, item, parent, siblings in stated(schema, example):
                    seen.add(path)
                    field = '.'.join(path)
                    for value, valid in boundaries(item):
                        error = refusal(capability, changed(example, path, value))
                        assert (error.code == 'capability_unavailable') == valid, (field, value, str(error))
                        assert valid or (error.code in ('invalid_request', 'precondition_failed')
                                         and error.data['field'] == field), (field, value, error.data)
                    chosen = next((group['required'] for group in parent.get('alternatives', [])
                                   if set(group['required']) <= set(siblings)), [])
                    omitted = refusal(capability, changed(example, path, ABSENT))
                    required = bool(item.get('required')) or path[-1] in chosen
                    assert (omitted.code == 'invalid_request') == required, field
                    unknown = refusal(capability, changed(example, (*path[:-1], 'unexpected'), 1))
                    assert unknown.code == 'invalid_request', field
                    assert unknown.data['field'] == '.'.join((*path[:-1], 'unexpected'))
            # A parameter added to a descriptor without an example here would be untested, so it fails.
            assert seen == set(declared(schema)), capability
        excluded = refusal('explore', {'resume_task_id': 'a' * 32, 'max_radius_m': 1})
        assert excluded.code == 'invalid_request' and excluded.data['field'] == 'max_radius_m'
        assert entries['explore']['schema']['excludes'] == {'resume_task_id': ['max_duration_sec', 'max_radius_m']}
        # The legacy keys are views of the same descriptor.
        navigate = entries['navigate']
        assert navigate['parameters'] == ['x', 'y', 'yaw', 'deadline_unix_sec', 'max_travel_m', 'target']
        assert navigate['required_parameters'] == ['x', 'y'] and navigate['defaults'] == {'yaw': 0.0}
        assert navigate['schema']['properties']['max_travel_m'] == {
            'type': 'number', 'unit': 'm', 'exclusive_min': 0, 'max': 10}
        assert navigate['schema']['properties']['deadline_unix_sec']['max_ahead_sec'] == 300
        assert navigate['schema']['resources'] == navigate['resources'] == ['base']
        assert navigate['schema']['preconditions'] == [
            {'condition': 'capability_enabled', 'switch': 'allow_navigation', 'satisfied': True},
            {'condition': 'action_server_ready', 'action': '/navigate_to_pose', 'satisfied': False}]
        assert not navigate['available'] and 'budget_exhausted' in navigate['schema']['reason_codes']
        assert entries['gripper']['schema']['preconditions'][0] == {
            'condition': 'capability_enabled', 'switch': 'allow_gripper', 'satisfied': False}
        assert manifest['refusals']['resource_busy'] == {'retryable': True, 'next_condition': 'wait_resource_free'}
        assert {'collision_stop', 'executor_paused'} <= set(manifest['reason_codes']['paused'])
        assert {'found', 'goal_rejected', 'outcome_unknown'} <= set(manifest['reason_codes']['terminal'])
    finally:
        gateway.destroy_node()
        context.try_shutdown()


def test_a_refusal_says_whether_retrying_helps_and_which_field_is_at_fault(robot):
    wait(lambda: robot.gateway._action_clients['navigate'].server_is_ready())
    fix, enable = {'retryable': False, 'next_condition': 'fix_request'}, {'retryable': False,
                                                                           'next_condition': 'enable_capability'}
    for capability, parameters, code, data in (
            ('navigate', {'x': 1, 'y': 0, 'max_travel_m': 11}, 'invalid_request', {**fix, 'field': 'max_travel_m'}),
            ('navigate', {'x': 1, 'y': 0, 'deadline_unix_sec': time.time() - 1}, 'precondition_failed',
             {**fix, 'field': 'deadline_unix_sec'}),
            ('navigate', {'x': 1, 'y': 0, 'max_travel_m': 2}, 'precondition_failed',
             {'retryable': True, 'next_condition': 'wait_odometry'}),
            ('navigate', {'target': {'point': {'x': 1}}}, 'invalid_request', {**fix, 'field': 'target.point.y'}),
            ('reach', {'named': 'home'}, 'capability_unavailable', enable),
            ('gripper', {'state': 'open'}, 'capability_unavailable', enable),
            ('fly', {}, 'invalid_request', {**fix, 'field': 'capability'})):
        reply = robot.api('validate', task_id='probe', capability=capability, parameters=parameters, owner='agent-a')
        assert (reply.error, reply.data) == (code, data), (capability, reply.detail)
    assert robot.submit().ok
    busy = robot.submit('second', 'navigate', {'x': 0, 'y': 0})
    assert busy.error == 'resource_busy'
    assert busy.data['retryable'] is True and busy.data['next_condition'] == 'wait_resource_free'
    # A request the service cannot even parse is still a structured refusal.
    malformed = robot.call(robot.api_clients['status'], TaskApi.Request(schema=2, request_json='{'))
    assert malformed.error == 'invalid_request' and json.loads(malformed.response_json) == fix


def occupancy(value, size=100, resolution=0.05):
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution, grid.info.width, grid.info.height = resolution, size, size
    grid.info.origin.position.x = grid.info.origin.position.y = -size * resolution / 2
    grid.info.origin.orientation.w = 1.0
    grid.data = [value] * (size * size)
    return grid


def arrived(robot, task_id):
    """A target is an ordinary navigation: Nav2's result ends it, with the same evidence and reason."""
    robot.peers.finish = True
    wait(lambda: robot.status(task_id)['state'] == 'succeeded')
    view = robot.status(task_id)
    assert view['reason_code'] == 'succeeded' and view['effect']['kind'] == 'pose_reached'
    assert view['effective_parameters']['resolved']['target'] in ('pose', 'point', 'anchored', 'object')


def localize(robot, x=-1.0):
    """Keep a fresh map to base_footprint transform on the graph."""
    broadcaster = TransformBroadcaster(robot.peers)

    def send():
        transform = TransformStamped()
        transform.header.stamp = robot.peers.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
        transform.transform.translation.x, transform.transform.rotation.w = x, 1.0
        broadcaster.sendTransform(transform)

    robot.peers.create_timer(0.05, send)


def test_a_pose_target_runs_as_the_pose_it_names(robot):
    wait(lambda: robot.gateway._action_clients['navigate'].server_is_ready())
    both = robot.submit('both', 'navigate', {'x': 0, 'y': 0, 'target': {'pose': {'x': 1, 'y': 2}}})
    assert both.error == 'invalid_request' and robot.journal_rows() == []
    reply = robot.submit('go', 'navigate', {'target': {'pose': {'x': 1, 'y': 2, 'yaw': 0.5}}})
    assert reply.ok, reply.detail
    assert reply.data['task']['parameters'] == {'target': {'pose': {'x': 1, 'y': 2, 'yaw': 0.5}}}
    assert reply.data['task']['effective_parameters'] == {
        'x': 1, 'y': 2, 'yaw': 0.5, 'max_duration_sec': 180.0, 'resolved': {'target': 'pose'}}
    wait(lambda: len(robot.peers.requests) == 1)
    pose = robot.peers.requests[0].pose
    assert pose.header.frame_id == 'map' and (pose.pose.position.x, pose.pose.position.y) == (1.0, 2.0)
    assert 2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w) == pytest.approx(0.5)
    arrived(robot, 'go')


def test_a_point_target_becomes_a_clear_standoff_pose_facing_it(robot):
    wait(lambda: robot.gateway._action_clients['navigate'].server_is_ready())
    target = {'target': {'point': {'x': 1.0, 'y': 0.0}, 'standoff_m': 0.6}}

    def validate():
        return robot.api('validate', task_id='near', capability='navigate', parameters=target, owner='agent-a')

    lost = validate()
    assert (lost.error, lost.data) == ('precondition_failed', {
        'retryable': True, 'next_condition': 'needs_localization', 'field': 'target.point'})
    localize(robot)
    wait(lambda: validate().data['next_condition'] == 'needs_map')
    maps = robot.peers.create_publisher(OccupancyGrid, '/map',
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait(lambda: robot.capability_catalog()['capabilities'][0]['targets']['point']['available'])
    maps.publish(occupancy(-1))
    unmapped = validate()
    assert (unmapped.error, unmapped.data) == ('precondition_failed', {
        'retryable': True, 'next_condition': 'needs_map', 'field': 'target.point'})
    maps.publish(occupancy(100))
    wait(lambda: validate().data.get('next_condition') == 'change_standoff')
    blocked = validate()
    assert (blocked.error, blocked.data) == ('precondition_failed', {
        'retryable': False, 'next_condition': 'change_standoff', 'field': 'target.standoff_m'})
    assert robot.journal_rows() == [] and robot.peers.requests == []
    maps.publish(occupancy(0))
    wait(lambda: validate().ok)
    reply = robot.submit('near', 'navigate', target)
    assert reply.ok, reply.detail
    effective = reply.data['task']['effective_parameters']
    # The robot is west of the point, so it stops on the west side and faces east.
    assert (effective['x'], effective['y'], effective['yaw']) == pytest.approx((0.4, 0.0, 0.0), abs=1e-9)
    assert effective['resolved'] == {'target': 'point', 'point': [1.0, 0.0], 'standoff_m': 0.6, 'robot': [-1.0, 0.0]}
    assert effective['max_duration_sec'] == 180.0 and 'target' not in effective
    wait(lambda: len(robot.peers.requests) == 1)
    sent = robot.peers.requests[0].pose.pose
    assert (sent.position.x, sent.position.y) == pytest.approx((0.4, 0.0), abs=1e-9)
    assert sent.orientation.z == pytest.approx(0.0, abs=1e-9)
    arrived(robot, 'near')


def test_an_anchored_target_follows_its_anchor_into_the_current_map_revision(robot, tmp_path):
    wait(lambda: robot.gateway._action_clients['navigate'].server_is_ready())
    remembered = {'map_pose': {'x': 2.0, 'y': 0.0, 'yaw': 0.3}, 'site_ref': 'home', 'map_ref': 'm', 'revision': 'r'}

    def validate(anchored):
        return robot.api('validate', task_id='back', capability='navigate',
                         parameters={'target': {'anchored': anchored}}, owner='agent-a')

    absent = validate(remembered)
    assert (absent.error, absent.data) == ('precondition_failed', {
        'retryable': True, 'next_condition': 'wait_spatial_memory', 'field': 'target.anchored'})
    database = tmp_path / 'rtabmap.db'
    rtabmap_database(database)
    spatial = SpatialMemory(context=robot.context, parameter_overrides=[
        Parameter('database_path', value=str(tmp_path / 'spatial.db')),
        Parameter('rtabmap_database', value=str(database)), Parameter('site_ref', value='home'),
        Parameter('graph_topic', value='/test/mapGraph'), Parameter('info_topic', value='/test/info')])
    robot.extra_nodes.append(spatial)
    robot.executor.add_node(spatial)
    graphs = robot.peers.create_publisher(MapGraph, '/test/mapGraph', 1)
    revisions = []
    robot.client.create_subscription(String, '/robot/spatial/revision',
                                     lambda message: revisions.append(json.loads(message.data)),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait(lambda: graphs.get_subscription_count() == 1
         and robot.capability_catalog()['capabilities'][0]['targets']['anchored']['available'])
    unmapped = validate(remembered)
    assert unmapped.error == 'precondition_failed' and unmapped.data['next_condition'] == 'needs_map'
    now = robot.peers.get_clock().now().to_msg().sec
    graphs.publish(graph(now - 5))
    wait(lambda: revisions)
    first = revisions[-1]
    remembered = {**remembered, 'map_ref': first['map_ref'], 'revision': first['revision']}
    same = validate(remembered)
    assert same.ok, same.detail
    assert same.data['request']['effective_parameters']['y'] == pytest.approx(0.0, abs=1e-6)
    for change, code in (({'site_ref': 'office'}, 'site_mismatch'), ({'revision': 'never-seen'}, 'revision_unknown'),
                         ({'map_ref': 'another-map'}, 'revision_unknown')):
        refused = validate({**remembered, **change})
        assert refused.error == code and refused.data == {
            'retryable': False, 'next_condition': 'refresh_reference', 'field': 'target.anchored',
            'spatial_code': code}, refused.detail
    # The map is optimized: node 2, the pose's anchor, moves 0.2 m sideways and the pose follows it.
    graphs.publish(graph(now - 4, shift_y=0.1))
    wait(lambda: revisions[-1]['revision'] != first['revision'])
    reply = robot.submit('back', 'navigate', {'target': {'anchored': remembered}})
    assert reply.ok, reply.detail
    effective = reply.data['task']['effective_parameters']
    assert (effective['x'], effective['y'], effective['yaw']) == pytest.approx((2.0, 0.2, 0.3), abs=1e-6)
    assert effective['resolved']['target'] == 'anchored' and effective['resolved']['method'] == 'anchor'
    assert effective['resolved']['revision'] == revisions[-1]['revision']
    assert effective['resolved']['capture_revision'] == first['revision']
    assert effective['resolved']['shift_m'] == [pytest.approx(0.2, abs=1e-6)]
    wait(lambda: len(robot.peers.requests) == 1)
    assert robot.peers.requests[0].pose.pose.position.y == pytest.approx(0.2, abs=1e-6)
    arrived(robot, 'back')


CUP = 'c' * 32


def cup(**changes):
    """An object-map entry as /robot/objects/query documents it."""
    return {'object_id': CUP, 'label': 'cup', 'state': 'present', 'site_ref': 'home', 'map_ref': 'm',
            'revision': 'r2', 'position': {'x': 1.0, 'y': 0.0, 'z': 0.1}, 'bearings': [],
            'method': 'ray_octomap_hit', 'sigma_m': 0.09, 'qualified': False,
            'last_seen_unix_sec': 1700000000.0, **changes}


def object_answer(*entries, **changes):
    return {'schema': OBJECT_MAP_SCHEMA, 'site_ref': 'home', 'localized': True,
            'map': {'map_ref': 'm', 'revision': 'r2'}, 'verification': 'last_observed', 'truncated': False,
            'objects': list(entries), **changes}


def test_an_object_target_approaches_its_last_observed_estimate_and_says_what_it_is(robot):
    wait(lambda: robot.gateway._action_clients['navigate'].server_is_ready())
    target = {'target': {'object': {'object_id': CUP, 'standoff_m': 0.6}}}

    def validate():
        return robot.api('validate', task_id='cup', capability='navigate', parameters=target, owner='agent-a')

    silent = validate()
    assert (silent.error, silent.data) == ('precondition_failed', {
        'retryable': True, 'next_condition': 'wait_object_map', 'field': 'target.object'})
    answer, asked = [object_answer()], []

    def serve(request, response):
        asked.append(json.loads(request.request_json))
        response.success, response.response_json = True, json.dumps(answer[0])
        return response

    robot.peers.create_service(QuerySpatial, '/robot/objects/query', serve)
    localize(robot)
    maps = robot.peers.create_publisher(OccupancyGrid, '/map',
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    maps.publish(occupancy(0))
    wait(lambda: robot.capability_catalog()['capabilities'][0]['targets']['object']['available'])
    assert robot.capability_catalog()['objects']['available']
    again = {'retryable': True, 'next_condition': 'run_search', 'field': 'target.object'}
    stale = {'retryable': False, 'next_condition': 'refresh_reference', 'field': 'target.object'}
    broken = {'retryable': False, 'next_condition': 'fix_deployment', 'field': 'target.object'}
    for case, code, data in (
            (object_answer(cup(object_id='d' * 32)), 'not_found',
             {'retryable': False, 'next_condition': 'query_objects', 'field': 'target.object.object_id'}),
            (object_answer(cup(state='absent_at_last_place')), 'precondition_failed',
             {**again, 'object_state': 'absent_at_last_place'}),
            (object_answer(cup(position=None, method='bearing_only', sigma_m=None)), 'precondition_failed',
             {**again, 'method': 'bearing_only'}),
            (object_answer(cup(position=None, revision=None, position_unavailable_reason='revision_unknown')),
             'revision_unknown', {**stale, 'spatial_code': 'revision_unknown'}),
            (object_answer(cup(position=None, revision=None, position_unavailable_reason='anchor_unknown')),
             'precondition_failed', {**stale, 'spatial_code': 'anchor_unknown'}),
            (object_answer(cup(map_ref='another-map')), 'map_unaligned', stale),
            (object_answer(cup(), localized=False, map=None), 'precondition_failed',
             {'retryable': True, 'next_condition': 'needs_localization', 'field': 'target.object'}),
            # Uncertain by more than half the standoff: a wider standoff admits 0.4 m, nothing admits 2 m.
            (object_answer(cup(sigma_m=0.4)), 'precondition_failed',
             {'retryable': False, 'next_condition': 'change_standoff', 'field': 'target.object.standoff_m',
              'sigma_m': 0.4, 'max_sigma_m': 0.3}),
            (object_answer(cup(sigma_m=2.0)), 'precondition_failed', {**again, 'sigma_m': 2.0, 'max_sigma_m': 0.3}),
            (object_answer(cup(), schema='lekiwi.object_map/2'), 'precondition_failed', broken),
            (object_answer(cup(sigma_m='narrow')), 'precondition_failed', broken)):
        answer[0] = case
        refused = validate()
        assert (refused.error, refused.data) == (code, data), refused.detail
    assert robot.journal_rows() == [] and robot.peers.requests == []
    # An unconfirmed entry is still the best estimate; the caller is told its state.
    answer[0] = object_answer(cup(object_id='d' * 32), cup(state='unconfirmed'))
    reply = robot.submit('cup', 'navigate', target)
    assert reply.ok, reply.detail
    assert asked[-1] == {'object_id': CUP, 'limit': 1}
    effective = reply.data['task']['effective_parameters']
    # The same standoff pose as the point target at the entry's x, y.
    assert (effective['x'], effective['y'], effective['yaw']) == pytest.approx((0.4, 0.0, 0.0), abs=1e-9)
    assert effective['resolved'] == {
        'target': 'object', 'point': [1.0, 0.0], 'standoff_m': 0.6, 'robot': [-1.0, 0.0],
        'object_id': CUP, 'label': 'cup', 'state': 'unconfirmed', 'method': 'ray_octomap_hit', 'sigma_m': 0.09,
        'qualified': False, 'last_seen_unix_sec': 1700000000.0, 'map_ref': 'm', 'revision': 'r2',
        'verification': 'last_observed'}
    assert effective['max_duration_sec'] == 180.0 and 'target' not in effective
    wait(lambda: len(robot.peers.requests) == 1)
    sent = robot.peers.requests[0].pose.pose
    assert (sent.position.x, sent.position.y) == pytest.approx((0.4, 0.0), abs=1e-9)
    arrived(robot, 'cup')
