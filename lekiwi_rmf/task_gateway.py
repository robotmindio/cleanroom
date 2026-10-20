#!/usr/bin/env python3
"""Durable admission and observation of existing ROS actions; never a motion controller."""

import json
import hashlib
import math
from pathlib import Path
import re
from queue import Empty, SimpleQueue
import time
import uuid

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID

from lekiwi_rmf.action import Explore, RunSkill, Search
from lekiwi_rmf.exploration import task_limits
from lekiwi_rmf.exploration_owner_guard import owned_goal_id
from lekiwi_rmf.latency_budget import LatencyBudget
from lekiwi_rmf.srv import CancelTask, GetTask, SetPerceptionQuery, SubmitTask, TaskApi
from lekiwi_rmf.perception_query import limits as query_limits
from lekiwi_rmf.task_journal import CAPABILITY_RESOURCES, TERMINAL, Refused, TaskJournal
from lekiwi_rmf.navigation_pause import NavigationPause
from lekiwi_rmf.navigation_observer import NavigationObserver, server_instance
from lekiwi_rmf.navigation_distance import NavigationDistance
from lekiwi_rmf.map_bundle import validate_map_bundle
from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.skills import BASE_COMMANDS, KINDS, SKILL_ID, UNDECLARED_EFFECTS, catalog, load_registry


ACTIONS = {'navigate': ('/navigate_to_pose', NavigateToPose), 'explore': ('/robot/explore', Explore),
           'skill': ('/robot/skills/run', RunSkill), 'search': ('/robot/search', Search)}
SEARCH_PARAMETERS = ('query', 'region', 'max_duration_sec', 'max_travel_m', 'target_coverage')
SEARCH_LIMITS = ('max_duration_sec', 'max_travel_m')
# Schema 1 is SubmitTask/GetTask/CancelTask; schema 2 is the TaskApi services below.
API_SCHEMAS = (1, 2)
TASK_API = {name: f'/robot/tasks/v2/{name}'
            for name in ('validate', 'submit', 'status', 'events', 'cancel', 'transfer', 'compensate')}
# Base motion creates no intended material change, and the base can drive back.
EFFECTS = {name: {'reversibility': 'reversible', 'compensation': text} for name, text in (
    ('navigate', 'Navigate back to the starting pose.'),
    ('explore', 'Map growth is data; navigate back to the starting pose.'),
    ('search', 'Observations are data; navigate back to the starting pose.'))}
UNITS = {'navigate': {'x': 'm', 'y': 'm', 'yaw': 'rad', 'deadline_unix_sec': 's', 'max_travel_m': 'm'},
         'explore': {'max_duration_sec': 's', 'max_radius_m': 'm'},
         'search': {'region': 'm', 'max_duration_sec': 's', 'max_travel_m': 'm', 'target_coverage': 'fraction'},
         'skill': {}}
FRAMES = {'navigate': 'map', 'search': 'map'}
# The joint and base-command mapping a plan was made for; a different body is refused, never guessed.
BODY_PROFILE = 'lekiwi:' + hashlib.sha256(json.dumps([*ARM_JOINTS, *BASE_COMMANDS]).encode()).hexdigest()[:16]
REFERENCE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,127}$')
TASK_ID = re.compile(r'^[A-Za-z0-9._-]{1,128}$')
CHECKPOINT_FIELDS = {'summary', 'goal', 'interpretation', 'references', 'plan', 'remote_commands',
                     'generations', 'resources', 'progress', 'pending_verifications'}
COMPENSATION_INTENTS = {'restore': ('reversible',), 'repair': ('reversible', 'compensable'),
                        'isolate': ('reversible', 'compensable', 'irreversible'),
                        'record': ('reversible', 'compensable', 'irreversible')}


def task_goal(capability, text):
    """Keep untrusted commands finite, small and limited to the advertised fields."""
    if len(text.encode()) > 4096:
        raise ValueError('parameters exceed 4096 bytes')
    try:
        values = json.loads(text)
    except RecursionError as error:
        raise ValueError('parameters JSON is nested too deeply') from error
    if not isinstance(values, dict):
        raise ValueError('parameters must be an object')
    if capability == 'navigate':
        if set(values) - {'x', 'y', 'yaw', 'deadline_unix_sec', 'max_travel_m'} or not {'x', 'y'} <= values.keys():
            raise ValueError('navigate requires x, y and optional yaw in map coordinates')
        for value in values.values():
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError('navigation coordinates must be finite numbers')
        if 'max_travel_m' in values and not 0 < values['max_travel_m'] <= 10:
            raise ValueError('max_travel_m must be in (0,10]')
        if 'deadline_unix_sec' in values and values['deadline_unix_sec'] <= 0:
            raise ValueError('navigation deadline must be positive')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = float(values['x']), float(values['y'])
        yaw = values.get('yaw', 0.0)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal = NavigateToPose.Goal(pose=pose)
    elif capability == 'explore':
        if set(values) - {'revisit_known', 'max_duration_sec', 'max_radius_m', 'resume_task_id'}:
            raise ValueError('unknown exploration parameter')
        if type(values.get('revisit_known', False)) is not bool:
            raise ValueError('revisit_known must be boolean')
        for name in ('max_duration_sec', 'max_radius_m'):
            value = values.get(name, 0.0)
            if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        resume = values.get('resume_task_id', '')
        if not isinstance(resume, str) or (resume and (len(resume) != 32 or any(c not in '0123456789abcdef' for c in resume))):
            raise ValueError('invalid exploration resume ID')
        if resume and (values.get('max_duration_sec') or values.get('max_radius_m')):
            raise ValueError('resumed exploration preserves its original duration and radius')
        goal = Explore.Goal(revisit_known=values.get('revisit_known', False),
                            max_duration_sec=float(values.get('max_duration_sec', 0.0)),
                            max_radius_m=float(values.get('max_radius_m', 0.0)), resume_task_id=resume)
    elif capability == 'search':
        if set(values) - set(SEARCH_PARAMETERS) or 'query' not in values:
            raise ValueError('search requires query and accepts only region, max_duration_sec, max_travel_m, target_coverage')
        query = values['query']
        if not isinstance(query, str) or not 1 <= len(' '.join(query.split())) <= 128:
            raise ValueError('search query must be a label of 1..128 characters')
        for name in (*SEARCH_LIMITS, 'target_coverage'):
            value = values.get(name, 0.0)
            if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if values.get('target_coverage', 0.0) > 1:
            raise ValueError('target_coverage must be within (0,1]')
        region = values.get('region', {'x': 0.0, 'y': 0.0, 'radius_m': 0.0})
        if (not isinstance(region, dict) or set(region) != {'x', 'y', 'radius_m'}
                or any(type(v) not in (float, int) or not math.isfinite(v) for v in region.values())
                or ('region' in values and region['radius_m'] <= 0)):
            raise ValueError('region requires finite map-frame x, y and a positive radius_m')
        goal = Search.Goal(query=query, region_x=float(region['x']), region_y=float(region['y']),
                           region_radius_m=float(region['radius_m']),
                           max_duration_sec=float(values.get('max_duration_sec', 0.0)),
                           max_travel_m=float(values.get('max_travel_m', 0.0)),
                           target_coverage=float(values.get('target_coverage', 0.0)))
    elif capability == 'skill':
        # The skill runner validates parameters against the registered skill; this only bounds the shape.
        if set(values) - {'skill_id', 'parameters'} or not isinstance(values.get('skill_id'), str) \
                or not SKILL_ID.match(values['skill_id']) or not isinstance(values.get('parameters', {}), dict):
            raise ValueError('skill requires a registered skill_id and an optional parameters object')
        goal = RunSkill.Goal(skill_id=values['skill_id'],
                             parameters_json=json.dumps(values.get('parameters', {}), sort_keys=True, allow_nan=False))
    else:
        raise ValueError('unsupported capability')
    return goal, json.dumps(values, sort_keys=True, allow_nan=False)


class TaskGateway(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_tasks', **kwargs)
        domain = self.context.get_domain_id()
        suffix = f'_{domain}' if domain else ''
        if self.get_parameter('use_sim_time').value:
            suffix += '_sim'
        self.declare_parameter('database_path', str(Path.home() / f'.ros/lekiwi_tasks{suffix}.sqlite3'))
        self.declare_parameter('navigation_timeout_sec', 180.0)
        self.declare_parameter('allow_navigation', True)
        self.declare_parameter('allow_exploration', True)
        self.declare_parameter('allow_skills', False)
        # The explorer's tracked allow_search keeps the action server itself off by default.
        self.declare_parameter('allow_search', True)
        self.declare_parameter('map_bundle', '')
        self._enabled = {'navigate': self.get_parameter('allow_navigation').value,
                         'explore': self.get_parameter('allow_exploration').value,
                         'skill': self.get_parameter('allow_skills').value,
                         'search': self.get_parameter('allow_search').value}
        try:
            # Experimental skills run only through an acquisition protocol, never a user task.
            self._skills = catalog(load_registry())
        except (OSError, ValueError) as error:
            self.get_logger().error(f'skill registry unavailable: {error}')
            self._enabled['skill'], self._skills = False, []
        self._timeout = self.get_parameter('navigation_timeout_sec').value
        if type(self._timeout) not in (float, int) or not math.isfinite(self._timeout) or self._timeout <= 0:
            raise ValueError('navigation_timeout_sec must be positive and finite')
        self._journal = TaskJournal(self.get_parameter('database_path').value)
        self._skill_effects = {skill['skill_id']: skill['effects'] for skill in self._skills}
        # Dispatched tasks observed live by this process; recovered ones stay uncertain until then.
        self._live = set()
        self._resource_snapshot = None
        self._pending = {}
        self._replies = SimpleQueue()
        self._queried = {}
        self._sent = set()
        self._started = {}
        # Monotonic admission and dispatch times of tasks this process saw; a restart loses only samples.
        self._budget, self._admitted_at, self._dispatched_at = LatencyBudget('task'), {}, {}
        self._recovering = {row['id'] for row in self._active() if row['dispatched']}
        self._observed = {capability: set() for capability in ACTIONS}
        self._terminal_seen = {}
        self._navigation = NavigationPause()
        self._children = NavigationObserver(self)
        self._pausing = {row['id'] for row in self._active()
                         if row['capability'] == 'navigate' and row['pause_requested']
                         and row['dispatched'] and not row['cancel_requested']}
        self._pause_started = {identifier: time.monotonic() - 3.0 for identifier in self._pausing}
        self._unknown_results = {}
        self._unknown_cancels = {}
        self._unknown_since = {}
        self._guard = (False, 0.0)
        self._ack = (b'', 0.0)
        self._action_clients = {}
        self._result_clients = {}
        self._cancel_clients = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._events = self.create_publisher(String, '/robot/tasks/events', latched)
        self._map_publisher = self.create_publisher(String, '/robot/map_context', latched)
        bundle_path = self.get_parameter('map_bundle').value
        map_context = {'ready': False}
        if bundle_path:
            validate_map_bundle(bundle_path)
            map_context = {'ready': True, 'map_id': hashlib.sha256(Path(bundle_path).read_bytes()).hexdigest(),
                           'epoch': uuid.uuid4().hex, 'started_at': self.get_clock().now().nanoseconds / 10**9}
        self._map_publisher.publish(String(data=json.dumps(map_context)))
        self._exploration_defaults = None
        self.create_subscription(String, '/robot/explore/defaults', self._explore_defaults, latched)
        self._search_defaults = None
        self.create_subscription(String, '/robot/search/defaults', self._search_defaults_message, latched)
        self._owner = self.create_publisher(UUID, '/robot/explore/navigation_owner', 10)
        self.create_subscription(Bool, '/safety/base_motion_permitted',
                                 lambda msg: self._navigation.permission(msg.data, time.monotonic()), latched)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 lambda msg: self._navigation.collision(msg.action_type == CollisionMonitorState.STOP,
                                                                       msg.polygon_name, time.monotonic()), 1)
        self.create_subscription(Bool, '/robot/explore/navigation_guard_ready', self._guard_ready, 1)
        self.create_subscription(UUID, '/robot/explore/navigation_guarded', self._guard_ack, 10)
        for capability, (name, action) in ACTIONS.items():
            self._action_clients[capability] = ActionClient(self, action, name)
            self._result_clients[capability] = self.create_client(action.Impl.GetResultService, f'{name}/_action/get_result')
            self._cancel_clients[capability] = self.create_client(CancelGoal, f'{name}/_action/cancel_goal')
            self.create_subscription(action.Impl.FeedbackMessage, f'{name}/_action/feedback', self._feedback, 10)
            self.create_subscription(GoalStatusArray, f'{name}/_action/status',
                                     lambda message, name=capability: self._status(message, name), latched)
        self.create_service(SubmitTask, '/robot/tasks/submit', self._submit)
        self.create_service(GetTask, '/robot/tasks/get', self._get)
        self.create_service(CancelTask, '/robot/tasks/cancel', self._cancel)
        self.create_service(Trigger, '/robot/capabilities', self._capabilities)
        for name, handler in (('validate', self._api_validate), ('submit', self._api_submit),
                              ('status', self._api_status), ('events', self._api_events),
                              ('cancel', self._api_cancel), ('transfer', self._api_transfer),
                              ('compensate', self._api_compensate)):
            self.create_service(TaskApi, TASK_API[name], self._api(handler))
        # Holders of exclusive resources, for nodes that enforce them outside this gateway.
        self._resources = self.create_publisher(String, '/robot/resources', latched)
        self._publish_resources()
        self.create_timer(0.2, self._publish_resources, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._observation_status = self.create_client(Trigger, '/robot/observations/status')
        # Only to report availability; clients call the detector's query service directly.
        self._perception_query = self.create_client(SetPerceptionQuery, '/robot/perception/set_query')
        self._distance = NavigationDistance()
        self.create_subscription(Odometry, '/odom', self._distance.update, 1)
        self.create_service(Trigger, '/robot/navigation/odometry', self._odometry)
        self._diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._health, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.2, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _health(self):
        message = DiagnosticArray(status=[self._budget.status()])
        message.header.stamp = self.get_clock().now().to_msg()
        self._diagnostics.publish(message)

    def _active(self):
        return self._journal.active()

    def _row(self, identifier):
        return self._journal.row(identifier)

    def _update(self, identifier, state, detail, result=None):
        if result is not None:
            metadata = json.loads(self._row(identifier)['result']).get('_task')
            if metadata is not None:
                result = {**result, '_task': metadata}
        if not self._journal.update(identifier, state, detail, result):
            return
        row = self._row(identifier)
        self._events.publish(String(data=json.dumps(dict(row), default=lambda value: bytes(value).hex())))
        if state in TERMINAL:
            dispatched = self._dispatched_at.pop(identifier, None)
            self._admitted_at.pop(identifier, None)
            if dispatched is not None:
                self._budget.record('execution', time.monotonic() - dispatched)
            self._release_requests(identifier, row)

    def _release_requests(self, identifier, row):
        for kind, client in (('send', None), ('result', self._result_clients[row['capability']]),
                             ('cancel', self._cancel_clients[row['capability']])):
            key = (kind, identifier)
            future = self._pending.pop(key, None)
            if future is not None:
                if client is not None:
                    client.remove_pending_request(future)
                future.cancel()
            self._queried.pop(key, None)
        self._recovering.discard(identifier)
        self._sent.discard(identifier)
        self._started.pop(identifier, None)
        self._terminal_seen.pop(identifier, None)
        self._pausing.discard(identifier)
        self._pause_started.pop(identifier, None)
        self._unknown_results.pop(identifier, None)
        self._unknown_cancels.pop(identifier, None)
        self._unknown_since.pop(identifier, None)

    def _effects(self, capability, values):
        if capability == 'skill':
            return self._skill_effects.get(values.get('skill_id'), UNDECLARED_EFFECTS)
        return EFFECTS[capability]

    def _admit(self, task_id, capability, parameters_json, request=None, dry_run=False):
        """Every admission check for SubmitTask (no request) and the negotiated API, before any dispatch.

        Returns (row, summary): an existing row with no summary for an idempotent retry,
        no row for a dry run, or the newly admitted row.
        """
        if not isinstance(task_id, str) or not TASK_ID.match(task_id):
            raise Refused('invalid_request', 'task_id requires 1..128 ASCII letters, numbers, dots, hyphens or underscores')
        try:
            _, parameters = task_goal(capability, parameters_json)
        except (ValueError, TypeError, OverflowError) as error:
            raise Refused('invalid_request', str(error)) from error
        values = json.loads(parameters)
        effects = self._effects(capability, values)
        if request is None and effects['reversibility'] != 'reversible':
            # Schema 1 reports an implementation that finished as succeeded; with effects
            # that cannot be undone a client must see execution and effect separately.
            raise Refused('schema_incompatible', f'{capability} has {effects["reversibility"]} effects; '
                          'submit it through the task API schema 2')
        compensates = (request or {}).get('compensates') or {}
        digest = hashlib.sha256(json.dumps(
            {'capability': capability, 'parameters': values, 'goal': request.get('goal'),
             'compensates': compensates.get('task_id', '')}, sort_keys=True).encode()).hexdigest() if request else ''
        row = self._row(task_id)
        if row is not None:
            same = (row['digest'] == digest if row['digest'] and digest
                    else (row['capability'], row['parameters']) == (capability, parameters))
            if not same:
                raise Refused('identity_conflict', 'task_id already has different parameters', task_id=task_id)
            # A retry after a lost reply returns what was admitted; it never sends a second goal.
            return row, None
        if not self._enabled[capability]:
            raise Refused('capability_unavailable', 'capability disabled by this deployment profile')
        resources = CAPABILITY_RESOURCES[capability]
        if request is not None:
            self._check_request(capability, request, resources)
        self._journal.require_free(resources)
        if any(self._observed.values()) or self._children.busy():
            raise Refused('resource_busy', 'another ROS navigation or exploration client has an active goal',
                          holders=[{'resource': 'base', 'task_id': '', 'owner': 'external ROS client'}])
        if not self._action_clients[capability].server_is_ready():
            raise Refused('capability_unavailable', 'capability action server unavailable')
        effective = None
        if capability == 'navigate' and 'deadline_unix_sec' in values and not (
                time.time() < values['deadline_unix_sec'] <= time.time() + 300):
            raise Refused('precondition_failed', 'navigation deadline must be within the next 300 seconds')
        if capability == 'navigate':
            effective = {**values, 'yaw': values.get('yaw', 0.0), 'max_duration_sec': self._timeout}
        elif (capability == 'explore' and self._exploration_defaults is not None
              and not values.get('resume_task_id')):
            duration, radius = task_limits(
                values.get('max_duration_sec', 0.0), values.get('max_radius_m', 0.0),
                self._exploration_defaults['max_duration_sec'], self._exploration_defaults['max_radius_m'])
            effective = {'max_duration_sec': duration, 'max_radius_m': radius}
            effective['revisit_known'] = values.get('revisit_known', False)
            effective['resume_task_id'] = ''
        elif capability == 'search' and self._search_defaults is not None:
            effective = dict(values)
            for name in SEARCH_LIMITS:
                if values.get(name, 0.0) > self._search_defaults[name]:
                    raise Refused('invalid_request', f'{name} exceeds the tracked limit {self._search_defaults[name]}')
                effective[name] = values.get(name, 0.0) or self._search_defaults[name]
            effective['target_coverage'] = values.get('target_coverage', 0.0) or self._search_defaults['target_coverage']
        metadata = {'parameters': values, 'effective_parameters': effective}
        if capability == 'navigate' and 'max_travel_m' in values:
            meter = self._distance.status()
            if not meter['ready']:
                raise Refused('precondition_failed', 'bounded navigation requires fresh odometry')
            metadata['odometry_origin'] = {'epoch': meter['epoch'], 'meters': meter['meters']}
        if request is not None:
            metadata['request'] = {key: request[key] for key in
                                   ('goal', 'body_profile', 'frame', 'units', 'authority') if key in request}
        summary = {'capability': capability, 'parameters': values, 'effective_parameters': effective,
                   'resources': list(resources), 'effects': effects, 'body_profile': BODY_PROFILE, 'digest': digest}
        if dry_run:
            return None, summary
        goal_id = bytes(owned_goal_id().uuid) if capability == 'navigate' else uuid.uuid4().bytes
        row = self._journal.admit(task_id, capability, parameters, goal_id, result={'_task': metadata},
                                  resources=resources, owner=(request or {}).get('owner', ''),
                                  api_schema=2 if request is not None else 1, digest=digest,
                                  compensates=compensates.get('task_id', ''))
        self._admitted_at[task_id] = time.monotonic()
        self._publish_resources()
        return row, summary

    def _check_request(self, capability, request, resources):
        """Refuse a plan made for another body, frame, unit system or resource set."""
        body = request.get('body_profile')
        if body is not None and body != BODY_PROFILE:
            raise Refused('body_profile_mismatch', f'request is for body {body}; this robot is {BODY_PROFILE}')
        frame = request.get('frame')
        if frame is not None and frame != FRAMES.get(capability):
            raise Refused('map_unaligned', f'{capability} takes {FRAMES.get(capability) or "no"} frame coordinates, '
                          f'not {frame!r}')
        if request.get('units', 'si') != 'si':
            raise Refused('unit_mismatch', 'parameters are SI: metres, radians, seconds')
        declared = request.get('resources')
        if declared is not None and (len(set(declared)) != len(declared) or set(declared) != set(resources)):
            raise Refused('resource_mismatch', f'{capability} uses exactly {sorted(resources)}')
        compensates = request.get('compensates')
        if compensates is not None:
            original = self._journal.fence(compensates['task_id'], compensates['generation'], request['owner'],
                                           reconciled=True)
            plan = json.loads(original['compensation'] or 'null')
            if plan is None or plan['status'] not in ('planned', 'in_progress'):
                raise Refused('precondition_failed', 'that task has no open compensation plan')
            steps = [step['capability'] for step in plan['plan']['steps']]
            if capability not in steps or len(plan.get('tasks', {})) >= len(steps):
                raise Refused('precondition_failed', 'the compensation plan has no remaining step for this capability')

    def _submit(self, request, response):
        try:
            row, _ = self._admit(request.task_id, request.capability, request.parameters_json)
            response.accepted, response.state, response.detail = True, row['state'], row['detail']
        except (ValueError, TypeError, OverflowError) as error:
            response.accepted, response.state, response.detail = False, 'rejected', str(error)
        return response

    def _api(self, handler):
        """One negotiated wrapper: an unsupported schema or payload is refused, never reinterpreted."""
        def call(request, response):
            response.schema = 2
            try:
                if request.schema != 2:
                    raise Refused('schema_incompatible', f'this service speaks schema 2; the gateway supports {list(API_SCHEMAS)}',
                                  supported=list(API_SCHEMAS))
                if len(request.request_json.encode()) > 32768:
                    raise Refused('invalid_request', 'request exceeds 32768 bytes')
                values = json.loads(request.request_json or '{}')
                if not isinstance(values, dict):
                    raise Refused('invalid_request', 'request must be a JSON object')
                result = handler(values)
                response.ok, response.response_json = True, json.dumps(result, allow_nan=False, sort_keys=True)
            except Refused as error:
                response.ok, response.error, response.detail = False, error.code, str(error)
                response.response_json = json.dumps(error.data, allow_nan=False, sort_keys=True)
            except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as error:
                response.ok, response.error, response.detail = False, 'invalid_request', str(error)
            return response
        return call

    @staticmethod
    def _envelope(values):
        allowed = {'task_id', 'capability', 'parameters', 'owner', 'goal', 'body_profile', 'frame', 'units',
                   'resources', 'authority', 'compensates'}
        if set(values) - allowed:
            raise Refused('invalid_request', f'unknown request fields {sorted(set(values) - allowed)}')
        if values.get('capability') not in ACTIONS:
            raise Refused('invalid_request', 'unsupported capability')
        parameters = values.get('parameters', {})
        if not isinstance(parameters, dict):
            raise Refused('invalid_request', 'parameters must be an object')
        owner = values.get('owner')
        if not isinstance(owner, str) or not REFERENCE.match(owner):
            raise Refused('invalid_request', 'owner requires a 1..128 character agent or worker reference')
        authority = values.get('authority')
        if authority is not None and (not isinstance(authority, str) or not REFERENCE.match(authority)):
            raise Refused('invalid_request', 'authority is a policy or approval reference, never a credential')
        goal = values.get('goal')
        if goal is not None and (not isinstance(goal, dict) or set(goal) - {'id', 'revision'}
                                 or not isinstance(goal.get('id'), str) or not REFERENCE.match(goal['id'])
                                 or type(goal.get('revision', 0)) is not int or goal.get('revision', 0) < 0):
            raise Refused('invalid_request', 'goal requires an id and an optional nonnegative integer revision')
        compensates = values.get('compensates')
        if compensates is not None and (not isinstance(compensates, dict) or set(compensates) != {'task_id', 'generation'}
                                        or not isinstance(compensates['task_id'], str)
                                        or type(compensates['generation']) is not int):
            raise Refused('invalid_request', 'compensates requires the original task_id and generation')
        if any(key in values and not isinstance(values[key], str) for key in ('body_profile', 'frame', 'units')):
            raise Refused('invalid_request', 'body_profile, frame and units are strings')
        resources = values.get('resources')
        if resources is not None and (not isinstance(resources, list) or not all(isinstance(r, str) for r in resources)):
            raise Refused('invalid_request', 'resources is a list of resource names')
        return values.get('task_id'), values['capability'], json.dumps(parameters, allow_nan=False), values

    def _api_validate(self, values):
        row, summary = self._admit(*self._envelope(values), dry_run=True)
        return {'valid': True, 'existing': self._view(row) if row is not None else None, 'request': summary}

    def _api_submit(self, values):
        row, summary = self._admit(*self._envelope(values))
        return {'admitted': summary is not None, 'existing': summary is None, 'task': self._view(row)}

    def _task_row(self, values):
        identifier = values.get('task_id')
        if not isinstance(identifier, str):
            raise Refused('invalid_request', 'task_id must be a string')
        generation = values.get('generation')
        if generation is not None and type(generation) is not int:
            raise Refused('invalid_request', 'generation must be an integer')
        row = self._row(identifier)
        if row is None:
            raise Refused('not_found', 'task not found')
        return row, generation

    def _api_status(self, values):
        row, generation = self._task_row(values)
        if generation is not None:
            # Reading the journal with its own generation is how a successor reconciles.
            self._journal.reconcile(row['id'], generation)
        return {'task': self._view(self._row(row['id']))}

    def _api_events(self, values):
        cursor, limit, identifier = values.get('cursor', 0), values.get('limit', 100), values.get('task_id', '')
        if type(cursor) is not int or cursor < 0 or type(limit) is not int or not 1 <= limit <= 100:
            raise Refused('invalid_request', 'cursor is a nonnegative integer and limit is 1..100')
        if not isinstance(identifier, str):
            raise Refused('invalid_request', 'task_id must be a string')
        if identifier and values.get('generation') is not None:
            row, generation = self._task_row(values)
            self._journal.reconcile(row['id'], generation)
        return self._journal.events_since(cursor, identifier, limit)

    def _api_cancel(self, values):
        identifier = values.get('task_id', '')
        if not isinstance(identifier, str):
            raise Refused('invalid_request', 'task_id must be a string')
        requester = values.get('requester', '')
        if requester and (not isinstance(requester, str) or not REFERENCE.match(requester)):
            raise Refused('invalid_request', 'requester is a 1..128 character reference')
        if not identifier:
            active = self._active()
            identifier = active[0]['id'] if active else ''
        row, generation = self._task_row({**values, 'task_id': identifier})
        if generation is not None:
            # A fenced caller cannot stop work it no longer owns; an unfenced stop is always accepted.
            self._journal.fence(row['id'], generation)
        accepted = row['state'] not in TERMINAL
        if accepted:
            self._journal.request_cancel(row['id'], requester=requester or 'task_api')
            self._update(row['id'], 'canceling', 'cancellation requested; awaiting terminal ROS result')
        view = self._view(self._row(row['id']))
        return {'accepted': accepted, 'receipt': view['cancel'], 'stop': view['stop'], 'task': view}

    def _api_transfer(self, values):
        if set(values) - {'task_id', 'generation', 'owner', 'new_owner', 'checkpoint'}:
            raise Refused('invalid_request', 'transfer takes task_id, generation, owner, new_owner and checkpoint')
        row, generation = self._task_row(values)
        owner, successor, checkpoint = values.get('owner'), values.get('new_owner'), values.get('checkpoint')
        if generation is None or not all(isinstance(value, str) and REFERENCE.match(value) for value in (owner, successor)):
            raise Refused('invalid_request', 'transfer requires generation, owner and new_owner references')
        if (not isinstance(checkpoint, dict) or set(checkpoint) - CHECKPOINT_FIELDS
                or not isinstance(checkpoint.get('summary'), str) or not 1 <= len(checkpoint['summary']) <= 2000
                or len(json.dumps(checkpoint, allow_nan=False)) > 16384):
            raise Refused('invalid_request', f'checkpoint is at most 16 KiB of {sorted(CHECKPOINT_FIELDS)} '
                          'with a readable summary; never credentials')
        row = self._journal.transfer(row['id'], generation, owner, successor, checkpoint)
        self._publish_resources()
        return {'task': self._view(row), 'generation': row['generation'], 'previous_generation': generation}

    def _api_compensate(self, values):
        if set(values) - {'task_id', 'generation', 'owner', 'plan'}:
            raise Refused('invalid_request', 'compensate takes task_id, generation, owner and plan')
        row, generation = self._task_row(values)
        plan = values.get('plan')
        if generation is None or not isinstance(values.get('owner'), str):
            raise Refused('invalid_request', 'compensate requires generation and owner')
        if (not isinstance(plan, dict) or set(plan) - {'intent', 'steps', 'note'} or plan.get('intent') not in COMPENSATION_INTENTS
                or not isinstance(plan.get('steps', []), list) or len(plan.get('steps', [])) > 8
                or not isinstance(plan.get('note', ''), str) or len(plan.get('note', '')) > 2000):
            raise Refused('invalid_request', f'plan has intent {sorted(COMPENSATION_INTENTS)}, at most 8 steps and a note')
        steps = plan.get('steps', [])
        for step in steps:
            if not isinstance(step, dict) or set(step) != {'capability', 'parameters'} or step['capability'] not in ACTIONS:
                raise Refused('invalid_request', 'each step is {capability, parameters}')
            try:
                task_goal(step['capability'], json.dumps(step['parameters'], allow_nan=False))
            except (ValueError, TypeError) as error:
                raise Refused('invalid_request', f'step {step["capability"]}: {error}') from error
        if row['state'] not in TERMINAL:
            raise Refused('precondition_failed', 'cancel the task and wait for its confirmed stop before compensating')
        reversibility = self._effects(row['capability'], json.loads(row['parameters']))['reversibility']
        if reversibility not in COMPENSATION_INTENTS[plan['intent']]:
            raise Refused('irreversible_effect', f'a {reversibility} effect cannot be {plan["intent"]}d; '
                          'stop, isolate and record it instead')
        compensation = {'plan': {'intent': plan['intent'], 'steps': steps, 'note': plan.get('note', '')},
                        'status': 'planned' if steps else 'recorded', 'recorded_at': time.time(),
                        'reversibility': reversibility, 'automatic': False,
                        # Compensation repairs state; it never achieves or replaces the original goal.
                        'counts_as_goal_success': False,
                        'material_rollback': reversibility == 'reversible' and plan['intent'] == 'restore',
                        'original_state': row['state'], 'tasks': {}}
        row = self._journal.record_compensation(row['id'], generation, values['owner'], compensation)
        return {'task': self._view(row)}

    def _view(self, row):
        """Separate dimensions of one task: command phase, stop, effect evidence and goal verification."""
        result = json.loads(row['result'])
        metadata = result.get('_task', {})
        state, identifier = row['state'], row['id']
        terminal = state in TERMINAL
        if not terminal and not row['dispatched']:
            phase = 'admitted'
        elif state == 'unknown':
            phase = 'uncertain'
        elif not terminal and row['navigation_stopped']:
            # Its native attempt already ended; the journal holds the pause, not a live executor.
            phase = 'paused'
        elif not terminal and identifier in self._recovering and identifier not in self._live:
            phase = 'uncertain'
        elif state == 'paused':
            phase = 'paused'
        elif not terminal:
            phase = 'running'
        elif not row['dispatched']:
            phase = 'not_started'
        else:
            phase = 'stopped' if state == 'canceled' else 'execution_finished'
        outcome = None
        if terminal:
            outcome = 'unknown' if result.get('outcome_known') is False else state
        if terminal:
            stop = 'confirmed'
        elif phase == 'uncertain':
            stop = 'uncertain'
        elif row['cancel_requested']:
            stop = 'stopping'
        else:
            stop = 'not_requested'
        effect = {'observed': False, 'verified_by_robot': False}
        if row['capability'] == 'search' and result.get('found') and result.get('evidence_ids'):
            effect.update(observed=True, kind='label_detected', evidence_ids=result['evidence_ids'])
        elif row['capability'] == 'navigate' and state == 'succeeded':
            effect.update(observed=True, kind='pose_reached', source='nav2')
        effects = self._effects(row['capability'], json.loads(row['parameters']))
        compensation = json.loads(row['compensation'] or 'null')
        request = metadata.get('request', {})
        receipt = json.loads(row['cancel_receipt'] or 'null')
        if receipt is None and row['cancel_requested']:
            receipt = {'receipt_id': None, 'task_id': identifier,
                       'reason': 'deadline' if row['cancel_requested'] == 2 else 'requested'}
        return {
            'task_id': identifier, 'capability': row['capability'], 'state': state, 'detail': row['detail'],
            'api_schema': row['api_schema'], 'owner': row['owner'], 'generation': row['generation'],
            'reconciled': row['reconciled_generation'] == row['generation'],
            'resources': json.loads(row['resources']), 'holding_resources': not terminal,
            'execution': {'phase': phase, 'dispatched': bool(row['dispatched']), 'outcome': outcome},
            'cancel': receipt, 'stop': stop, 'effect': effect,
            # Only the authorized verifier can credit the goal; the robot never does.
            'goal': {**request.get('goal', {}), 'verified': None, 'verifier': 'recuriant'},
            'effects': effects, 'compensation': compensation,
            'compensation_required': (terminal and bool(row['dispatched']) and state != 'succeeded'
                                      and effects['reversibility'] != 'reversible' and compensation is None),
            'compensates': row['compensates'] or None,
            'checkpoint': json.loads(row['checkpoint'] or 'null'),
            'parameters': metadata.get('parameters'), 'effective_parameters': metadata.get('effective_parameters'),
            'times': self._journal.times(identifier), 'cursor': self._journal.last_event(identifier),
            'journal_id': self._journal.journal_id,
            'result': {key: value for key, value in result.items() if key != '_task'}}

    def _publish_resources(self):
        state = self._journal.resource_state()
        if state != self._resource_snapshot:
            self._resource_snapshot = state
            self._resources.publish(String(data=json.dumps(
                {'schema': 1, 'journal_id': self._journal.journal_id, 'resources': state}, sort_keys=True)))

    def _get(self, request, response):
        row = self._row(request.task_id)
        response.found = row is not None
        if row is not None:
            response.capability, response.state, response.detail, response.result_json = row['capability'], row['state'], row['detail'], row['result']
        return response

    def _cancel(self, request, response):
        active = self._active()
        identifier = request.task_id or (active[0]['id'] if active else '')
        row = self._row(identifier)
        if row is None:
            response.detail = 'task not found'
            return response
        if row['state'] not in TERMINAL:
            self._journal.request_cancel(identifier, requester='CancelTask')
            self._update(identifier, 'canceling', 'cancellation requested; awaiting terminal ROS result')
        row = self._row(identifier)
        response.accepted, response.state, response.detail = True, row['state'], row['detail']
        return response

    def _explore_defaults(self, message):
        try:
            if len(message.data.encode()) > 4096:
                raise ValueError('exploration defaults exceed 4096 bytes')
            values = json.loads(message.data)
            if (not isinstance(values, dict) or set(values) != {'max_duration_sec', 'max_radius_m'}
                    or any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0
                           for value in values.values())):
                raise ValueError('invalid exploration defaults')
        except (ValueError, TypeError, RecursionError, OverflowError):
            self._exploration_defaults = None
            self.get_logger().error('invalid exploration defaults contract')
            return
        self._exploration_defaults = values

    def _search_defaults_message(self, message):
        try:
            if len(message.data.encode()) > 4096:
                raise ValueError('search defaults exceed 4096 bytes')
            values = json.loads(message.data)
            if (not isinstance(values, dict) or set(values) != {*SEARCH_LIMITS, 'target_coverage'}
                    or any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0
                           for value in values.values()) or values['target_coverage'] > 1):
                raise ValueError('invalid search defaults')
        except (ValueError, TypeError, RecursionError, OverflowError):
            self._search_defaults = None
            self.get_logger().error('invalid search defaults contract')
            return
        self._search_defaults = values

    def _capability(self, name, client):
        entry = {'name': name, 'available': self._enabled[name] and client.server_is_ready()}
        if name == 'navigate':
            return {**entry, 'parameters': ['x', 'y', 'yaw', 'deadline_unix_sec', 'max_travel_m'],
                    'required_parameters': ['x', 'y'], 'defaults': {'yaw': 0.0},
                    'effective_defaults': {'yaw': 0.0, 'max_duration_sec': self._timeout},
                    'description': 'Map-frame x, y; yaw in radians. Pauses preserve the active-time budget.'}
        if name == 'explore':
            return {**entry, 'parameters': ['revisit_known', 'max_duration_sec', 'max_radius_m', 'resume_task_id'],
                    'required_parameters': [],
                    'defaults': {'revisit_known': False, 'max_duration_sec': 0.0, 'max_radius_m': 0.0, 'resume_task_id': ''},
                    'effective_defaults': ({**self._exploration_defaults, 'revisit_known': False, 'resume_task_id': ''}
                                           if self._exploration_defaults is not None else None),
                    'description': ('Omit parameters to explore unmapped space. revisit_known=true also revisits mapped space. '
                                    'Omitted/zero limits use the tracked robot limits; positive limits may only reduce them. '
                                    'Duration is active seconds, radius is metres around the starting pose. '
                                    'resume_task_id is a saved native exploration UUID and preserves its original bounds.')}
        if name == 'skill':
            return {**entry, 'parameters': ['skill_id', 'parameters'], 'required_parameters': ['skill_id'],
                    'defaults': {'parameters': {}}, 'effective_defaults': {'parameters': {}}, 'skills': self._skills,
                    # The implementation kinds the runner executes, so a client acquires only runnable skills.
                    'implementation_kinds': list(KINDS),
                    'description': 'Run a registered skill implementation on the arm. Success means the implementation '
                                   'finished, not that the world reached the goal; verify the effect independently. '
                                   'Each skill lists the resources it commands or reads (arm, base, camera); '
                                   'admission holds the capability resources whatever the skill uses.'}
        return {**entry, 'parameters': list(SEARCH_PARAMETERS), 'required_parameters': ['query'],
                'defaults': {'region': None, 'max_duration_sec': 0.0, 'max_travel_m': 0.0, 'target_coverage': 0.0},
                'effective_defaults': (dict(self._search_defaults, region=None)
                                       if self._search_defaults is not None else None),
                'labels': '/robot/perception/profile',
                'outcomes': ['found', 'coverage_reached', 'inaccessible', 'budget_exhausted', 'incapable', 'canceled', 'failed'],
                'description': ('Moves the base between viewpoints chosen by expected information gain until the query '
                                'label is detected, target_coverage of the searchable mapped region has fresh evidence '
                                'of absence, or a budget ends. query is a native label of the perception profile; '
                                'region {x, y, radius_m} is map-frame, default the tracked motion bound. Results carry '
                                'evidence_ids, coverage_fraction and a negative-evidence report; "not found" holds only '
                                'for the covered cells. Zero/omitted limits use the tracked limits; positive limits may only reduce them.')}

    @staticmethod
    def _body_profile():
        return {'id': BODY_PROFILE, 'arm_joints': list(ARM_JOINTS), 'base_commands': list(BASE_COMMANDS)}

    def _contract(self, entry):
        """Resources, units, frame and effects every capability declares, for planning and validation."""
        name = entry['name']
        entry = {**entry, 'resources': list(CAPABILITY_RESOURCES[name]), 'units': UNITS[name],
                 'frame': FRAMES.get(name)}
        if name == 'skill':
            entry['skills'] = [{**skill, 'requires_schema': 1 if skill['effects']['reversibility'] == 'reversible' else 2}
                               for skill in entry['skills']]
            entry['effects'] = {'per_skill': True, 'undeclared': UNDECLARED_EFFECTS}
            entry['requires_schema'] = 'per_skill'
        else:
            entry['effects'], entry['requires_schema'] = EFFECTS[name], 1
        return entry

    def _capabilities(self, _request, response):
        response.success = True
        response.message = json.dumps({'schema': 1, 'capabilities': [
            self._contract(self._capability(name, client)) for name, client in self._action_clients.items()],
            'terminal_states': sorted(TERMINAL),
            'task_api': {'schemas': list(API_SCHEMAS), 'type': 'lekiwi_rmf/srv/TaskApi', 'services': TASK_API,
                         'body_profile': self._body_profile(), 'journal_id': self._journal.journal_id,
                         'resources': self._journal.resource_state(), 'resource_topic': '/robot/resources',
                         'description': ('Schema 2 separates admission, execution, stop and effect evidence; '
                                         'grants a generation (fencing token) per owner; serializes tasks whose '
                                         'resources overlap; returns cancel receipts; and replays journal events '
                                         'from a cursor. Capabilities with requires_schema 2 are refused through '
                                         'SubmitTask.')},
            'observations': {'available': self._observation_status.service_is_ready(),
                             'schema': 1, 'grounding': 'image_only',
                             'services': {'query': '/robot/observations/query',
                                          'evidence': '/robot/observations/evidence',
                                          'pin': '/robot/observations/pin',
                                          'status': '/robot/observations/status'},
                             'events': '/robot/observations/events',
                             # Open-vocabulary detectors localize the active query; closed ones reject it.
                             # Every open observation records the query revision it ran under.
                             'query': {'available': self._perception_query.service_is_ready(),
                                       'service': '/robot/perception/set_query',
                                       'type': 'lekiwi_rmf/srv/SetPerceptionQuery',
                                       'profile': '/robot/perception/profile', **query_limits(),
                                       'description': 'Set the labels the detector localizes from the next frame. '
                                                      'Empty labels restore the default query; expected_revision '
                                                      'applies the change only while that revision is in force.'}},
            'examples': [{'capability': 'explore', 'parameters': {}},
                         {'capability': 'explore', 'parameters': {'revisit_known': True}}],
            'services': {'submit': '/robot/tasks/submit', 'get': '/robot/tasks/get', 'cancel': '/robot/tasks/cancel'},
            'events': '/robot/tasks/events'})
        return response

    def _guard_ready(self, message):
        self._guard = message.data, time.monotonic()

    def _odometry(self, _request, response):
        result = self._distance.status()
        response.success = result['ready']
        response.message = json.dumps(result, allow_nan=False)
        return response

    def _guard_ack(self, message):
        self._ack = bytes(message.uuid), time.monotonic()

    def _feedback(self, message):
        row = self._journal.goal(bytes(message.goal_id.uuid))
        if row is None or row['state'] in TERMINAL:
            return
        self._live.add(row['id'])
        if row['cancel_requested'] or row['navigation_stopped']:
            return
        if row['id'] in self._pausing:
            return
        stage = getattr(message.feedback, 'stage', 'navigating')
        self._update(row['id'], 'paused' if stage.startswith('paused:') else 'running', stage)

    def _status(self, message, capability):
        self._observed[capability] = {bytes(status.goal_info.goal_id.uuid) for status in message.status_list
                                      if status.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)}
        for status in message.status_list:
            row = self._journal.goal(bytes(status.goal_info.goal_id.uuid))
            if row is not None and row['state'] not in TERMINAL and not row['navigation_stopped']:
                if status.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING):
                    self._live.add(row['id'])
                if status.status in (4, 5, 6):
                    self._terminal_seen.setdefault(row['id'], time.monotonic())
                self._lookup_result(row)

    def _lookup_result(self, row):
        key = ('result', row['id'])
        client = self._result_clients[row['capability']]
        if (key in self._pending or not client.service_is_ready()
                or time.monotonic() - self._queried.get(key, 0.0) < 1.0):
            return
        action = ACTIONS[row['capability']][1]
        request = action.Impl.GetResultService.Request(goal_id=UUID(uuid=list(row['goal_id'])))
        server_id = self._server_id(row['capability'])
        future = client.call_async(request)
        self._queried[key] = time.monotonic()
        self._pending[key] = future
        future.add_done_callback(lambda done: self._queue_reply('result', row, done, server_id))

    def _result(self, identifier, future, server_id=''):
        if future.cancelled():
            return
        key = ('result', identifier)
        if self._pending.get(key) is future:
            self._pending.pop(key)
        if self._row(identifier)['state'] in TERMINAL or self._row(identifier)['navigation_stopped']:
            return
        try:
            response = future.result()
            result = dict(message_to_ordereddict(response.result))
            json.dumps(result, allow_nan=False)
        except Exception as error:
            self.get_logger().error(f'task result unconfirmed: {error}')
            self._update(identifier, 'unknown', 'result transport failed; termination unconfirmed')
            return
        if response.status == GoalStatus.STATUS_UNKNOWN:
            self._recovering.add(identifier)
            if server_id and self._server_id(self._row(identifier)['capability']) == server_id:
                if self._unknown_results.get(identifier) != server_id:
                    self._unknown_since[identifier] = time.monotonic()
                self._unknown_results[identifier] = server_id
            self._update(identifier, 'unknown', 'ROS server has no result; delivery or termination remains unconfirmed')
            return
        states = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'failed'}
        if response.status not in states:
            self._update(identifier, 'unknown', 'nonterminal ROS result')
            return
        state = states[response.status]
        if state == 'succeeded' and (result.get('complete') is False or result.get('error_code', 0)):
            state = 'failed'
        detail = result.get('message') or result.get('error_msg') or result.get('detail') or state
        if self._row(identifier)['cancel_requested'] == 2:
            state, detail = 'failed', f'navigation deadline reached; ROS terminal state: {state}'
        row = self._row(identifier)
        if row['capability'] == 'navigate' and identifier in self._pausing and not row['cancel_requested']:
            if response.status == GoalStatus.STATUS_SUCCEEDED and not result.get('error_code', 0):
                self._update(identifier, state, detail, result)
            else:
                self._journal.navigation_stopped(identifier)
                self._release_requests(identifier, row)
                self._update(identifier, 'paused', 'paused: navigation stopped; waiting for safety recovery')
            return
        self._update(identifier, state, detail, result)

    def _delivery_failed(self, identifier, error):
        self.get_logger().error(f'task acceptance unconfirmed: {error}')
        self._recovering.add(identifier)
        self._update(identifier, 'unknown', 'action acceptance unconfirmed; not resubmitted')

    def _accepted(self, identifier, future):
        self._pending.pop(('send', identifier), None)
        if self._row(identifier)['state'] in TERMINAL:
            return
        try:
            accepted = future.result().accepted
        except Exception as error:
            self._delivery_failed(identifier, error)
            return
        if not accepted:
            self._update(identifier, 'failed', 'ROS action server rejected the goal')
            return
        row = self._row(identifier)
        if not row['cancel_requested'] and identifier not in self._pausing:
            self._update(identifier, 'running', 'ROS action accepted')
        self._lookup_result(self._row(identifier))

    def _canceled(self, identifier, future, server_id=''):
        if future.cancelled():
            return
        key = ('cancel', identifier)
        if self._pending.get(key) is future:
            self._pending.pop(key)
        if self._row(identifier)['state'] in TERMINAL:
            return
        try:
            reply = future.result()
            if (server_id and reply.return_code == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID
                    and self._server_id(self._row(identifier)['capability']) == server_id):
                self._unknown_cancels[identifier] = server_id
            if reply.return_code:
                self.get_logger().warning(f'task cancellation returned {reply.return_code}; terminal result still required', throttle_duration_sec=5)
        except Exception as error:
            self.get_logger().error(f'task cancellation unconfirmed: {error}')

    def _dispatch(self, row, now):
        identifier, capability = row['id'], row['capability']
        goal_id = UUID(uuid=list(row['goal_id']))
        if not self._enabled[capability]:
            self._update(identifier, 'failed', 'capability disabled by this deployment profile before submission')
            return
        if any(self._observed.values()) or self._children.busy():
            return
        if capability == 'navigate':
            self._owner.publish(goal_id)
            if not (self._guard[0] and now - self._guard[1] < 1.0
                    and self._ack[0] == row['goal_id'] and now - self._ack[1] < 1.0):
                return
        client = self._action_clients[capability]
        if not client.server_is_ready():
            return
        parameters = row['parameters']
        if capability in ('explore', 'search'):
            effective = json.loads(row['result']).get('_task', {}).get('effective_parameters')
            if effective is not None:
                parameters = json.dumps(effective, allow_nan=False)
        goal, _ = task_goal(capability, parameters)
        if capability == 'navigate':
            reason = self._navigation.reason(now)
            if reason:
                self._update(identifier, 'paused', f'paused: {reason}')
                return
            if row['remaining_sec'] is None:
                self._journal.navigation_budget(identifier, self._timeout)
        server_id = self._server_id(capability)
        if not server_id:
            return
        self._journal.mark_dispatched(identifier, server_id)
        self._sent.add(identifier)
        self._started[identifier] = now
        admitted = self._admitted_at.pop(identifier, None)
        if admitted is not None:
            self._budget.record('queue', now - admitted)
        self._dispatched_at.setdefault(identifier, now)
        try:
            future = client.send_goal_async(goal, goal_uuid=goal_id)
        except Exception as error:
            self._delivery_failed(identifier, error)
            return
        self._pending[('send', identifier)] = future
        future.add_done_callback(lambda done: self._queue_reply('accepted', row, done))

    def _tick(self):
        while True:
            try:
                reply = self._replies.get_nowait()
            except Empty:
                break
            kind, identifier, goal_id, server_id, future = reply
            if self._row(identifier)['goal_id'] != goal_id:
                # A reply for a superseded attempt is recorded as evidence, never applied as state.
                self._journal.event(identifier, 'late_reply', {'kind': kind, 'goal_id': bytes(goal_id).hex()})
                continue
            if kind == 'accepted':
                self._accepted(identifier, future)
            else:
                {'result': self._result, 'canceled': self._canceled}[kind](identifier, future, server_id)
        now = time.monotonic()
        for row in self._active():
            identifier, capability = row['id'], row['capability']
            goal_id = UUID(uuid=list(row['goal_id']))
            if capability == 'navigate' and not row['cancel_requested']:
                metadata = json.loads(row['result']).get('_task', {})
                limits = metadata.get('parameters', {})
                expired = time.time() >= limits.get('deadline_unix_sec', math.inf)
                origin = metadata.get('odometry_origin')
                if origin:
                    meter = self._distance.status()
                    expired = expired or not meter['ready'] or meter['epoch'] != origin['epoch'] or (
                        meter['meters'] - origin['meters'] >= max(0, limits['max_travel_m'] - .5))
                if expired:
                    self._journal.request_cancel(identifier, reason=2)
                    self._update(identifier, 'canceling', 'bounded navigation budget or telemetry lost; awaiting terminal ROS result')
                    row = self._row(identifier)
            if self._reconcile(row, now):
                continue
            # Result calls wait through execution; only expire after observed termination.
            for kind in ('cancel', 'result'):
                key = (kind, identifier)
                since = self._queried.get(key, now) if kind == 'cancel' else self._terminal_seen.get(identifier, now)
                if key in self._pending and now - since >= 3.0:
                    future = self._pending.pop(key)
                    client = self._cancel_clients[capability] if kind == 'cancel' else self._result_clients[capability]
                    client.remove_pending_request(future)
                    future.cancel()
                    if kind == 'result':
                        self._terminal_seen[identifier] = now
            if identifier in self._terminal_seen:
                self._lookup_result(row)
            if row['navigation_stopped']:
                if row['cancel_requested']:
                    self._update(identifier, 'canceled', 'canceled after confirmed navigation stop')
                elif not self._navigation.reason(now) and not any(self._observed.values()) and not self._children.busy():
                    self._release_requests(identifier, row)
                    self._journal.resume_navigation(identifier, bytes(owned_goal_id().uuid))
                    self._dispatch(self._row(identifier), now)
                continue
            if identifier in self._recovering:
                # Never revive a potentially expired Nav2 ownership lease after a crash.
                self._lookup_result(row)
            elif identifier not in self._sent:
                if row['cancel_requested']:
                    self._update(identifier, 'canceled', 'canceled before action submission')
                    continue
                self._dispatch(row, now)
            elif capability == 'navigate' and not row['cancel_requested']:
                self._navigation_tick(row, now)
            row = self._row(identifier)
            if row['state'] in TERMINAL:
                continue
            if row['cancel_requested'] or identifier in self._pausing or identifier in self._unknown_results:
                client = self._cancel_clients[capability]
                key = ('cancel', identifier)
                if (key not in self._pending and client.service_is_ready()
                        and now - self._queried.get(key, 0.0) >= 1.0):
                    request = CancelGoal.Request()
                    request.goal_info.goal_id = goal_id
                    server_id = self._server_id(capability)
                    future = client.call_async(request)
                    self._queried[key] = now
                    self._pending[key] = future
                    future.add_done_callback(lambda done, task=row, server=server_id: self._queue_reply('canceled', task, done, server))
                self._lookup_result(row)

    def _reconcile(self, row, now):
        identifier, capability = row['id'], row['capability']
        original = row['server_id']
        current = self._unknown_results.get(identifier, '')
        if not original or not current or original == current or self._unknown_cancels.get(identifier) != current:
            return False
        # Let the independent guard expire an orphan's lease before clearing uncertainty.
        if now - self._unknown_since.get(identifier, now) < 3.0:
            return False
        if self._server_id(capability) != current:
            return False
        # The replacement server denied this goal; its old latched status is obsolete.
        self._observed[capability].discard(row['goal_id'])
        if any(self._observed.values()) or self._children.busy():
            return False
        if not self._guard[0] or now - self._guard[1] >= 1.0:
            return False
        detail = 'action server restarted; goal absent and owned navigation stopped; outcome unavailable'
        if row['cancel_requested'] == 2:
            detail = f'navigation deadline reached; {detail}'
        self._update(identifier, 'failed', detail, {'outcome_known': False, 'termination_confirmed': True})
        return True

    def _server_id(self, capability):
        return server_instance(self, ACTIONS[capability][0])

    def _queue_reply(self, kind, row, future, server_id=''):
        self._replies.put((kind, row['id'], row['goal_id'], server_id, future))

    def _navigation_tick(self, row, now):
        identifier = row['id']
        reason = self._navigation.reason(now)
        previous = self._started.get(identifier, now)
        self._started[identifier] = now
        remaining = row['remaining_sec']
        if not reason and identifier not in self._pausing:
            remaining -= max(0.0, now - previous)
            self._journal.navigation_budget(identifier, remaining)
        if remaining <= 0:
            self._journal.request_cancel(identifier, reason=2)
            self._update(identifier, 'canceling', 'navigation deadline reached; awaiting terminal ROS result')
            return
        if identifier in self._pausing and now - self._pause_started[identifier] >= 3.0:
            self._recovering.add(identifier)
            self._update(identifier, 'unknown', 'navigation pause stop unconfirmed; ownership lease no longer renewed')
            return
        self._owner.publish(UUID(uuid=list(row['goal_id'])))
        if reason or identifier in self._pausing:
            if identifier not in self._pausing:
                self._journal.request_navigation_pause(identifier)
            self._pause_started.setdefault(identifier, now)
            self._pausing.add(identifier)
            self._update(identifier, 'paused', f'paused: {reason or "awaiting confirmed navigation stop"}')

    def destroy_node(self):
        for client in self._action_clients.values():
            client.destroy()
        self._journal.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaskGateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
