#!/usr/bin/env python3
"""Durable admission and observation of existing ROS actions; never a motion controller."""

import json
import hashlib
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import time
import uuid

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
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

from lekiwi_rmf.action import Explore, RunSkill
from lekiwi_rmf.exploration import task_limits
from lekiwi_rmf.exploration_owner_guard import owned_goal_id
from lekiwi_rmf.srv import CancelTask, GetTask, SubmitTask
from lekiwi_rmf.task_journal import TERMINAL, TaskJournal
from lekiwi_rmf.navigation_pause import NavigationPause
from lekiwi_rmf.navigation_observer import NavigationObserver, server_instance
from lekiwi_rmf.navigation_distance import NavigationDistance
from lekiwi_rmf.map_bundle import validate_map_bundle
from lekiwi_rmf.skills import SKILL_ID, load_registry


ACTIONS = {'navigate': ('/navigate_to_pose', NavigateToPose), 'explore': ('/robot/explore', Explore),
           'skill': ('/robot/skills/run', RunSkill)}


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
        self.declare_parameter('map_bundle', '')
        self._enabled = {'navigate': self.get_parameter('allow_navigation').value,
                         'explore': self.get_parameter('allow_exploration').value,
                         'skill': self.get_parameter('allow_skills').value}
        try:
            self._skills = [skill.catalog() for skill in load_registry().values()]
        except (OSError, ValueError) as error:
            self.get_logger().error(f'skill registry unavailable: {error}')
            self._enabled['skill'], self._skills = False, []
        self._timeout = self.get_parameter('navigation_timeout_sec').value
        if type(self._timeout) not in (float, int) or not math.isfinite(self._timeout) or self._timeout <= 0:
            raise ValueError('navigation_timeout_sec must be positive and finite')
        self._journal = TaskJournal(self.get_parameter('database_path').value)
        self._pending = {}
        self._replies = SimpleQueue()
        self._queried = {}
        self._sent = set()
        self._started = {}
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
        self._observation_status = self.create_client(Trigger, '/robot/observations/status')
        self._distance = NavigationDistance()
        self.create_subscription(Odometry, '/odom', self._distance.update, 1)
        self.create_service(Trigger, '/robot/navigation/odometry', self._odometry)
        self.create_timer(0.2, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

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

    def _submit(self, request, response):
        try:
            if not 1 <= len(request.task_id) <= 128 or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.' for c in request.task_id):
                raise ValueError('task_id requires 1..128 ASCII letters, numbers, dots, hyphens or underscores')
            _, parameters = task_goal(request.capability, request.parameters_json)
            row = self._row(request.task_id)
            if row is not None:
                if (row['capability'], row['parameters']) != (request.capability, parameters):
                    raise ValueError('task_id already has different parameters')
            else:
                if not self._enabled[request.capability]:
                    raise ValueError('capability disabled by this deployment profile')
                if self._active():
                    raise ValueError('another managed task is active or its result is unconfirmed')
                if any(self._observed.values()) or self._children.busy():
                    raise ValueError('another ROS navigation or exploration client has an active goal')
                if not self._action_clients[request.capability].server_is_ready():
                    raise ValueError('capability action server unavailable')
                goal_id = bytes(owned_goal_id().uuid) if request.capability == 'navigate' else uuid.uuid4().bytes
                values = json.loads(parameters)
                effective = None
                if request.capability == 'navigate' and 'deadline_unix_sec' in values and not (
                        time.time() < values['deadline_unix_sec'] <= time.time() + 300):
                    raise ValueError('navigation deadline must be within the next 300 seconds')
                if request.capability == 'navigate':
                    effective = {**values, 'yaw': values.get('yaw', 0.0),
                                 'max_duration_sec': self._timeout}
                elif self._exploration_defaults is not None and not values.get('resume_task_id'):
                    duration, radius = task_limits(
                        values.get('max_duration_sec', 0.0), values.get('max_radius_m', 0.0),
                        self._exploration_defaults['max_duration_sec'], self._exploration_defaults['max_radius_m'])
                    effective = {'max_duration_sec': duration, 'max_radius_m': radius}
                    effective['revisit_known'] = values.get('revisit_known', False)
                    effective['resume_task_id'] = ''
                metadata = {'parameters': values, 'effective_parameters': effective}
                if request.capability == 'navigate' and 'max_travel_m' in values:
                    meter = self._distance.status()
                    if not meter['ready']:
                        raise ValueError('bounded navigation requires fresh odometry')
                    metadata['odometry_origin'] = {'epoch': meter['epoch'], 'meters': meter['meters']}
                self._journal.admit(request.task_id, request.capability, parameters, goal_id,
                                    result={'_task': metadata})
                row = self._row(request.task_id)
            response.accepted, response.state, response.detail = True, row['state'], row['detail']
        except (ValueError, TypeError, OverflowError) as error:
            response.accepted, response.state, response.detail = False, 'rejected', str(error)
        return response

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
            self._journal.request_cancel(identifier)
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
                    'description': 'Omit parameters to explore unmapped space. revisit_known=true also revisits mapped space. '
                                   'Omitted/zero limits use the tracked robot limits; positive limits may only reduce them. '
                                   'Duration is active seconds, radius is metres around the starting pose. '
                                   'resume_task_id is a saved native exploration UUID and preserves its original bounds.'}
        return {**entry, 'parameters': ['skill_id', 'parameters'], 'required_parameters': ['skill_id'],
                'defaults': {'parameters': {}}, 'effective_defaults': {'parameters': {}}, 'skills': self._skills,
                'description': 'Run a registered skill implementation on the arm. Success means the implementation '
                               'finished, not that the world reached the goal; verify the effect independently.'}

    def _capabilities(self, _request, response):
        response.success = True
        response.message = json.dumps({'schema': 1, 'capabilities': [
            self._capability(name, client) for name, client in self._action_clients.items()],
            'terminal_states': sorted(TERMINAL),
            'observations': {'available': self._observation_status.service_is_ready(),
                             'schema': 1, 'grounding': 'image_only',
                             'services': {'query': '/robot/observations/query',
                                          'evidence': '/robot/observations/evidence',
                                          'pin': '/robot/observations/pin',
                                          'status': '/robot/observations/status'},
                             'events': '/robot/observations/events'},
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
        if row is None or row['state'] in TERMINAL or row['cancel_requested'] or row['navigation_stopped']:
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
        if capability == 'explore':
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
