#!/usr/bin/env python3
"""Durable admission and observation of existing ROS actions; never a motion controller."""

import json
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import time
import uuid

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID

from lekiwi_rmf.action import Explore
from lekiwi_rmf.exploration_owner_guard import owned_goal_id
from lekiwi_rmf.srv import CancelTask, GetTask, SubmitTask
from lekiwi_rmf.task_journal import TERMINAL, TaskJournal


ACTIONS = {'navigate': ('/navigate_to_pose', NavigateToPose), 'explore': ('/robot/explore', Explore)}


def task_goal(capability, text):
    """Keep untrusted commands finite, small and limited to the advertised fields."""
    if len(text.encode()) > 4096:
        raise ValueError('parameters exceed 4096 bytes')
    values = json.loads(text)
    if not isinstance(values, dict):
        raise ValueError('parameters must be an object')
    if capability == 'navigate':
        if set(values) - {'x', 'y', 'yaw'} or not {'x', 'y'} <= values.keys():
            raise ValueError('navigate requires x, y and optional yaw in map coordinates')
        for value in values.values():
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError('navigation coordinates must be finite numbers')
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
        goal = Explore.Goal(revisit_known=values.get('revisit_known', False),
                            max_duration_sec=float(values.get('max_duration_sec', 0.0)),
                            max_radius_m=float(values.get('max_radius_m', 0.0)), resume_task_id=resume)
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
        self._enabled = {'navigate': self.get_parameter('allow_navigation').value,
                         'explore': self.get_parameter('allow_exploration').value}
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
        self._guard = (False, 0.0)
        self._ack = (b'', 0.0)
        self._action_clients = {}
        self._result_clients = {}
        self._cancel_clients = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._events = self.create_publisher(String, '/robot/tasks/events', latched)
        self._owner = self.create_publisher(UUID, '/robot/explore/navigation_owner', 10)
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
        self.create_timer(0.2, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _active(self):
        return self._journal.active()

    def _row(self, identifier):
        return self._journal.row(identifier)

    def _update(self, identifier, state, detail, result=None):
        if not self._journal.update(identifier, state, detail, result):
            return
        row = self._row(identifier)
        self._events.publish(String(data=json.dumps(dict(row), default=lambda value: bytes(value).hex())))
        if state in TERMINAL:
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
                if any(self._observed.values()):
                    raise ValueError('another ROS navigation or exploration client has an active goal')
                if not self._action_clients[request.capability].server_is_ready():
                    raise ValueError('capability action server unavailable')
                goal_id = bytes(owned_goal_id().uuid) if request.capability == 'navigate' else uuid.uuid4().bytes
                self._journal.admit(request.task_id, request.capability, parameters, goal_id)
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

    def _capabilities(self, _request, response):
        response.success = True
        response.message = json.dumps({'schema': 1, 'capabilities': [
            {'name': name, 'available': self._enabled[name] and client.server_is_ready(), 'parameters':
             ['x', 'y', 'yaw'] if name == 'navigate' else ['revisit_known', 'max_duration_sec', 'max_radius_m', 'resume_task_id']}
            for name, client in self._action_clients.items()],
            'services': {'submit': '/robot/tasks/submit', 'get': '/robot/tasks/get', 'cancel': '/robot/tasks/cancel'},
            'events': '/robot/tasks/events'})
        return response

    def _guard_ready(self, message):
        self._guard = message.data, time.monotonic()

    def _guard_ack(self, message):
        self._ack = bytes(message.uuid), time.monotonic()

    def _feedback(self, message):
        row = self._journal.goal(bytes(message.goal_id.uuid))
        if row is None or row['state'] in TERMINAL or row['cancel_requested']:
            return
        stage = getattr(message.feedback, 'stage', 'navigating')
        self._update(row['id'], 'paused' if stage.startswith('paused:') else 'running', stage)

    def _status(self, message, capability):
        self._observed[capability] = {bytes(status.goal_info.goal_id.uuid) for status in message.status_list
                                      if status.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)}
        for status in message.status_list:
            row = self._journal.goal(bytes(status.goal_info.goal_id.uuid))
            if row is not None and row['state'] not in TERMINAL:
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
        future = client.call_async(request)
        self._queried[key] = time.monotonic()
        self._pending[key] = future
        future.add_done_callback(lambda done: self._replies.put(('result', row['id'], done)))

    def _result(self, identifier, future):
        if future.cancelled():
            return
        key = ('result', identifier)
        if self._pending.get(key) is future:
            self._pending.pop(key)
        if self._row(identifier)['state'] in TERMINAL:
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
            self._update(identifier, 'unknown', 'ROS server has no result; delivery or termination remains unconfirmed')
            return
        states = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'failed'}
        if response.status not in states:
            self._update(identifier, 'unknown', 'nonterminal ROS result')
            return
        state = states[response.status]
        if state == 'succeeded' and (result.get('complete') is False or result.get('error_code', 0)):
            state = 'failed'
        detail = result.get('message') or result.get('error_msg') or state
        if self._row(identifier)['cancel_requested'] == 2:
            state, detail = 'failed', f'navigation deadline reached; ROS terminal state: {state}'
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
        if not row['cancel_requested']:
            self._update(identifier, 'running', 'ROS action accepted')
        self._lookup_result(self._row(identifier))

    def _canceled(self, identifier, future):
        if future.cancelled():
            return
        key = ('cancel', identifier)
        if self._pending.get(key) is future:
            self._pending.pop(key)
        if self._row(identifier)['state'] in TERMINAL:
            return
        try:
            reply = future.result()
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
        if any(self._observed.values()):
            return
        if capability == 'navigate':
            self._owner.publish(goal_id)
            if not (self._guard[0] and now - self._guard[1] < 1.0
                    and self._ack[0] == row['goal_id'] and now - self._ack[1] < 1.0):
                return
        client = self._action_clients[capability]
        if not client.server_is_ready():
            return
        goal, _ = task_goal(capability, row['parameters'])
        self._journal.mark_dispatched(identifier)
        self._sent.add(identifier)
        self._started[identifier] = now
        try:
            future = client.send_goal_async(goal, goal_uuid=goal_id)
        except Exception as error:
            self._delivery_failed(identifier, error)
            return
        self._pending[('send', identifier)] = future
        future.add_done_callback(lambda done, task=identifier: self._replies.put(('accepted', task, done)))

    def _tick(self):
        while True:
            try:
                kind, identifier, future = self._replies.get_nowait()
            except Empty:
                break
            {'result': self._result, 'accepted': self._accepted, 'canceled': self._canceled}[kind](identifier, future)
        now = time.monotonic()
        for row in self._active():
            identifier, capability = row['id'], row['capability']
            goal_id = UUID(uuid=list(row['goal_id']))
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
            if identifier in self._recovering:
                # Never revive a potentially expired Nav2 ownership lease after a crash.
                self._lookup_result(row)
            elif identifier not in self._sent:
                if row['cancel_requested']:
                    self._update(identifier, 'canceled', 'canceled before action submission')
                    continue
                self._dispatch(row, now)
            elif capability == 'navigate' and not row['cancel_requested']:
                if now - self._started[identifier] >= self._timeout:
                    self._journal.request_cancel(identifier, reason=2)
                    self._update(identifier, 'canceling', 'navigation deadline reached; awaiting terminal ROS result')
                else:
                    self._owner.publish(goal_id)
            if row['cancel_requested']:
                client = self._cancel_clients[capability]
                key = ('cancel', identifier)
                if (key not in self._pending and client.service_is_ready()
                        and now - self._queried.get(key, 0.0) >= 1.0):
                    request = CancelGoal.Request()
                    request.goal_info.goal_id = goal_id
                    future = client.call_async(request)
                    self._queried[key] = now
                    self._pending[key] = future
                    future.add_done_callback(lambda done, task=identifier: self._replies.put(('canceled', task, done)))
                self._lookup_result(row)

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
