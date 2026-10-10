#!/usr/bin/env python3
"""Operator exploration client with meaningful exit codes and owned cancellation."""

import signal
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from unique_identifier_msgs.msg import UUID

from lekiwi_rmf.action import Explore


class ExploreClient(Node):
    def __init__(self, **kwargs):
        super().__init__('explore_operator', **kwargs)
        self.action = ActionClient(self, Explore, '/robot/explore')
        self.cancel = self.create_client(CancelGoal, '/robot/explore/_action/cancel_goal')
        self.result = self.create_client(Explore.Impl.GetResultService, '/robot/explore/_action/get_result')
        self.reason = 'no exploration status received'
        self.status_at = time.monotonic()
        self.ready = False
        self.busy = None
        self._prerequisites_ready = False
        self._idle_since = None
        self._sent_id = self._sent_future = None
        self.create_subscription(DiagnosticArray, '/diagnostics', self._status, 10)

    def destroy_node(self):
        self.action.destroy()
        return super().destroy_node()

    def _status(self, message):
        for status in message.status:
            if status.name == 'lekiwi/exploration':
                values = {item.key: item.value for item in status.values}
                self.reason = values.get('last_rejection') or status.message
                self.busy = {'true': True, 'false': False}.get(values.get('busy'))
                self.ready = status.message == 'ready' and self.busy is not None
                self._prerequisites_ready = values.get('prerequisite_reason') == ''
                if self.busy is not None:
                    self.status_at = time.monotonic()
                if self.busy is False and self._sent_id is not None:
                    if self._idle_since is None:
                        self._idle_since = time.monotonic()
                else:
                    self._idle_since = None

    def wait(self, predicate, seconds, interrupted=None):
        until = time.monotonic() + seconds
        while rclpy.ok(context=self.context) and not predicate():
            if time.monotonic() >= until or (interrupted and interrupted.is_set()):
                return False
            rclpy.spin_once(self, executor=self._client_executor, timeout_sec=0.1)
        return bool(predicate())

    def run(self, interrupted):
        self._client_executor = SingleThreadedExecutor(context=self.context)
        self._client_executor.add_node(self)
        try:
            try:
                return self._run(interrupted)
            except Exception as error:
                print(f'Exploration client failed: {error}', flush=True)
                if self._sent_id is not None:
                    return self._stop(self._sent_id, self._sent_future, interrupted)
                return 2
        finally:
            self._client_executor.remove_node(self)
            self._client_executor.shutdown()

    def _run(self, interrupted):
        task_id = None
        while True:
            outcome, task_id = self._attempt(interrupted, task_id)
            if outcome is not None:
                return outcome
            print('Exploration server recovered; resuming the original task bounds and progress', flush=True)

    def _attempt(self, interrupted, task_id):
        requested_at = time.monotonic()
        if not self.wait(lambda: self.action.server_is_ready() and self.ready
                         and self.busy is False and time.monotonic() - self.status_at < 1
                         and (task_id is None or self.status_at > requested_at), 10, interrupted):
            print(f'Exploration unavailable: {self.reason}', flush=True)
            return (130 if interrupted.is_set() else 2), task_id
        goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        request = Explore.Goal(revisit_known=True, resume_task_id=task_id or '')
        task_id = task_id or bytes(goal_id.uuid).hex()
        future = self.action.send_goal_async(request, goal_uuid=goal_id,
                                            feedback_callback=lambda msg: print(msg.feedback.stage, flush=True))
        self._sent_id, self._sent_future = goal_id, future
        if self.wait(future.done, 10, interrupted):
            handle = future.result()
            if not handle.accepted:
                # Let the periodic diagnostic carry the goal callback's refusal reason.
                self.wait(lambda: False, 0.5)
                print(f'Exploration rejected: {self.reason}', flush=True)
                return 2, task_id
            result = handle.get_result_async()
            self._idle_since = None
            # The tracked server owns the task duration; Ctrl-C cancels this exact UUID.
            while (rclpy.ok(context=self.context) and not result.done() and not interrupted.is_set()
                   and time.monotonic() - self.status_at < 10
                   and (self._idle_since is None or time.monotonic() - self._idle_since < 1)):
                rclpy.spin_once(self, executor=self._client_executor, timeout_sec=0.1)
            if result.done():
                response = result.result()
                if response.status != GoalStatus.STATUS_UNKNOWN:
                    return self._finished(response, interrupted), task_id
                # Failover may answer UNKNOWN before the restarted peer's idle status arrives.
                self.wait(lambda: self._idle_since is not None and self.busy is False
                          and self._prerequisites_ready and time.monotonic() - self.status_at < 1
                          and time.monotonic() - self._idle_since >= 1, 10, interrupted)
            if self._idle_since is not None and time.monotonic() - self._idle_since >= 1:
                print('Exploration task is no longer active; recovering its result or confirming its stop', flush=True)
                if self.result.service_is_ready() and not interrupted.is_set():
                    reply = self.result.call_async(Explore.Impl.GetResultService.Request(goal_id=goal_id))
                    if self.wait(reply.done, 1, interrupted):
                        response = reply.result()
                        if response.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                               GoalStatus.STATUS_ABORTED):
                            return self._finished(response, interrupted), task_id
                        if (response.status == GoalStatus.STATUS_UNKNOWN and self.busy is False
                                and self._prerequisites_ready and time.monotonic() - self.status_at < 1
                                and not interrupted.is_set()):
                            return None, task_id
        return self._stop(goal_id, future, interrupted, allow_resume=True), task_id

    def _finished(self, response, interrupted):
        print(response.result.message, flush=True)
        if (response.status == GoalStatus.STATUS_ABORTED
                and response.result.message == 'exploration server is shutting down'
                and not interrupted.is_set()):
            return None
        return 0 if response.status == GoalStatus.STATUS_SUCCEEDED else 1

    def _stop(self, goal_id, future, interrupted, allow_resume=False):
        # Acceptance can arrive late: cancellation still targets the UUID we sent.
        until = time.monotonic() + 10
        terminal = False
        result_status = None
        while time.monotonic() < until and rclpy.ok(context=self.context):
            accepted = False
            if self.cancel.service_is_ready():
                request = CancelGoal.Request()
                request.goal_info.goal_id = goal_id
                cancellation = self.cancel.call_async(request)
                self.wait(cancellation.done, 1)
            if future.done():
                try:
                    accepted = future.result().accepted
                    rejected = not accepted
                except Exception:
                    rejected = False  # The exact UUID can still be canceled without a client handle.
                if rejected:
                    terminal = True
                    break
            if self.result.service_is_ready():
                reply = self.result.call_async(Explore.Impl.GetResultService.Request(goal_id=goal_id))
                if self.wait(reply.done, 1):
                    result_status = reply.result().status
                    if result_status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                        terminal = True
                        break
                    # Fresh idle prerequisites include the independent navigation ownership guard.
                    if (result_status == GoalStatus.STATUS_UNKNOWN and accepted and self.busy is False
                            and self._prerequisites_ready and time.monotonic() - self.status_at < 1):
                        print('Exploration server lost this task; owned navigation has stopped', flush=True)
                        terminal = True
                        break
            self.wait(lambda: False, 0.1)
        if (terminal and result_status == GoalStatus.STATUS_UNKNOWN
                and allow_resume and not interrupted.is_set()):
            return None
        print('Exploration stopped' if terminal else 'Exploration stop unconfirmed; check robot status', flush=True)
        if interrupted.is_set() and terminal:
            return 130
        if terminal and result_status == GoalStatus.STATUS_SUCCEEDED:
            return 0
        if terminal and result_status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            return 1
        return 2


def main(args=None):
    interrupted = threading.Event()
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _sig, _frame: interrupted.set())
    node = ExploreClient()
    try:
        code = node.run(interrupted)
    except Exception as error:
        print(f'Exploration client failed: {error}', flush=True)
        code = 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
