#!/usr/bin/env python3
"""Run registered skill implementations through the arm's existing guarded trajectory path.

Receding horizon: observe, ask the implementation for one action chunk, send it as
one FollowJointTrajectory goal that the driver validates and the host executes,
wait for its result, then observe again. Nothing here bypasses arming, permissions,
calibration or joint limits, and a finished run never claims the goal succeeded.
"""

import json
import threading
import time

from control_msgs.action import FollowJointTrajectory
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from lekiwi_rmf.action import RunSkill
from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.object_recognition import reject_parameter_changes
from lekiwi_rmf.skills import arm_targets, catalog, load_registry, skill_parameters, verify_artifact

CAMERA_TOPICS = {'front': '/camera/front/image_raw', 'wrist': '/camera/wrist/image_raw',
                 'astra': '/camera/astra/color/image_raw'}


def rgb_pixels(message):
    # cv_bridge is built against NumPy 1 and the policy environment uses NumPy 2.
    if message.encoding not in ('rgb8', 'bgr8') or message.is_bigendian or message.step < message.width * 3:
        raise RuntimeError(f'unsupported camera encoding {message.encoding}')
    image = np.frombuffer(bytes(message.data), np.uint8).reshape(message.height, message.step)
    image = image[:, :message.width * 3].reshape(message.height, message.width, 3)
    return np.ascontiguousarray(image[..., ::-1] if message.encoding == 'bgr8' else image)


class LeRobotPolicy:
    """A trained LeRobot policy with its saved normalization, loaded from a verified checkpoint."""

    def __init__(self, path):
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        config = PreTrainedConfig.from_pretrained(path)
        self.torch = torch
        self.policy = get_policy_class(config.type).from_pretrained(path).eval()
        self.pre, self.post = make_pre_post_processors(self.policy.config, pretrained_path=str(path))
        self.image_shapes = {name: tuple(feature.shape) for name, feature in self.policy.config.input_features.items()
                             if name.startswith('observation.images.')}

    def reset(self):
        self.policy.reset()

    def chunk(self, state, images, task):
        import cv2
        observation = {'observation.state': self.torch.tensor(state, dtype=self.torch.float32), 'task': task}
        for camera, pixels in images.items():
            key = f'observation.images.{camera}'
            channels, height, width = self.image_shapes[key]
            resized = cv2.resize(pixels, (width, height)) if pixels.shape[:2] != (height, width) else pixels
            observation[key] = self.torch.from_numpy(resized).permute(2, 0, 1).float() / 255.0
        with self.torch.inference_mode():
            actions = self.post(self.policy.predict_action_chunk(self.pre(observation)))
        return np.asarray(actions[0].cpu())


class SkillRunner(Node):
    def __init__(self, policy_factory=LeRobotPolicy, **kwargs):
        super().__init__('skill_runner', **kwargs)
        self.declare_parameter('registry_file', '')
        self.declare_parameter('artifact_directory', '')
        self.declare_parameter('allow_experimental', False)
        self.declare_parameter('max_observation_age_sec', .5)
        reject_parameter_changes(self, ['registry_file', 'artifact_directory', 'allow_experimental',
                                        'max_observation_age_sec'])
        self._registry = load_registry(self.get_parameter('registry_file').value or None,
                                       self.get_parameter('artifact_directory').value or None)
        self._experimental = self.get_parameter('allow_experimental').value
        self._max_age = self.get_parameter('max_observation_age_sec').value
        if not 0 < self._max_age <= 2:
            raise ValueError('max_observation_age_sec must be in (0, 2]')
        self._policy_factory, self._policies = policy_factory, {}
        self._lock, self._busy = threading.Lock(), False
        self._joints, self._images = None, {}
        group = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._joint_state, 10, callback_group=group)
        for camera in sorted({camera for skill in self._registry.values() for camera in skill.cameras}):
            self.create_subscription(Image, CAMERA_TOPICS[camera], lambda message, name=camera: self._image(name, message),
                                     qos_profile_sensor_data, callback_group=group)
        self._arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
                                 callback_group=group)
        self._server = ActionServer(self, RunSkill, '/robot/skills/run', self._execute, goal_callback=self._accept,
                                    cancel_callback=lambda _goal: CancelResponse.ACCEPT, callback_group=group)

    def catalog(self):
        return catalog(self._registry, self._experimental)

    def _joint_state(self, message):
        with self._lock:
            self._joints = message

    def _image(self, camera, message):
        with self._lock:
            self._images[camera] = message

    def _admitted(self, request):
        skill = self._registry.get(request.skill_id)
        if skill is None:
            raise ValueError('unknown skill')
        if skill.status != 'qualified' and not self._experimental:
            raise ValueError('skill is experimental; only an acquisition protocol may run it')
        return skill, skill_parameters(skill, request.parameters_json)

    def _accept(self, request):
        try:
            self._admitted(request)
        except ValueError as error:
            self.get_logger().warn(f'Rejecting skill {request.skill_id!r}: {error}')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn('Rejecting skill: another skill owns the arm')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _age(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        return self.get_clock().now().nanoseconds * 1e-9 - stamp

    def _observation(self, skill):
        with self._lock:
            joints, images = self._joints, {camera: self._images.get(camera) for camera in skill.cameras}
        if joints is None or not 0 <= self._age(joints) <= self._max_age:
            raise RuntimeError('joint state is missing or stale')
        measured = dict(zip(joints.name, joints.position))
        if set(ARM_JOINTS) - set(measured):
            raise RuntimeError('joint state lacks arm joints')
        pixels = {}
        for camera, message in images.items():
            if message is None or not 0 <= self._age(message) <= self._max_age:
                raise RuntimeError(f'{camera} image is missing or stale')
            pixels[camera] = rgb_pixels(message)
        return measured, pixels

    def _wait(self, future, timeout, goal_handle=None):
        deadline = time.monotonic() + timeout
        while not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return None
            if time.monotonic() >= deadline or not rclpy.ok(context=self.context):
                raise RuntimeError('arm controller did not answer in time')
            time.sleep(.02)
        return future.result()

    def _send_chunk(self, skill, rows, goal_handle):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        for index, row in enumerate(rows, 1):
            point = JointTrajectoryPoint(positions=[row[name] for name in ARM_JOINTS])
            seconds = index / skill.fps
            point.time_from_start.sec, point.time_from_start.nanosec = int(seconds), int(seconds % 1 * 1e9)
            goal.trajectory.points.append(point)
        handle = self._wait(self._arm.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            raise RuntimeError('arm controller rejected the chunk (disarmed, no permission or invalid targets)')
        result = self._wait(handle.get_result_async(), len(rows) / skill.fps + 5.0, goal_handle)
        if result is None:
            self._wait(handle.cancel_goal_async(), 5.0)
            return False
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'arm chunk failed: {result.result.error_string or result.result.error_code}')
        return True

    def _execute(self, goal_handle):
        request, started, chunks = goal_handle.request, time.monotonic(), 0
        result, detail, last = RunSkill.Result(), '', None
        try:
            skill, parameters = self._admitted(request)
            policy = self._policies.get(skill.skill_id)
            if policy is None:
                policy = self._policies[skill.skill_id] = self._policy_factory(verify_artifact(skill))
            policy.reset()
            task = json.dumps({'skill': skill.skill_id, **parameters}, sort_keys=True)
            while time.monotonic() - started < skill.max_duration_sec:
                if goal_handle.is_cancel_requested:
                    raise InterruptedError
                measured, images = self._observation(skill)
                chunk = policy.chunk([measured[name] for name in ARM_JOINTS], images, task)
                rows = arm_targets(skill, measured, chunk)
                if not self._send_chunk(skill, rows, goal_handle):
                    raise InterruptedError
                chunks, last = chunks + 1, rows[-1]
                goal_handle.publish_feedback(RunSkill.Feedback(
                    stage='executing', chunks_executed=chunks, elapsed_sec=time.monotonic() - started))
            result.outcome = 'finished'
            goal_handle.succeed()
        except InterruptedError:
            result.outcome, detail = 'canceled', 'canceled before the run finished'
            goal_handle.canceled()
        except (RuntimeError, ValueError, OSError) as error:
            result.outcome, detail = 'failed', str(error)
            goal_handle.abort()
        finally:
            with self._lock:
                self._busy = False
        skill = self._registry.get(request.skill_id)
        result.detail = detail
        result.result_json = json.dumps({
            'skill_id': request.skill_id, 'version': skill.version if skill else None,
            'weights_sha256': skill.weights_sha256 if skill else None, 'chunks_executed': chunks,
            'elapsed_sec': time.monotonic() - started, 'last_targets': last,
            'verification': 'not performed by the runner'}, sort_keys=True)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = SkillRunner()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
