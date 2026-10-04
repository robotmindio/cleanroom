#!/usr/bin/env python3
"""Finite live obstacle and MoveIt workspace gates; restores production.

Obstacle uses a real object in the lidar StopZone. Workspace uses an explicit
temporary MoveIt collision object and verifies the real arm rejects a goal.
Neither case edits named poses, collision padding or physical acceptance.
"""
import argparse
import json
import math
from pathlib import Path
import runpy
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray

nav = runpy.run_path(str(Path(__file__).with_name('test-navigation.py')))


class Gates(nav['Test']):
    def __init__(self, case):
        super().__init__()
        self.case, self.checks, self.points, self.safe, self.joints = case, {}, [], None, None
        self.create_subscription(MarkerArray, '/collision_monitor/collision_points_marker',
                                 self.obstacles, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', lambda m:setattr(self, 'safe', m), 10)
        self.create_subscription(JointState, '/joint_states', lambda m:setattr(self, 'joints', m), 10)
        self.create_subscription(Bool, '/safety/arm_workspace_collision',
            lambda m:self.flags.update(collision=m.data), 10)
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def tick(self, twist=None, check=True):
        # Confirmed intrusion intentionally withdraws permission and may disarm.
        # The tested goals and every wait remain finite; no base motion is used.
        super().tick(twist, check=False)
        if time.monotonic()>self.deadline:
            raise RuntimeError('gate-test deadline expired')

    def obstacles(self, message):
        self.points = [[p.x, p.y] for m in message.markers
                       if m.header.frame_id == 'base_footprint' for p in m.points
                       if -.27 < p.x < .29 and -.27 < p.y < .27]

    def scene_object(self, operation):
        obj = CollisionObject(id='qualification_workspace_intrusion', operation=operation)
        obj.header.frame_id = 'so101_wrist_link'
        if operation == CollisionObject.ADD:
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.03,.03,.03])]
            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.orientation.w = 1.0
            obj.primitive_poses = [pose]
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [obj]
        future = self.scene.call_async(request)
        self.wait(future.done, 5)
        if not future.result().success:
            raise RuntimeError('MoveIt did not apply the qualification scene object')

    def run(self):
        self.wait_ready()
        self.center, self.active = self.pose, True
        self.wait(lambda:self.flags.get('base_motion_permitted') and self.joints is not None, 10)
        if self.case == 'obstacle':
            self.wait(lambda:len(self.points)>0, 5)
            command = Twist()
            command.linear.x = .04
            end = time.monotonic()+2
            wheel = []
            while time.monotonic()<end:
                self.tick(command, check=False)
                wheel.append(self.pose)
                if math.dist(self.center[:2],self.pose[:2])>.005:
                    raise RuntimeError('obstacle gate allowed physical movement')
            self.stop()
            if not (self.monitor_action == ('StopZone',nav['CollisionMonitorState'].STOP)
                    and self.safe is not None and self.safe.linear.x == 0
                    and self.flags.get('base_motion_permitted')):
                raise RuntimeError('obstacle did not independently stop the authorized command')
            self.checks['collision_monitor_obstacle_stop'] = {
                'passed':True, 'input':'real lidar obstacle', 'points_in_stop_zone':self.points,
                'monitor':self.monitor_action, 'requested_linear_m_s':.04,
                'wheel_max_translation_m':max(math.dist(self.center[:2],p[:2]) for p in wheel)}
        else:
            self.wait(lambda:self.scene.service_is_ready() and self.validity.service_is_ready()
                      and self.arm.server_is_ready(), 10)
            before = dict(zip(self.joints.name,self.joints.position))
            try:
                self.scene_object(CollisionObject.ADD)
                self.wait(lambda:self.flags.get('collision') and
                          not self.flags.get('arm_motion_permitted'), 5)
                request = GetStateValidity.Request(group_name='arm')
                request.robot_state.joint_state = self.joints
                future = self.validity.call_async(request)
                self.wait(future.done, 5)
                response = future.result()
                contacts = [[c.contact_body_1,c.contact_body_2] for c in response.contacts]
                if response.valid or not any('qualification_workspace_intrusion' in c for c in contacts):
                    raise RuntimeError('MoveIt did not report the injected real geometry collision')
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = list(self.joints.name)
                point = JointTrajectoryPoint(positions=list(self.joints.position))
                point.positions[goal.trajectory.joint_names.index('arm_wrist_roll')] += .02
                point.time_from_start.sec = 1
                goal.trajectory.points = [point]
                future = self.arm.send_goal_async(goal)
                self.wait(future.done, 5)
                handle = future.result()
                if handle.accepted:
                    result = handle.get_result_async()
                    self.wait(result.done, 5)
                    if result.result().status == 4:
                        raise RuntimeError('arm action succeeded with a confirmed collision')
                end = time.monotonic()+1
                while time.monotonic()<end:self.tick(Twist(),check=False)
                delta = max(abs(p-before[n]) for n,p in zip(self.joints.name,self.joints.position))
                if delta>.02:
                    raise RuntimeError('arm moved during the collision gate test')
                self.checks['arm_workspace_intrusion_stop'] = {
                    'passed':True, 'input':'temporary MoveIt collision object at wrist',
                    'contacts':contacts, 'action_accepted':handle.accepted,
                    'maximum_joint_change_rad':delta}
            finally:
                self.scene_object(CollisionObject.REMOVE)
            self.wait(lambda:not self.flags.get('collision') and
                      self.flags.get('arm_motion_permitted') and self.flags.get('driver')=='ARMED', 30)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=['obstacle','workspace'])
    args = parser.parse_args()
    output = nav['ROOT']/'.benchmarks/acceptance-gates'/time.strftime('%Y%m%d-%H%M%S')
    nav['main'](lambda:Gates(args.case), output)
