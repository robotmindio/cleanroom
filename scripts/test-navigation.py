#!/usr/bin/env python3
"""Run a finite attended base/SLAM test, then restore the production service.

Source scripts/setup.bash first. Requires a clear 30 cm radius around the base
center and the arm already in travel_stow. Never changes physical acceptance.
"""
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import signal
import shlex
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import ManageLifecycleNodes
from rtabmap_msgs.msg import Info
from rtabmap_msgs.srv import GetMap
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from lekiwi_rmf.motion_guards import load_base_speed_limits

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / '.benchmarks/navigation-test'


def installed_stack_arguments(path=Path('/etc/default/lekiwi-stack')):
    for line in path.read_text().splitlines():
        if line.startswith('LEKIWI_STACK_ARGS='):
            args = shlex.split(line.partition('=')[2])
            # systemd EnvironmentFile permits an unquoted value with spaces.
            if len(args)==1:
                args = shlex.split(args[0])
            if not args or any(':=' not in arg for arg in args):
                raise ValueError('invalid installed LEKIWI_STACK_ARGS')
            return args
    raise ValueError('installed LEKIWI_STACK_ARGS is missing')


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class Test(Node):
    def __init__(self):
        super().__init__('bounded_navigation_test')
        self.active = False
        self.flags = {}
        self.pose = None
        self.odom_at = 0.0
        self.trace = []
        self.slam = []
        self.counts = {}
        self.center = None
        self.deadline = time.monotonic() + 180
        self.lease = self.create_publisher(Bool, '/safety/base_test_active', 1)
        self.command = self.create_publisher(Twist, '/cmd_vel_manual', 1)
        for topic in ['arm_stowed', 'base_motion_permitted', 'arm_motion_permitted']:
            self.create_subscription(Bool, '/safety/'+topic,
                lambda m,t=topic:self.flags.update({t:m.data}), 10)
        self.create_subscription(String, '/safety/driver_state',
            lambda m:self.flags.update(driver=m.data), 10)
        self.create_subscription(Odometry, '/wheel/odometry', self.odometry, 10)
        self.create_subscription(Info, '/info', self.info, 10)
        for topic,kind in [('/camera/front/image_raw',Image), ('/camera/depth/points',PointCloud2),
                           ('/scan',LaserScan), ('/slam/cloud',PointCloud2)]:
            self.create_subscription(kind, topic,
                lambda m,t=topic:self.counts.update({t:self.counts.get(t,0)+1}), qos_profile_sensor_data)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.navigation = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal = None
        self.health = {}
        self.health_faults = []
        self.monitor_action = None
        self.blocked_at = None
        self.motion_pauses = 0
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
            lambda m:setattr(self,'monitor_action',(m.polygon_name,m.action_type)), 10)
        self.map_client = self.create_client(GetMap, '/rtabmap/get_map_data')
        self.lifecycle_client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.monitor_state = self.create_client(GetState,'/collision_monitor/get_state')
        self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, 10)

    def diagnostics(self, m):
        for status in m.status:
            if status.name=='lekiwi/safety_supervisor':
                self.health = {v.key:v.value for v in status.values}
                if self.health.get('faults'):
                    self.health_faults.append(self.health.copy())

    def odometry(self, m):
        p = m.pose.pose.position
        self.pose = (p.x,p.y,yaw(m.pose.pose.orientation))
        self.odom_at = time.monotonic()
        self.trace.append(self.pose)

    def info(self, m):
        stats = dict(zip(m.stats_keys,m.stats_values))
        self.slam.append({'ref':m.ref_id, 'loop':m.loop_closure_id,
            'proximity':m.proximity_detection_id, 'wm':list(m.wm_state),
            'stats':{k:v for k,v in stats.items() if k.startswith(('Loop/','Memory/Rehearsal'))}})

    def tick(self, twist=None, check=True):
        self.lease.publish(Bool(data=self.active))
        requested = twist is not None and any((twist.linear.x,twist.linear.y,twist.angular.z))
        if twist is not None and not requested:
            self.command.publish(twist)
        rclpy.spin_once(self,timeout_sec=0.04)
        # Camera processing can take a frame period. Drain ready callbacks so
        # high-rate sensor topics cannot leave our pose/permissions queued.
        for _ in range(20):
            rclpy.spin_once(self,timeout_sec=0)
        # Fault injection may deliberately trip StopZone; ordinary positioning
        # still needs its collision guard when fault tests skip feedback checks.
        if requested and (check or not getattr(self,'phase',None)) and self.monitor_action == ('StopZone',CollisionMonitorState.STOP):
            if self.blocked_at is None:
                self.blocked_at = time.monotonic()
            elif time.monotonic()-self.blocked_at > 3:
                self.command.publish(Twist())
                raise RuntimeError('collision-monitor StopZone blocks motion')
        else:
            self.blocked_at = None
        if check and self.center is not None:
            self.check_feedback()
        if requested:
            self.command.publish(twist)

    def check_feedback(self):
        if time.monotonic()>self.deadline:
            raise RuntimeError('total test deadline expired')
        def unverified():
            return (time.monotonic()-self.odom_at>0.5 or
                    not self.flags.get('arm_stowed') or self.flags.get('driver')!='ARMED')
        if unverified():
            self.pause_until(lambda:not unverified(),'verified motion feedback')
        if math.dist(self.pose[:2],self.center[:2])>=0.18:
            raise RuntimeError('early 18 cm test boundary reached')

    def wait(self, condition, seconds):
        end = time.monotonic()+seconds
        while not condition():
            if time.monotonic()>end:
                raise RuntimeError('timed out waiting for '+str(condition))
            self.tick()

    def stop(self):
        end = time.monotonic()+0.6
        while time.monotonic()<end:
            self.tick(Twist(),check=False)

    def pause_until(self,condition,reason):
        print('paused for',reason,self.health,flush=True)
        was_active = self.active
        self.active = False
        self.motion_pauses += 1
        start = time.monotonic()
        while not condition():
            self.tick(Twist(),check=False)
            if time.monotonic()-start>3:
                raise RuntimeError(reason+' did not recover: '+str(self.health))
        self.active = was_active

    def wait_ready(self):
        self.wait(lambda:self.pose is not None and self.flags.get('arm_stowed')
            and self.flags.get('driver')=='ARMED',70)
        self.wait(lambda:self.monitor_state.service_is_ready(),20)
        end = time.monotonic()+30
        while True:
            future = self.monitor_state.call_async(GetState.Request())
            self.wait(future.done,5)
            if future.result().current_state.id==3:
                return
            if time.monotonic()>end:
                raise RuntimeError('collision monitor did not activate')
            next_query = time.monotonic()+0.5
            while time.monotonic()<next_query:
                self.tick(Twist())

    def move(self, target, linear_limit=0.025, angular_limit=0.15, pose_source=None):
        print('manual target',target,flush=True)
        # Collision monitoring legitimately scales manual commands to 35% near
        # obstacles; a 0.6 rad reversal plus proportional settling exceeds 12 s.
        end = time.monotonic()+60
        while True:
            x,y,a = self.pose if pose_source is None else pose_source()
            dx,dy = target[0]-x,target[1]-y
            da = angle(target[2]-a)
            if math.hypot(dx,dy)<0.008 and abs(da)<0.03:
                break
            if time.monotonic()>end:
                raise RuntimeError(f'manual motion did not reach target; collision monitor={self.monitor_action}')
            if not self.flags.get('base_motion_permitted'):
                print('paused for permission',self.health,flush=True)
                self.stop()
                self.wait(lambda:self.flags.get('base_motion_permitted'),3)
                continue
            command = Twist()
            norm = max(math.hypot(dx,dy),0.001)
            speed = min(linear_limit, 2*norm)
            command.linear.x = (math.cos(a)*dx+math.sin(a)*dy)/norm*speed
            command.linear.y = (-math.sin(a)*dx+math.cos(a)*dy)/norm*speed
            command.angular.z = max(-angular_limit,min(angular_limit,2*da)) if abs(da)>=0.03 else 0.0
            self.tick(command)
        self.stop()
        print('manual reached',self.pose if pose_source is None else pose_source(),flush=True)

    def navigate(self, goal_pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y,a = goal_pose
        goal.pose.pose.orientation.z = math.sin(a/2)
        goal.pose.pose.orientation.w = math.cos(a/2)
        print('Nav2 target',goal_pose,flush=True)
        future = self.navigation.send_goal_async(goal)
        self.wait(future.done,5)
        self.goal = future.result()
        if not self.goal.accepted:
            raise RuntimeError('Nav2 rejected goal')
        result = self.goal.get_result_async()
        self.wait(result.done,40)
        status = result.result().status
        print('Nav2 result',status,'pose',self.pose,flush=True)
        if status!=4:
            raise RuntimeError('Nav2 failed: '+str(result.result().result))
        self.goal = None
        self.stop()

    def run(self):
        self.wait_ready()
        self.center = self.pose
        self.active = True
        self.wait(lambda:self.flags.get('base_motion_permitted'),10)
        # Verify denial on a dead test client before sending any nonzero command.
        end = time.monotonic()+0.8
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=0.05)
        if self.flags.get('base_motion_permitted'):
            raise RuntimeError('test lease did not expire')
        self.wait(lambda:self.flags.get('base_motion_permitted'),5)
        print('lease expiry verified; center',self.center,flush=True)
        x,y,a = self.center
        self.move((x+0.05*math.cos(a),y+0.05*math.sin(a),a))
        self.move(self.center)
        self.move((x-0.04*math.sin(a),y+0.04*math.cos(a),a))
        self.move(self.center)
        self.move((x,y,a+0.30))
        self.move((x,y,a-0.30))
        self.move(self.center)
        self.wait(lambda:self.navigation.server_is_ready(),10)
        self.wait(lambda:self.buffer.can_transform('map','base_footprint',Time()),10)
        tf = self.buffer.lookup_transform('map','base_footprint',Time()).transform
        mx,my,ma = tf.translation.x,tf.translation.y,yaw(tf.rotation)
        before = self.pose
        self.navigate((mx+0.16*math.cos(ma),my+0.16*math.sin(ma),ma))
        if math.dist(before[:2],self.pose[:2])<0.035:
            raise RuntimeError('Nav2 reported success without meaningful motion')
        self.navigate((mx,my,ma))
        self.move(self.center)
        end = time.monotonic()+5
        while time.monotonic()<end:
            self.tick(Twist())


def main(test_class=Test, output=OUTPUT, launch_arguments=(), production=False, payload_kg=None):
    if payload_kg is not None and (not math.isfinite(payload_kg) or payload_kg < 0):
        raise ValueError('reported payload must be a finite nonnegative mass')
    output.mkdir(parents=True,exist_ok=True)
    stack = None
    node = None
    error = None
    arguments = installed_stack_arguments()
    linear,angular = load_base_speed_limits(ROOT/'config/nav2_params.yaml')
    was_active = subprocess.run(['systemctl','is-active','--quiet','lekiwi-stack.service']).returncode==0
    if production and not was_active:
        raise RuntimeError('production verification requires the running managed stack')
    if not production:
        subprocess.run(['sudo','-n','/usr/bin/systemctl','stop','lekiwi-stack.service'],check=True)
    try:
        with (output/'stack.log').open('w') as log:
            if not production:
                stack = subprocess.Popen([str(ROOT/'scripts/ros-start.sh'), *arguments,
                    'bounded_base_test:=true',f'base_test_linear_limit:={linear}',
                    f'base_test_angular_limit:={angular}',*launch_arguments], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                    env={**os.environ,'LEKIWI_RUNTIME_DIR':str(output/'runtime')}, start_new_session=True)
            # Keep ROS alive through Python's interrupt cleanup so it can send
            # zero commands, withdraw the test lease and shut down Nav2 first.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = test_class()
            # Reporting can itself dispatch a failing sensor callback. Always
            # release clients/observer processes even when that cleanup fails.
            with ExitStack() as cleanup:
                cleanup.callback(rclpy.try_shutdown)
                cleanup.callback(node.destroy_node)
                cleanup.callback(node.navigation.destroy)
                cleanup.callback(node.listener.unregister)
                try:
                    node.run()
                except (Exception,KeyboardInterrupt) as e:
                    error = str(e) or type(e).__name__
                    raise
                finally:
                    if node.goal is not None:
                        future=node.goal.cancel_goal_async()
                        end=time.monotonic()+2
                        while not future.done() and time.monotonic()<end:
                            node.tick(Twist(),check=False)
                    node.active=False
                    node.stop()
                    graph = None
                    if node.map_client.wait_for_service(timeout_sec=1):
                        future=node.map_client.call_async(GetMap.Request(global_map=True,optimized=True,graph_only=False))
                        end=time.monotonic()+3
                        while not future.done() and time.monotonic()<end:
                            node.tick(Twist(),check=False)
                        if future.done() and future.result():
                            data=future.result().data
                            graph={'nodes':[{'id':m.id,'session':m.map_id,'features':len(m.word_kpts),
                                'valid_3d_features':sum(all(math.isfinite(v) for v in (p.x,p.y,p.z)) and
                                    math.hypot(p.x,p.y,p.z)>.01 for p in m.word_pts)} for m in data.nodes],
                                'links':[(link.from_id,link.to_id,link.type) for link in data.graph.links]}
                    report={'error':error,'origin':node.center,'final_pose':node.pose,'sensors':node.counts,'health':node.health,
                        'mode':'production' if production else 'bounded_test', 'reported_payload_kg':payload_kg,
                        'source_revision':subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
                        'collision_monitor_action':node.monitor_action,
                        'max_radius_m':max((math.dist(p[:2],node.center[:2]) for p in node.trace),default=0) if node.center else None,
                        'trace':node.trace,'slam':node.slam,'graph':graph,'health_faults':node.health_faults,
                        'fault_checks':getattr(node,'checks',None)}
                    if getattr(node,'config',{}).get('stage'):
                        report.update(origin_frame='raw_lidar_reference',final_pose=node.ranges[-1]['pose'] if node.ranges else None,
                            wheel_final_pose=node.pose,trace_frame='wheel_odometry',
                            max_radius_m=max(math.dist(r['pose'][:2],node.center[:2]) for r in node.ranges)
                                if node.ranges and node.center is not None else None)
                    (output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
                    print('report',output/'result.json',flush=True)
                    if not production and node.lifecycle_client.wait_for_service(timeout_sec=1):
                        future=node.lifecycle_client.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN))
                        end=time.monotonic()+8
                        while not future.done() and time.monotonic()<end:
                            node.tick(Twist(),check=False)
                        if not future.done() or not future.result().success:
                            print('Nav2 did not confirm graceful lifecycle shutdown',flush=True)
    finally:
        try:
            if stack is not None and stack.poll() is None:
                os.kill(stack.pid,signal.SIGINT)
                try:
                    stack.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    os.killpg(stack.pid,signal.SIGTERM)
                    try:
                        stack.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(stack.pid,signal.SIGKILL)
                        stack.wait(timeout=3)
        finally:
            if was_active and not production:
                subprocess.run(['sudo','-n','/usr/bin/systemctl','start','lekiwi-stack.service'],check=True)


if __name__=='__main__':
    main()
