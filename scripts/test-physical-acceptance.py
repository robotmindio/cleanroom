#!/usr/bin/env python3
"""Finite live sensor/telemetry fault checks; never approves braking acceptance.

Source setup.bash first. Requires the authorized clear 30 cm area and folded arm.
Uses the existing bounded stack runner and restores interrupted services on exit.
Wheel readings prove the fault response, not an independent stopping distance.
"""
import argparse
import math
from importlib import import_module
import os
import shlex
import subprocess
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from lekiwi_rmf.motion_guards import load_base_speed_limits

navigation = import_module('test-navigation')


class FaultTest(navigation.Test):
    def __init__(self, restart_tests=False, output=None, angular_test=False, telemetry_tests=False):
        super().__init__()
        self.deadline = time.monotonic()+360
        self.speed = math.inf
        self.checks = {}
        self.restart_tests = restart_tests
        self.output = output
        self.angular_test = angular_test
        self.telemetry_tests = telemetry_tests
        self.test_speed = load_base_speed_limits(navigation.ROOT/'config/nav2_params.yaml')[int(angular_test)]
        self.maximum_center_radius_m = .12
        self.phase = None
        self.safe_speed = None
        self.measured_speed = math.inf
        self.views = {}
        self.bridge = CvBridge()
        for camera in ('front','wrist','astra/color'):
            self.create_subscription(Image,'/camera/'+camera+'/image_raw',
                lambda m,c=camera:self.views.update({c:m}),navigation.qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel_safe',lambda m:setattr(self,'safe_speed',
            math.hypot(m.linear.x,m.linear.y)+abs(m.angular.z)),10)
        arguments = navigation.installed_stack_arguments()
        self.device = next(a.partition(':=')[2] for a in arguments if a.startswith('remote_ip:='))
        self.ssh = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',
                    '-o','ControlMaster=no','-o','ControlPath=none',self.device]

    def odometry(self, message):
        super().odometry(message)
        velocity = message.twist.twist
        linear_speed = math.hypot(velocity.linear.x,velocity.linear.y)
        self.measured_speed = abs(velocity.angular.z) if self.angular_test else linear_speed
        self.speed = linear_speed+abs(velocity.angular.z)
        if self.phase:
            stamp = message.header.stamp
            self.checks[self.phase]['samples'].append({
                'time':time.monotonic(),'wheel_speed':self.speed,'safe_command':self.safe_speed,
                'linear_speed':linear_speed,'angular_speed':velocity.angular.z,
                'capture_age_s':(self.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)/10**9,
                'base_permitted':self.flags.get('base_motion_permitted'),
                'faults':self.health.get('faults')})

    def tick(self, twist=None, check=True, pose_source=None):
        # Faults intentionally remove feedback/permission. Keep the finite
        # deadline and radius checks without aborting on the expected dropout.
        super().tick(twist,check=False)
        if check:
            if time.monotonic()>self.deadline:
                raise RuntimeError('fault-test deadline expired')
            pose = self.pose if pose_source is None else pose_source()
            if self.center and math.dist(pose[:2],self.center[:2])>=self.maximum_center_radius_m:
                raise RuntimeError('fault test reached its early center boundary')

    def action(self, argv, twist=None, data=None):
        environment = dict(os.environ)
        environment.setdefault('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')
        with subprocess.Popen(argv,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT,text=True,env=environment) as process:
            if data:
                process.stdin.write(data)
            process.stdin.close()
            end = time.monotonic()+20
            try:
                while process.poll() is None:
                    if time.monotonic()>end:
                        raise RuntimeError('fault action timed out: '+shlex.join(argv))
                    self.tick(twist,check=twist is not None and any((twist.linear.x,twist.linear.y,twist.angular.z)))
                output = process.stdout.read()
                if process.returncode:
                    raise RuntimeError('fault action failed: '+output)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=3)

    def remote(self, argv, twist=None, data=None):
        self.action([*self.ssh,shlex.join(argv)],twist,data)

    def fault(self, name, begin, restore, expected):
        self.wait(lambda:self.flags.get('base_motion_permitted') and self.flags.get('arm_stowed'),30)
        command = Twist()
        if self.angular_test:
            command.angular.z = self.test_speed
        else:
            command.linear.x = self.test_speed
        no_feedback = name in ('telemetry_loss','telemetry_replay_or_duplicate','compute_command_loss',
                              'host_restart_stops_then_gated_rearm')
        start = self.pose
        end = time.monotonic()+5
        while (self.measured_speed<self.test_speed*.9 or self.safe_speed is None or self.safe_speed<self.test_speed*.9 or
               (abs(navigation.angle(self.pose[2]-start[2]))<0.015 if self.angular_test else
                math.dist(self.pose[:2],start[:2])<0.003) or time.monotonic()-self.odom_at>0.3):
            if self.monitor_action and self.monitor_action[1]!=navigation.CollisionMonitorState.DO_NOTHING:
                action = self.monitor_action
                self.stop()
                raise RuntimeError('fault speed test blocked by collision monitor: '+str(action))
            if time.monotonic()>end:
                self.stop()
                units = 'rad/s' if self.angular_test else 'm/s'
                raise RuntimeError(f'{name} did not reach {self.test_speed} {units}; '
                                   f'wheel={self.measured_speed}, guarded={self.safe_speed}')
            self.tick(command)
        origin = self.pose
        self.phase = name
        self.checks[name] = {'passed':False,'samples':[],
                             'angular_test':self.angular_test,
                             'requested_speed':self.test_speed,'injection_speed':self.measured_speed}
        print('injecting',name,'moving speed',self.speed,flush=True)
        try:
            begin(command)
            end = time.monotonic()+5
            while not (expected in self.health.get('faults','') and
                       not self.flags.get('base_motion_permitted')):
                if time.monotonic()>end:
                    raise RuntimeError(name+' did not withdraw permission: '+str(self.health))
                self.tick(command)
            denial = self.health.copy()
            self.checks[name]['denial'] = denial
            if 'base_test:' in denial.get('faults',''):
                raise RuntimeError('test-client lease expired; fault result would be confounded')
            denied_at = time.monotonic()
            end = denied_at+3
            while not no_feedback and sum(
                s['wheel_speed']<=0.001 and s['capture_age_s']<0.3
                for s in self.checks[name]['samples'] if s['time']>=denied_at
            )<3:
                if time.monotonic()>end:
                    raise RuntimeError(name+' lacks fresh stopped wheel samples')
                self.tick(command)
            end = time.monotonic()+1
            while time.monotonic()<end:
                self.tick(command)
            # Sensor fault tests retain fresh wheel feedback. For telemetry loss
            # check the first recovered reading before sending another command.
            if not no_feedback and self.safe_speed!=0:
                raise RuntimeError(name+' left a nonzero guarded command')
            for camera,message in self.views.items():
                image = self.output/f'{name}-{camera.replace("/","-")}.jpg'
                if not cv2.imwrite(str(image),self.bridge.imgmsg_to_cv2(message,'bgr8')):
                    raise RuntimeError('could not save robot camera view: '+str(image))
        finally:
            self.stop()
            restore()
        self.wait(lambda:time.monotonic()-self.odom_at<0.3 and self.speed<=0.001,10)
        self.wait(lambda:self.flags.get('base_motion_permitted') and
                  self.flags.get('arm_stowed') and self.flags.get('driver')=='ARMED',45)
        # Permission recovery can dispatch newer wheel feedback. Recheck the
        # stopped state before recording it or starting the return movement.
        self.wait(lambda:time.monotonic()-self.odom_at<0.3 and self.speed<=0.001,10)
        self.checks[name].update({'passed':True,'denial':denial,'recovered_driver':self.flags['driver'],
                             'stopped_wheel_speed':self.speed,
                             'observed_displacement_m':math.dist(origin[:2],self.pose[:2])})
        self.phase = None
        print('passed',name,'stopped speed',self.speed,flush=True)
        self.move(self.center)

    def run(self):
        self.wait_ready()
        self.center = self.pose
        self.active = True
        if self.telemetry_tests:
            self.run_telemetry_faults()
            return
        if self.restart_tests:
            # Pause only our driver: the Pi's independent command watchdog must
            # stop the motors while compute cannot send its own stop command.
            pids = subprocess.check_output(['pgrep','-f',
                '/lib/lekiwi_rmf/lekiwi_driver '],text=True).split()
            if len(pids)!=1:
                raise RuntimeError('expected exactly one robot driver process')
            self.fault('compute_command_loss',
                lambda c:self.action(['kill','-STOP',pids[0]],c),
                lambda:self.action(['kill','-CONT',pids[0]],Twist()),'driver:')
            # The arm is still compact and supported in travel_stow before the
            # host restart releases torque. Observe loss before waiting for boot.
            self.fault('host_restart_stops_then_gated_rearm',
                lambda c:self.remote(['sudo','-n','systemctl','--no-block','restart',
                                     'lekiwi-host.service'],c),
                lambda:self.remote(['sudo','-n','systemctl','start','lekiwi-host.service'],Twist()),
                'driver:')
            return
        for name,unit,reason in [('scan_disconnect','lekiwi-lidar.service','scan:'),
                                 ('depth_disconnect','lekiwi-astra.service','depth:')]:
            self.fault(name,
                lambda command,u=unit:self.remote(['sudo','-n','systemctl','stop',u],command),
                lambda u=unit:self.remote(['sudo','-n','systemctl','start',u],Twist()),reason)
        table = 'lekiwi_acceptance_test'
        def block(command):
            # A native timer removes this specific test table even if this
            # process dies; SSH and the command/safety ports remain available.
            self.remote(['sudo','-n','systemd-run','--quiet',
                '--collect',
                '--unit=lekiwi-acceptance-telemetry-cleanup','--on-active=15s',
                '/usr/sbin/nft','delete','table','inet',table],command)
            rules = (f'add table inet {table}\n'
                     f'add chain inet {table} output {{ type filter hook output priority 0; policy accept; }}\n'
                     f'add rule inet {table} output tcp sport 5556 drop\n')
            self.remote(['sudo','-n','/usr/sbin/nft','-f','-'],command,rules)
        def unblock():
            self.remote(['sudo','-n','/usr/sbin/nft','delete','table','inet',table],Twist())
            self.remote(['sudo','-n','systemctl','stop','lekiwi-acceptance-telemetry-cleanup.timer'],Twist())
        self.fault('telemetry_loss',block,unblock,'driver:')
        self.stop()

    def run_telemetry_faults(self):
        # Clone the native service's environment without printing credentials.
        # A finite test unit and independent rollback restore the normal host.
        setup = '''import subprocess
def run(*a): return subprocess.check_output(a,text=True).strip()
env=run('systemctl','show','--value','-p','Environment','lekiwi-host.service')
directory=run('systemctl','show','--value','-p','WorkingDirectory','lekiwi-host.service')
user=run('systemctl','show','--value','-p','User','lekiwi-host.service')
run('systemd-run','--quiet','--collect','--unit=lekiwi-qualification-restore','--on-active=120s',
    '/bin/sh','-c','systemctl stop lekiwi-qualification-host.service; systemctl start lekiwi-host.service')
run('systemctl','stop','lekiwi-host.service')
run('systemd-run','--quiet','--collect','--unit=lekiwi-qualification-host',
    '--property=RuntimeMaxSec=100','--property=User='+user,
    '--property=WorkingDirectory='+directory,'--property=Environment='+env,
    directory+'/scripts/robot-host.sh','--telemetry-fault-test')
'''
        try:
            self.remote(['sudo','-n','/usr/bin/python3','-'],Twist(),setup)
            self.wait(lambda:self.flags.get('base_motion_permitted') and
                      self.flags.get('driver')=='ARMED',45)
            for name,signum,reason in [('motor_diagnostic_fault','USR1','motor_health:'),
                    ('telemetry_replay_or_duplicate','USR2','driver:')]:
                def inject(command,signum=signum):
                    self.remote(['/bin/bash','-c',
                        'mapfile -t pids < <(pgrep -f "[p]ython scripts/test-host-telemetry.py"); '
                        '[[ ${#pids[@]} == 1 ]] && kill -'+signum+' "${pids[0]}"'],command)
                self.fault(name,inject,lambda:None,reason)
        finally:
            try:
                self.remote(['sudo','-n','systemctl','stop','lekiwi-qualification-host.service'],Twist())
            finally:
                try:
                    self.remote(['sudo','-n','systemctl','start','lekiwi-host.service'],Twist())
                finally:
                    self.remote(['sudo','-n','systemctl','stop','lekiwi-qualification-restore.timer'],Twist())


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--restart-tests',action='store_true',
                        help='test compute-driver suspension and the Pi motor-host restart')
    parser.add_argument('--angular',action='store_true',help='inject faults during rotation instead of translation')
    parser.add_argument('--telemetry-fault-tests',action='store_true',
                        help='use the finite qualification host for motor diagnostics and duplicate telemetry')
    args = parser.parse_args()
    output = navigation.ROOT/'.benchmarks/physical-faults'/time.strftime('%Y%m%d-%H%M%S')
    navigation.main(lambda:FaultTest(args.restart_tests,output,args.angular,args.telemetry_fault_tests),output)
