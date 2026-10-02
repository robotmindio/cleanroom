#!/usr/bin/env python3
"""Finite live sensor/telemetry fault checks; never approves braking acceptance.

Source setup.bash first. Requires the authorized clear 30 cm area and folded arm.
Uses the existing bounded stack runner and restores interrupted services on exit.
Wheel readings prove the fault response, not an independent stopping distance.
"""
import math
from pathlib import Path
import runpy
import shlex
import subprocess
import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState

navigation = runpy.run_path(str(Path(__file__).with_name('test-navigation.py')))


class FaultTest(navigation['Test']):
    def __init__(self):
        super().__init__()
        self.deadline = time.monotonic()+360
        self.speed = math.inf
        self.checks = {}
        self.phase = None
        self.safe_speed = None
        self.linear_speed = math.inf
        self.monitor_state = self.create_client(GetState,'/collision_monitor/get_state')
        self.create_subscription(Twist,'/cmd_vel_safe',lambda m:setattr(self,'safe_speed',
            math.hypot(m.linear.x,m.linear.y)+abs(m.angular.z)),10)
        arguments = navigation['installed_stack_arguments']()
        self.device = next(a.partition(':=')[2] for a in arguments if a.startswith('remote_ip:='))
        self.ssh = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',
                    '-o','ControlMaster=no','-o','ControlPath=none',self.device]

    def odometry(self, message):
        super().odometry(message)
        velocity = message.twist.twist
        self.linear_speed = math.hypot(velocity.linear.x,velocity.linear.y)
        self.speed = self.linear_speed+abs(velocity.angular.z)
        if self.phase:
            stamp = message.header.stamp
            self.checks[self.phase]['samples'].append({
                'time':time.monotonic(),'wheel_speed':self.speed,'safe_command':self.safe_speed,
                'linear_speed':self.linear_speed,'angular_speed':velocity.angular.z,
                'capture_age_s':(self.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)/10**9,
                'base_permitted':self.flags.get('base_motion_permitted'),
                'faults':self.health.get('faults')})

    def tick(self, twist=None, check=True):
        # Faults intentionally remove feedback/permission. Keep the finite
        # deadline and radius checks without aborting on the expected dropout.
        super().tick(twist,check=False)
        if check:
            if time.monotonic()>self.deadline:raise RuntimeError('fault-test deadline expired')
            if self.center and math.dist(self.pose[:2],self.center[:2])>=0.12:
                raise RuntimeError('fault test reached its early 12 cm boundary')

    def action(self, argv, twist=None, data=None):
        with subprocess.Popen(argv,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT,text=True) as process:
            if data:process.stdin.write(data)
            process.stdin.close()
            end = time.monotonic()+20
            try:
                while process.poll() is None:
                    if time.monotonic()>end:raise RuntimeError('fault action timed out: '+shlex.join(argv))
                    self.tick(twist,check=twist is not None and twist.linear.x!=0)
                output = process.stdout.read()
                if process.returncode:raise RuntimeError('fault action failed: '+output)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=3)

    def remote(self, argv, twist=None, data=None):
        self.action([*self.ssh,shlex.join(argv)],twist,data)

    def fault(self, name, begin, restore, expected):
        self.wait(lambda:self.flags.get('base_motion_permitted') and self.flags.get('arm_stowed'),30)
        command = Twist()
        command.linear.x = 0.03
        start = self.pose
        end = time.monotonic()+5
        while (self.linear_speed<0.005 or not self.safe_speed or
               math.dist(self.pose[:2],start[:2])<0.003 or time.monotonic()-self.odom_at>0.3):
            if time.monotonic()>end:raise RuntimeError('base did not start for '+name)
            self.tick(command)
        origin = self.pose
        self.phase = name
        self.checks[name] = {'passed':False,'samples':[]}
        print('injecting',name,'moving speed',self.speed,flush=True)
        try:
            begin(command)
            end = time.monotonic()+5
            while not (expected in self.health.get('faults','') and
                       not self.flags.get('base_motion_permitted')):
                if time.monotonic()>end:raise RuntimeError(name+' did not withdraw permission: '+str(self.health))
                self.tick(command)
            denial = self.health.copy()
            self.checks[name]['denial'] = denial
            if 'base_test:' in denial.get('faults',''):
                raise RuntimeError('test-client lease expired; fault result would be confounded')
            denied_at = time.monotonic()
            end = denied_at+3
            while name!='telemetry_loss' and sum(
                s['wheel_speed']<=0.001 and s['capture_age_s']<0.3
                for s in self.checks[name]['samples'] if s['time']>=denied_at
            )<3:
                if time.monotonic()>end:raise RuntimeError(name+' lacks fresh stopped wheel samples')
                self.tick(command)
            end = time.monotonic()+1
            while time.monotonic()<end:self.tick(command)
            # Sensor fault tests retain fresh wheel feedback. For telemetry loss
            # check the first recovered reading before sending another command.
            if name!='telemetry_loss' and self.safe_speed!=0:
                raise RuntimeError(name+' left a nonzero guarded command')
            for frame in range(2):
                image = navigation['ROOT']/f'.benchmarks/physical-acceptance/{name}-{frame}.jpg'
                self.action(['timeout','12','gst-launch-1.0','-q','pipewiresrc',
                    'target-object=v4l2_input.pci-0000_04_00.3-usb-0_2.1.2_1.0','num-buffers=1','!',
                    'image/jpeg,width=1280,height=720,framerate=30/1','!',
                    'filesink',f'location={image}'],command)
                end = time.monotonic()+0.7
                while time.monotonic()<end:self.tick(command)
        finally:
            self.stop()
            restore()
        self.wait(lambda:time.monotonic()-self.odom_at<0.3 and self.speed<=0.001,10)
        self.wait(lambda:self.flags.get('base_motion_permitted') and
                  self.flags.get('arm_stowed') and self.flags.get('driver')=='ARMED',45)
        self.checks[name].update({'passed':True,'denial':denial,'recovered_driver':self.flags['driver'],
                             'stopped_wheel_speed':self.speed,
                             'observed_displacement_m':math.dist(origin[:2],self.pose[:2])})
        self.phase = None
        print('passed',name,'stopped speed',self.speed,flush=True)
        self.move(self.center)

    def run(self):
        self.wait(lambda:self.pose is not None and self.flags.get('arm_stowed') and
                  self.flags.get('driver')=='ARMED',70)
        self.wait(lambda:self.monitor_state.service_is_ready(),20)
        end = time.monotonic()+30
        while True:
            future = self.monitor_state.call_async(GetState.Request())
            self.wait(future.done,5)
            if future.result().current_state.id==3:break
            if time.monotonic()>end:raise RuntimeError('collision monitor did not activate')
            self.tick(Twist())
        self.center = self.pose
        self.active = True
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


if __name__=='__main__':
    navigation['main'](FaultTest,navigation['ROOT']/'.benchmarks/physical-acceptance')
