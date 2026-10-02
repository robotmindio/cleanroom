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

navigation = runpy.run_path(str(Path(__file__).with_name('test-navigation.py')))


class FaultTest(navigation['Test']):
    def __init__(self):
        super().__init__()
        self.deadline = time.monotonic()+360
        self.speed = math.inf
        self.checks = {}
        arguments = navigation['installed_stack_arguments']()
        self.device = next(a.partition(':=')[2] for a in arguments if a.startswith('remote_ip:='))
        self.ssh = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',
                    '-o','ControlMaster=no','-o','ControlPath=none',self.device]

    def odometry(self, message):
        super().odometry(message)
        velocity = message.twist.twist
        self.speed = math.hypot(velocity.linear.x,velocity.linear.y)+abs(velocity.angular.z)

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
        command.linear.x = 0.02
        end = time.monotonic()+2
        while self.speed<=0.001 or time.monotonic()-self.odom_at>0.3:
            if time.monotonic()>end:raise RuntimeError('base did not start for '+name)
            self.tick(command)
        origin = self.pose
        print('injecting',name,'moving speed',self.speed,flush=True)
        try:
            begin(command)
            end = time.monotonic()+5
            while not (expected in self.health.get('faults','') and
                       not self.flags.get('base_motion_permitted')):
                if time.monotonic()>end:raise RuntimeError(name+' did not withdraw permission: '+str(self.health))
                self.tick(command)
            denial = self.health.copy()
            if 'base_test:' in denial.get('faults',''):
                raise RuntimeError('test-client lease expired; fault result would be confounded')
            end = time.monotonic()+1
            while time.monotonic()<end:self.tick(command)
            # Sensor fault tests retain fresh wheel feedback. For telemetry loss
            # check the first recovered reading before sending another command.
            if name!='telemetry_loss' and (
                self.speed>0.001 or time.monotonic()-self.odom_at>0.4
            ):raise RuntimeError(name+' did not produce fresh stopped wheel readings')
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
        self.checks[name] = {'passed':True,'denial':denial,'recovered_driver':self.flags['driver'],
                             'stopped_wheel_speed':self.speed,
                             'observed_displacement_m':math.dist(origin[:2],self.pose[:2])}
        print('passed',name,self.checks[name],flush=True)
        self.move(self.center)

    def run(self):
        self.wait(lambda:self.pose is not None and self.flags.get('arm_stowed') and
                  self.flags.get('driver')=='ARMED',70)
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
