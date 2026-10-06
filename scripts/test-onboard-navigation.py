#!/usr/bin/env python3
"""Finite unloaded navigation checks using the robot cameras and a temporary map.

Source scripts/setup.bash first. Requires the attended clear 30 cm area and
travel_stow. Restores the production service and preserves its map/acceptance.
"""
import json
import math
from pathlib import Path
from importlib import import_module
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import OdomInfo, RGBDImages
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[1]
NAV = import_module('test-navigation')
OUTPUT = ROOT / '.benchmarks/onboard-navigation' / time.strftime('%Y%m%d-%H%M%S')


class OnboardTest(NAV.Test):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.views, self.visual, self.visual_info, self.steps, self.arrays = {}, [], [], [], []
        self.create_subscription(Odometry, '/verification/visual_odometry',
                                 self.visual_odometry, qos_profile_sensor_data)
        self.create_subscription(OdomInfo, '/verification/visual_info',
            lambda m: self.visual_info.append({'lost': m.lost, 'inliers': m.inliers}),
            qos_profile_sensor_data)
        self.create_subscription(RGBDImages, '/slam/rgbd_images',
            lambda m: self.arrays.append(len(m.rgbd_images)), qos_profile_sensor_data)
        for camera in ('front', 'wrist', 'astra/color'):
            self.create_subscription(Image, f'/camera/{camera}/image_raw',
                lambda m, c=camera: self.views.update({c: m}), qos_profile_sensor_data)
        self.log = (OUTPUT / 'visual-odometry.log').open('w')
        binary = Path(get_package_prefix('rtabmap_odom')) / 'lib/rtabmap_odom/rgbd_odometry'
        self.observer = subprocess.Popen([
            str(binary), '--ros-args', '-r', '__node:=onboard_visual_verification',
            '-r', 'rgbd_image:=/slam/astra/rgbd_image',
            '-r', 'odom:=/verification/visual_odometry',
            '-r', 'odom_info:=/verification/visual_info',
            '-p', 'subscribe_rgbd:=true', '-p', 'frame_id:=base_footprint',
            '-p', 'odom_frame_id:=verification_odom', '-p', 'publish_tf:=false',
            '-p', 'qos:=1', '-p', 'Odom/ResetCountdown:="0"',
        ], stdout=self.log, stderr=subprocess.STDOUT)

    def visual_odometry(self, message):
        pose = message.pose.pose
        stamp = message.header.stamp
        age = (self.get_clock().now().nanoseconds - stamp.sec*10**9 - stamp.nanosec)/1e9
        self.visual.append({'t': time.monotonic(), 'source_age_s': age,
                            'x': pose.position.x, 'y': pose.position.y,
                            'yaw': NAV.yaw(pose.orientation)})

    def check_visual_feedback(self, now):
        if not self.visual:
            raise RuntimeError('independent Astra pose unavailable')
        if not self.visual_info or self.visual_info[-1]['lost']:
            raise RuntimeError('independent Astra tracking unavailable or lost')
        sample = self.visual[-1]
        if not all(math.isfinite(sample[k]) for k in ('t', 'source_age_s', 'x', 'y', 'yaw')):
            raise RuntimeError('invalid independent Astra pose')
        age = sample['source_age_s'] + now - sample['t']
        if not -.2 <= age <= 2.:
            raise RuntimeError('independent Astra pose is stale or from a future clock')
        if math.hypot(sample['x'], sample['y']) > .16:
            raise RuntimeError('early 16 cm independent visual boundary reached')

    def check_feedback(self):
        super().check_feedback()
        def fresh():
            return (self.visual and self.visual_info and not self.visual_info[-1]['lost']
                    and -.2 <= self.visual[-1]['source_age_s']
                    + time.monotonic() - self.visual[-1]['t'] <= 2.)
        if not fresh():
            self.pause_until(fresh, 'fresh independent Astra tracking')
        self.check_visual_feedback(time.monotonic())

    def mark(self, name):
        end = time.monotonic() + 1
        while time.monotonic() < end:
            self.tick()
        self.steps.append({'name': name, 'wheel': self.pose,
                           'visual': self.visual[-1], 'flags': self.flags.copy()})
        print('step', name, self.pose, self.visual[-1], flush=True)
        for camera, image in self.views.items():
            path = OUTPUT / f'{len(self.steps)}-{camera.replace("/", "-")}.jpg'
            if not cv2.imwrite(str(path), self.bridge.imgmsg_to_cv2(image, 'bgr8')):
                raise RuntimeError(f'could not save onboard image: {path}')

    def run(self):
        self.wait_ready()
        self.wait(lambda: len(self.visual) > 3, 15)
        self.center = self.pose
        self.deadline = time.monotonic() + 170
        self.active = True
        self.wait(lambda: self.flags.get('base_motion_permitted'), 10)
        self.wait(self.navigation.server_is_ready, 20)
        self.wait(lambda: self.buffer.can_transform('map', 'base_footprint', NAV.Time()), 15)
        transform = self.buffer.lookup_transform('map', 'base_footprint', NAV.Time()).transform
        x, y, a = transform.translation.x, transform.translation.y, NAV.yaw(transform.rotation)
        self.mark('origin')
        for name, goal, minimum in (
            ('forward', (x+.10*math.cos(a), y+.10*math.sin(a), a), .025),
            ('forward return', (x, y, a), 0.),
            ('lateral', (x-.06*math.sin(a), y+.06*math.cos(a), a), .015),
            ('lateral return', (x, y, a), 0.),
        ):
            before = self.visual[-1]
            self.navigate(goal)
            self.mark(name)
            distance = math.hypot(self.visual[-1]['x']-before['x'],
                                  self.visual[-1]['y']-before['y'])
            if distance < minimum:
                raise RuntimeError(f'Nav2 success without independent visual movement: {name}')
        cx, cy, ca = self.center
        self.move((cx, cy, ca+.30), angular_limit=.06)
        self.mark('left rotation')
        self.move((cx, cy, ca-.30), angular_limit=.06)
        self.mark('right rotation')
        self.move(self.center, angular_limit=.06)
        self.mark('final return')

    def destroy_node(self):
        try:
            report = {'visual': self.visual, 'visual_info': self.visual_info,
                      'steps': self.steps, 'dual_view_arrays': len(self.arrays),
                      'dual_view_sizes': sorted(set(self.arrays))}
            (OUTPUT / 'onboard-result.json').write_text(json.dumps(report, indent=2)+'\n')
        finally:
            if self.observer.poll() is None:
                self.observer.send_signal(signal.SIGINT)
                try:
                    self.observer.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    self.observer.kill()
                    self.observer.wait()
            self.log.close()
            super().destroy_node()


if __name__ == '__main__':
    NAV.main(test_class=OnboardTest, output=OUTPUT,
                launch_arguments=(f'rtabmap_database:={OUTPUT}/test-map.db',))
