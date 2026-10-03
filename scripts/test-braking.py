#!/usr/bin/env python3
"""Finite optical braking characterization, followed by production restoration.

The tracked profile is the test specification. Raw camera frames, receive times,
ROS feedback and every attempted trial are retained. This runner never changes
validated:false or claims independent camera/floor calibration.
"""
import argparse
import json
import math
from pathlib import Path
import runpy
import time

import cv2
import numpy as np
import yaml
from geometry_msgs.msg import Twist

navigation = runpy.run_path(str(Path(__file__).with_name('test-navigation.py')))
ROOT = navigation['ROOT']
DIRECTIONS = {'forward':(1,0,0), 'reverse':(-1,0,0), 'left':(0,1,0), 'right':(0,-1,0),
              'rotation_cw':(0,0,-1), 'rotation_ccw':(0,0,1)}


def metric_reference(markers, identifiers, side):
    if not math.isfinite(side) or side <= 0 or len(identifiers) < 2:
        raise ValueError('invalid marker dimensions or reference identifiers')
    corners = [np.asarray(markers[key], dtype=float) for key in identifiers]
    u = np.mean([(c[1]-c[0]+c[2]-c[3])/2 for c in corners], axis=0)
    v = np.mean([(c[3]-c[0]+c[2]-c[1])/2 for c in corners], axis=0)
    matrix = side*np.linalg.inv(np.column_stack((u, v)))
    origin = corners[0].mean(axis=0)
    reference = {key: (np.asarray(markers[key])-origin)@matrix.T for key in identifiers}
    error = max(abs(np.linalg.norm(c-np.roll(c, -1, axis=0), axis=1)-side).max()
                for c in reference.values())
    if error > side*.15:
        raise ValueError(f'marker scale is inconsistent: {error:.4f} m edge residual')
    return matrix, origin, reference, float(error)


def metric_pose(markers, matrix, origin, reference):
    common = sorted(set(markers).intersection(reference))
    if len(common) < 2:
        return None
    before = np.concatenate([reference[key] for key in common])
    after = np.concatenate([(np.asarray(markers[key])-origin)@matrix.T for key in common])
    a, b = before.mean(axis=0), after.mean(axis=0)
    u, _, vt = np.linalg.svd((before-a).T@(after-b))
    rotation = vt.T@u.T
    if np.linalg.det(rotation) < 0:
        return None
    translation = b-rotation@a
    error = float(np.max(np.linalg.norm(before@rotation.T+translation-after, axis=1)))
    if error > .004:
        return None
    return [float(translation[0]), float(translation[1]),
            math.atan2(rotation[1, 0], rotation[0, 0])], error, common


def maximum_swept_excursion(poses, radius):
    """Bound any body's point displacement from the pre-stop pose.

    The farthest excursion consumes obstacle clearance. Summing frame-to-frame
    travel instead accumulates stationary detector jitter as braking distance.
    """
    start = poses[0]
    return max(math.dist(start[:2], pose[:2]) +
               2*radius*abs(math.sin(navigation['angle'](pose[2]-start[2])/2))
               for pose in poses)


def observed_speeds(samples,angular):
    rates = []
    for a,b in zip(samples,samples[1:]):
        if not 0<=a['pts_ns']<b['pts_ns']<2**64-1:
            raise RuntimeError('camera capture timestamps are invalid')
        dt = (b['pts_ns']-a['pts_ns'])/1e9
        if .005<dt<=.20:
            distance = (abs(navigation['angle'](b['pose'][2]-a['pose'][2])) if angular
                        else math.dist(b['pose'][:2],a['pose'][:2]))
            rates.append(distance/dt)
    return rates


class Camera:
    def __init__(self, config, output):
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
        Gst.init(None)
        self.Gst = Gst
        width, height = config['image_size']
        # Restrict the camera selector before constructing a GStreamer pipeline.
        name = config['camera']
        if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-' for c in name):
            raise ValueError('invalid camera source name')
        self.pipeline = Gst.parse_launch(
            f'pipewiresrc target-object={name} ! image/jpeg,width={width},height={height},framerate=30/1 '
            '! appsink name=frames max-buffers=1 drop=true sync=false')
        self.sink = self.pipeline.get_by_name('frames')
        self.video = (output/'camera.mjpg').open('wb')
        self.records = (output/'camera.jsonl').open('w')
        self.config = config
        self.output = output
        self.calibration = None
        self.last_pose = None
        self.last_time = 0.0
        self.closed = False
        self.count = 0
        cv2.setNumThreads(2)
        self.detector = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco,'DetectorParameters_create')
                           else cv2.aruco.DetectorParameters())
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.parameters.aprilTagQuadDecimate = 2.0
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            message = self.pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
            details = message.parse_error() if message is not None else 'no GStreamer error detail'
            self.close()
            raise RuntimeError(f'measurement camera pipeline failed to start: {details}')

    def sample(self):
        if self.closed:
            return None
        sample = self.sink.emit('try-pull-sample', self.Gst.MSECOND)
        if sample is None:
            message = self.pipeline.get_bus().pop_filtered(self.Gst.MessageType.ERROR)
            if message is not None:
                error, detail = message.parse_error()
                raise RuntimeError(f'measurement camera failed: {error}; {detail}')
            return None
        buffer = sample.get_buffer()
        ok, mapped = buffer.map(self.Gst.MapFlags.READ)
        if not ok:
            raise RuntimeError('cannot read measurement camera buffer')
        try:
            raw = bytes(mapped.data)
        finally:
            buffer.unmap(mapped)
        received = time.monotonic()
        self.video.write(raw)
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE) if raw else None
        if image is None:
            self.count += 1
            record = {'time':received, 'pts_ns':int(buffer.pts), 'pose':None,
                      'ids':[], 'error':'invalid JPEG', 'bytes':len(raw)}
            self.records.write(json.dumps(record)+'\n')
            return record
        corners, ids, _ = cv2.aruco.detectMarkers(image, self.detector, parameters=self.parameters)
        markers = {} if ids is None else {int(key):c.reshape(4, 2) for key, c in zip(ids.flatten(), corners)}
        if self.calibration is None and all(key in markers for key in self.config['marker_ids']):
            self.calibration = metric_reference(markers, self.config['marker_ids'], self.config['marker_side_m'])
            matrix,origin,reference,error = self.calibration
            (self.output/'calibration.json').write_text(json.dumps({
                'matrix':matrix.tolist(),'origin_px':origin.tolist(),
                'reference':{key:value.tolist() for key,value in reference.items()},
                'marker_edge_residual_m':error,'opencv':cv2.__version__},indent=2)+'\n')
        pose = None if self.calibration is None else metric_pose(markers, *self.calibration[:3])
        if pose is not None:
            self.last_pose = pose[0]
            self.last_time = received
        self.count += 1
        record = {'time':received, 'pts_ns':int(buffer.pts), 'pose':None if pose is None else pose[0],
                  'fit_error_m':None if pose is None else pose[1], 'ids':list(markers)}
        self.records.write(json.dumps(record)+'\n')
        return record

    def close(self):
        if not self.closed:
            self.pipeline.set_state(self.Gst.State.NULL)
            self.video.close()
            self.records.close()
            self.closed = True


class BrakingTest(navigation['Test']):
    def __init__(self, config, output, camera):
        super().__init__()
        self.config = config
        self.output = output
        self.deadline = time.monotonic()+config['maximum_runtime_s']
        self.camera = camera
        self.optical_samples = []
        self.feedback = (output/'feedback.jsonl').open('w')
        self.checks = {'trials':[], 'navigation':[], 'measurement_scope':'marker-plane characterization',
                       'floor_plane_alignment_verified':False, 'physical_acceptance_granted':False}
        self.latest_safe = None
        self.create_subscription(Twist, '/cmd_vel_safe', self.safe_command, 10)

    def safe_command(self, message):
        self.latest_safe = [message.linear.x, message.linear.y, message.angular.z]

    def move(self, target):
        super().move(target, linear_limit=.10, angular_limit=.40)

    def tick(self, twist=None, check=True):
        super().tick(twist, check)
        sample = self.camera.sample()
        if sample is not None and sample['pose'] is not None:
            self.optical_samples.append(sample)
        if twist is not None:
            self.feedback.write(json.dumps({'time':time.monotonic(), 'command':[twist.linear.x,twist.linear.y,twist.angular.z],
                'safe_command':self.latest_safe, 'wheel_pose':self.pose, 'health':self.health,
                'collision_monitor_action':self.monitor_action})+'\n')
        if check and self.active:
            if time.monotonic()-self.camera.last_time > self.config['maximum_frame_age_s']:
                self.pause_until(lambda:time.monotonic()-self.camera.last_time <= self.config['maximum_frame_age_s'],
                                 'fresh optical tracking')
            x, y, a = self.camera.last_pose
            distance = math.hypot(x, y)
            # A chassis point can sweep around an offset reference during rotation.
            center_bound = distance+2*self.config['marker_center_offset_bound_m']*abs(math.sin(a/2))
            if distance >= self.config['maximum_optical_displacement_m'] or center_bound >= self.config['maximum_center_radius_m']:
                self.command.publish(Twist())
                raise RuntimeError('optical test boundary reached; zero command sent')

    def settle(self):
        start = time.monotonic()
        while time.monotonic()-start < 1.0:
            self.tick(Twist())
        samples = [s for s in self.optical_samples if s['time'] >= start]
        if len(samples) < 8:
            raise RuntimeError('insufficient camera samples to verify stopping')
        return samples

    def trial(self, direction, speed):
        self.move(self.center)
        self.wait(lambda:self.flags.get('base_motion_permitted'), 10)
        before = self.camera.last_pose.copy()
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = (speed*v for v in DIRECTIONS[direction])
        beginning = len(self.optical_samples)
        pauses = self.motion_pauses
        paused_seconds = self.paused_seconds
        faults = len(self.health_faults)
        timeout = time.monotonic()+5
        angular = direction.startswith('rotation')
        while True:
            self.tick(command)
            x, y, a = self.camera.last_pose
            distance = abs(navigation['angle'](a-before[2])) if angular else math.dist((x,y), before[:2])
            if distance >= self.config['trial_rotation_rad' if angular else 'trial_displacement_m']:
                break
            if time.monotonic() > timeout+self.paused_seconds-paused_seconds:
                raise RuntimeError(f'optical motion pulse incomplete; collision monitor={self.monitor_action}')
        moving = self.optical_samples[beginning:]
        if len(moving) < 3:
            raise RuntimeError('too few optical frames to measure the attained speed')
        receive_rates = []
        for a, b in zip(moving, moving[1:]):
            dt = b['time']-a['time']
            if dt > .005:
                distance = abs(navigation['angle'](b['pose'][2]-a['pose'][2])) if angular else math.dist(b['pose'][:2], a['pose'][:2])
                receive_rates.append(distance/dt)
        rates = observed_speeds(moving,angular)
        if not rates or float(np.median(rates)) < (.015 if angular else .001):
            raise RuntimeError('requested motion was not independently observed')
        # The preceding image arrived before t0; its exposure is earlier still.
        # Including that entire remaining path conservatively includes camera delay.
        baseline = self.optical_samples[-1]
        stopped_at = time.monotonic()
        self.command.publish(Twist())
        post = [baseline, *self.settle()]
        path = sum(math.dist(a['pose'][:2], b['pose'][:2]) for a,b in zip(post,post[1:]))
        rotation = sum(abs(navigation['angle'](b['pose'][2]-a['pose'][2])) for a,b in zip(post,post[1:]))
        # ponytail: conservative swept-point bound; surveyed marker-to-center
        # extrinsics can replace the 20 cm offset allowance after calibration.
        swept = maximum_swept_excursion([s['pose'] for s in post],
                                       .33+self.config['marker_center_offset_bound_m'])
        final = post[-1]['pose']
        outside = [i for i,s in enumerate(post) if math.dist(s['pose'][:2], final[:2]) > .002
                   or abs(navigation['angle'](s['pose'][2]-final[2])) > .01]
        stable_start = post[min((max(outside)+1 if outside else 1),len(post)-1)]
        stop_time = max(0.0, stable_start['time']-stopped_at)
        result = {'direction':direction,'requested_speed':speed,'median_observed_speed':float(np.median(rates)),
                  'speed_time_source':'camera_capture_pts',
                  'receive_clock_median_speed':float(np.median(receive_rates)) if receive_rates else None,
                  'maximum_observed_speed':float(max(rates)),'stop_command_time':stopped_at,
                  'requested_speed_covered':float(np.median(rates[-3:])) >= speed*.9,
                  'marker_path_after_pre_stop_frame_m':path,'residual_rotation_rad':rotation,
                  'conservative_swept_distance_m':swept,'stop_time_receive_upper_s':stop_time,
                  'camera_frames':len(moving)+len(post),'measurement_uncertainty_m':self.config['measurement_uncertainty_m']}
        result['within_budget'] = (swept+self.config['measurement_uncertainty_m'] <= self.config['maximum_stopping_distance_m']
                                   and stop_time <= self.config['maximum_stop_time_s'])
        result['feedback_interrupted'] = self.motion_pauses>pauses or len(self.health_faults)>faults
        result['qualification_eligible'] = (result['within_budget'] and result['requested_speed_covered']
                                            and not result['feedback_interrupted'])
        self.checks['trials'].append(result)
        (self.output/'measurements.json').write_text(json.dumps(self.checks,indent=2)+'\n')
        print(json.dumps(result), flush=True)
        self.move(self.center)
        return result

    def run(self):
        try:
            self.wait(lambda:self.camera.calibration is not None and self.camera.last_pose is not None, 10)
            self.wait_ready()
            self.center = self.pose
            self.active = True
            self.wait(lambda:self.flags.get('base_motion_permitted'), 15)
            self.settle()
            self.checks['marker_edge_residual_m'] = self.camera.calibration[3]
            candidates = []
            for directions, steps in [(['forward','reverse','left','right'],self.config['linear_steps_m_s']),
                                      (['rotation_cw','rotation_ccw'],self.config['angular_steps_rad_s'])]:
                directions = [d for d in directions if d in self.config.get('directions',DIRECTIONS)]
                if not directions:
                    continue
                accepted = None
                for speed in steps:
                    results = []
                    for direction in directions:
                        result = self.trial(direction,speed)
                        while result['feedback_interrupted']:
                            print('retrying interrupted exploration',direction,speed,flush=True)
                            result = self.trial(direction,speed)
                        results.append(result)
                        if not result['within_budget']:
                            break
                    if len(results)!=len(directions) or not all(result['within_budget'] for result in results):
                        break
                    accepted = speed
                if accepted is not None:
                    candidates.append((directions,accepted))
            for directions,speed in candidates:
                for direction in directions:
                    completed = 0
                    while completed<self.config['trials_per_direction']:
                        result = self.trial(direction,speed)
                        if result['feedback_interrupted']:
                            print('retrying interrupted stop',direction,speed,flush=True)
                            continue
                        if not result['within_budget']:
                            raise RuntimeError('repeated stopping trial exceeded the predeclared budget')
                        if not result['qualification_eligible']:
                            print('retrying unattained speed',direction,speed,flush=True)
                            continue
                        completed += 1
            self.move(self.center)
            self.wait(lambda:self.navigation.server_is_ready(), 10)
            self.wait(lambda:self.buffer.can_transform('map','base_footprint',navigation['Time']()), 10)
            tf = self.buffer.lookup_transform('map','base_footprint',navigation['Time']()).transform
            x, y, a = tf.translation.x, tf.translation.y, navigation['yaw'](tf.rotation)
            for target in [(x+.04*math.cos(a),y+.04*math.sin(a),a),(x,y,a)]:
                self.navigate(target)
                self.checks['navigation'].append({'target':target,'optical_pose':self.camera.last_pose.copy(),'wheel_pose':self.pose})
            (self.output/'measurements.json').write_text(json.dumps(self.checks,indent=2)+'\n')
        finally:
            self.active = False
            self.stop()
            (self.output/'measurements.json').write_text(json.dumps(self.checks,indent=2)+'\n')

    def destroy_node(self):
        self.camera.close()
        self.feedback.close()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inspect', action='store_true', help='measure ten seconds of stationary camera data without touching ROS services')
    parser.add_argument('--explore-only', action='store_true', help='explore speed steps without the final thirty repetitions')
    parser.add_argument('--directions', nargs='+', choices=list(DIRECTIONS), default=list(DIRECTIONS),
                        help='test only these directions; earlier evidence remains in its original run')
    args = parser.parse_args()
    config = yaml.safe_load((ROOT/'config/physical_test.yaml').read_text())
    config['directions'] = args.directions
    output = ROOT/'.benchmarks/physical-braking'/time.strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True)
    (output/'profile.yaml').write_text(yaml.safe_dump(config))
    # Check the external measurement device before interrupting production.
    camera = Camera(config,output)
    try:
        if args.inspect:
            end = time.monotonic()+10
            while time.monotonic()<end:
                camera.sample()
                time.sleep(.01)
            if camera.calibration is None:
                raise RuntimeError('cannot establish the optical reference')
            print(json.dumps({'camera_frames':camera.count,'marker_edge_residual_m':camera.calibration[3],
                              'last_pose':camera.last_pose,'output':str(output)}))
        else:
            end = time.monotonic()+10
            while camera.calibration is None or time.monotonic()-camera.last_time > config['maximum_frame_age_s']:
                camera.sample()
                if time.monotonic()>end:
                    raise RuntimeError('measurement camera cannot establish a fresh marker reference; production untouched')
            if args.explore_only:
                config['trials_per_direction'] = 0
            navigation['main'](lambda:BrakingTest(config,output,camera),output,
                               (f"base_test_linear_limit:={config['maximum_linear_speed_m_s']}",
                                f"base_test_angular_limit:={config['maximum_angular_speed_rad_s']}"))
    finally:
        camera.close()


if __name__ == '__main__':
    main()
