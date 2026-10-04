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


def plane_points(corners,matrix,origin):
    return cv2.perspectiveTransform((np.asarray(corners,dtype=float)-origin).reshape(-1,1,2),
                                    matrix).reshape(-1,2)


def metric_reference(markers, identifiers, side):
    if not math.isfinite(side) or side <= 0 or len(identifiers) < 2:
        raise ValueError('invalid marker dimensions or reference identifiers')
    corners = [np.asarray(markers[key], dtype=float) for key in identifiers]
    square = np.float32([[-side/2,-side/2],[side/2,-side/2],[side/2,side/2],[-side/2,side/2]])
    candidates = []
    for anchor in corners:
        origin = anchor.mean(axis=0)
        matrix = cv2.getPerspectiveTransform(np.float32(anchor-origin),square)
        reference = {key:plane_points(markers[key],matrix,origin) for key in identifiers}
        error = max(abs(np.linalg.norm(c-np.roll(c, -1, axis=0), axis=1)-side).max()
                    for c in reference.values())
        candidates.append((matrix,origin,reference,float(error)))
    matrix,origin,reference,error = min(candidates,key=lambda c:c[3])
    if not math.isfinite(error) or error > side*.15:
        raise ValueError(f'marker scale is inconsistent: {error:.4f} m edge residual')
    # Keep the physical origin at the first marker when another anchor fits better.
    shift = reference[identifiers[0]].mean(axis=0)
    rebase = np.eye(3)
    rebase[:2, 2] = -shift
    matrix = rebase @ matrix
    reference = {key:value-shift for key,value in reference.items()}
    return matrix, origin, reference, float(error)


def metric_pose(markers, matrix, origin, reference):
    common = sorted(set(markers).intersection(reference))
    if len(common) < 2:
        return None
    before = np.concatenate([reference[key] for key in common])
    after = np.concatenate([plane_points(markers[key],matrix,origin) for key in common])
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


def floor_reference(markers, floor_id, floor_side, body_id, body_side):
    """Rectify the floor and account for the tracked square's raised plane."""
    if isinstance(body_id,(list,tuple)):
        references = [floor_reference(markers,floor_id,floor_side,key,body_side) for key in body_id]
        references[0]['others'] = references[1:]
        return references[0]
    if any(not math.isfinite(s) or s <= 0 for s in (floor_side, body_side)):
        raise ValueError('invalid floor or body marker dimensions')
    unit = np.float32([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])
    matrix = cv2.getPerspectiveTransform(np.float32(markers[floor_id]), unit*floor_side)
    body = cv2.getPerspectiveTransform(unit*body_side, np.float32(markers[body_id]))
    ground = matrix@body
    ground /= ground[2,2]
    gradient = ground[2,:2]
    axis = np.array([-gradient[1],gradient[0]])
    axis = axis/np.linalg.norm(axis) if np.linalg.norm(axis)>1e-8 else np.array([1.,0.])
    jacobian = ground[:2,:2]-np.outer(ground[:2,2],gradient)
    magnification = float(np.linalg.norm(jacobian@axis))
    if not math.isfinite(magnification) or not .8<magnification<5:
        raise ValueError('inconsistent raised-marker magnification')
    points = plane_points([[0,0],*(-body_side/2*axis,body_side/2*axis)],ground,np.zeros(2))
    direction = points[2]-points[1]
    return {'matrix':matrix.tolist(), 'body_id':body_id, 'body_side_m':body_side,
            'corners_px':np.asarray(markers[body_id]).tolist(),
            'axis':axis.tolist(), 'magnification':magnification, 'origin':points[0].tolist(),
            'heading':math.atan2(direction[1],direction[0])}


def reference_marker_corners(markers, reference):
    """Locate the reference square from two visible squares on its rigid plane."""
    key = reference['body_id']
    if key in markers:
        return markers[key]
    common = [r for r in [reference,*reference.get('others',[])]
              if r['body_id'] in markers and 'corners_px' in r]
    if len(common)<2 or 'corners_px' not in reference:
        return None
    before = np.float64(np.concatenate([r['corners_px'] for r in common]))
    after = np.float64(np.concatenate([markers[r['body_id']] for r in common]))
    transform,_ = cv2.findHomography(before,after,0)
    if transform is None or max(np.linalg.norm(plane_points(before,transform,np.zeros(2))-after,axis=1))>3:
        return None
    return plane_points(reference['corners_px'],transform,np.zeros(2))


def floor_pose(markers, reference):
    key = reference['body_id']
    corners = reference_marker_corners(markers,reference)
    if corners is None:
        return None
    side,axis = reference['body_side_m'],np.array(reference['axis'])
    square = np.float32([[-side/2,-side/2],[side/2,-side/2],[side/2,side/2],[-side/2,side/2]])
    body = cv2.getPerspectiveTransform(square,np.float32(corners))
    points = plane_points([[0,0],*(-side/2*axis,side/2*axis)],np.array(reference['matrix'])@body,np.zeros(2))
    translation = (points[0]-reference['origin'])/reference['magnification']
    direction = points[2]-points[1]
    heading = navigation['angle'](math.atan2(direction[1],direction[0])-reference['heading'])
    headings = [heading]
    for other in reference.get('others',[]):
        pose = floor_pose(markers,other)
        if pose is not None:
            headings.append(heading+navigation['angle'](pose[2]-heading))
    heading = float(np.median(headings))
    return [float(translation[0]),float(translation[1]),heading]


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


def optical_return_twist(pose, target, body_to_marker):
    handedness = math.copysign(1,np.linalg.det(body_to_marker))
    heading = handedness*navigation['angle'](pose[2]-target[2])
    dx,dy = np.linalg.solve(body_to_marker,np.subtract(target[:2],pose[:2]))
    vx,vy = 2*(math.cos(heading)*dx+math.sin(heading)*dy), 2*(-math.sin(heading)*dx+math.cos(heading)*dy)
    scale = min(1,.05/max(math.hypot(vx,vy),1e-9))
    command = Twist()
    command.linear.x,command.linear.y = vx*scale,vy*scale
    command.angular.z = max(-.20,min(.20,-2*heading))
    return command


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
        self.floor_calibration = None
        self.reference_samples = []
        self.last_pose = None
        self.last_time = 0.0
        self.closed = False
        self.count = 0
        self.roi = None
        cv2.setNumThreads(2)
        self.detector = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco,'DetectorParameters_create')
                           else cv2.aruco.DetectorParameters())
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.parameters.aprilTagQuadDecimate = 1.5
        if config.get('floor_marker_id') is not None:
            self.parameters.minDistanceToBorder = 0
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            message = self.pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
            details = message.parse_error() if message is not None else 'no GStreamer error detail'
            self.close()
            raise RuntimeError(f'measurement camera pipeline failed to start: {details}')

    def detect(self, image):
        wanted = set(self.config['marker_ids'])
        if self.config.get('floor_marker_id') is not None and getattr(self,'calibration',None) is None:
            wanted.add(self.config['floor_marker_id'])
        for region in ([self.roi,None] if self.roi is not None else [None]):
            x,y,w,h = region or (0,0,image.shape[1],image.shape[0])
            corners,ids,_ = cv2.aruco.detectMarkers(image[y:y+h,x:x+w],self.detector,
                                                   parameters=self.parameters)
            markers = {} if ids is None else {
                int(key):c.reshape(4,2)+[x,y] for key,c in zip(ids.flatten(),corners)}
            if len(wanted.intersection(markers))<2:
                # Some small tilted squares decode at only one sampling scale.
                # Combine observations from the same image before dropping it.
                self.parameters.aprilTagQuadDecimate = 1.0
                try:
                    extra,keys,_ = cv2.aruco.detectMarkers(image[y:y+h,x:x+w],self.detector,
                                                          parameters=self.parameters)
                    if keys is not None:
                        for key,c in zip(keys.flatten(),extra):
                            markers.setdefault(int(key),c.reshape(4,2)+[x,y])
                finally:
                    self.parameters.aprilTagQuadDecimate = 1.5
            if len(wanted.intersection(markers))>=2:
                break
        if wanted <= set(markers):
            points = np.concatenate([markers[key] for key in wanted])
            # One marker width of context follows the bounded slow motion.
            # Lost tracking falls back to the full frame without resetting scale.
            pad = max(np.linalg.norm(c-np.roll(c,-1,axis=0),axis=1).max()
                      for key,c in markers.items() if key in wanted)
            lo = np.maximum(np.floor(points.min(axis=0)-pad),0).astype(int)
            hi = np.minimum(np.ceil(points.max(axis=0)+pad),
                            [image.shape[1],image.shape[0]]).astype(int)
            self.roi = (*lo,*(hi-lo))
        return markers

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
        markers = self.detect(image)
        identifiers = self.config['marker_ids']+[self.config['floor_marker_id']] if self.config.get('floor_marker_id') is not None else self.config['marker_ids']
        if self.calibration is None and all(key in markers for key in identifiers):
            self.reference_samples.append({key:markers[key] for key in identifiers})
            if len(self.reference_samples) < 10:
                record = {'time':received,'pts_ns':int(buffer.pts),'pose':None,
                          'ids':list(markers),'bytes':len(raw)}
                self.records.write(json.dumps(record)+'\n')
                return record
            stable = {key:np.median([s[key] for s in self.reference_samples],axis=0)
                      for key in identifiers}
            self.calibration = metric_reference(stable, self.config['marker_ids'], self.config['marker_side_m'])
            if self.config.get('floor_marker_id') is not None:
                self.floor_calibration = floor_reference(stable,self.config['floor_marker_id'],
                    self.config['floor_marker_side_m'],self.config['marker_ids'],self.config['marker_side_m'])
            matrix,origin,reference,error = self.calibration
            (self.output/'calibration.json').write_text(json.dumps({
                'matrix':matrix.tolist(),'origin_px':origin.tolist(),
                'reference':{key:value.tolist() for key,value in reference.items()},
                'marker_edge_residual_m':error,'floor_reference':self.floor_calibration,'opencv':cv2.__version__},indent=2)+'\n')
        pose = None if self.calibration is None else metric_pose(markers, *self.calibration[:3])
        marker_pose = pose
        if self.floor_calibration is not None:
            ground = floor_pose(markers,self.floor_calibration)
            common = sorted(set(markers).intersection(self.config['marker_ids']))
            # A tilted marker plane does not undergo a rigid 2D transform when
            # the base moves horizontally. Its old fit is diagnostic only;
            # floor tracking uses the actual observed squares independently.
            pose = None if ground is None or len(common)<2 else (
                ground,None if marker_pose is None else marker_pose[1],common)
        if pose is not None:
            self.last_pose = pose[0]
            self.last_time = received
        self.count += 1
        record = {'time':received, 'pts_ns':int(buffer.pts), 'pose':None if pose is None else pose[0],
                  'marker_pose':None if marker_pose is None else marker_pose[0],
                  'markers':{key:value.tolist() for key,value in markers.items() if key in self.config['marker_ids']},
                  'reference_marker_observed':None if self.floor_calibration is None else self.floor_calibration['body_id'] in markers,
                  'fit_error_m':None if pose is None else pose[1], 'ids':list(markers),'bytes':len(raw)}
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
        if camera.floor_calibration is not None:
            self.checks.update(measurement_scope='floor-referenced raised-marker tracking',floor_plane_alignment_verified=True)
        self.latest_safe = None
        self.return_transform = None
        self.optical_center = None
        self.create_subscription(Twist, '/cmd_vel_safe', self.safe_command, 10)

    def safe_command(self, message):
        self.latest_safe = [message.linear.x, message.linear.y, message.angular.z]

    def move(self, target):
        if self.return_transform is None or target!=self.center:
            return super().move(target, linear_limit=.10, angular_limit=.40)
        end = time.monotonic()+30
        while (math.dist(self.camera.last_pose[:2],self.optical_center[:2])>.002 or
               abs(navigation['angle'](self.camera.last_pose[2]-self.optical_center[2]))>.015):
            if time.monotonic()>end:
                raise RuntimeError('optical return to the fixed test center timed out')
            if not self.flags.get('base_motion_permitted'):
                self.stop()
                self.wait(lambda:self.flags.get('base_motion_permitted'),3)
                continue
            self.tick(optical_return_twist(self.camera.last_pose,self.optical_center,self.return_transform))
        self.stop()

    def calibrate_return(self):
        self.optical_center = self.camera.last_pose.copy()
        x,y,a = self.center
        rotation = np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        wheel,markers = [],[]
        for offset in [(0.02,0),(0,0.02)]:
            before_wheel,before_marker = np.array(self.pose[:2]),np.array(self.camera.last_pose[:2])
            dx,dy = rotation@offset
            super().move((x+dx,y+dy,a))
            wheel.append(rotation.T@(np.array(self.pose[:2])-before_wheel))
            markers.append(np.array(self.camera.last_pose[:2])-before_marker)
            super().move(self.center)
        measured = np.column_stack(wheel)
        if np.linalg.cond(measured)>10:
            raise RuntimeError('return calibration did not observe two independent base directions')
        self.return_transform = np.column_stack(markers)@np.linalg.inv(measured)
        values = np.linalg.svd(self.return_transform,compute_uv=False)
        if not np.all(np.isfinite(values)) or values.min()<.25 or values.max()>3:
            raise RuntimeError('return calibration has inconsistent optical motion')
        self.checks['return_calibration'] = {'body_to_marker':self.return_transform.tolist(),
                                            'center':self.optical_center}
        self.move(self.center)

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
        # Sparse valid camera captures may need a longer stationary observation,
        # while the configured stopping-time limit remains unchanged.
        while len(samples)<8 and time.monotonic()-start<3:
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
        # Initial command/feedback waits are stationary. Verify the terminal
        # capture intervals used below for speed coverage, not startup delay.
        if not rates or float(np.median(rates[-3:])) < (.015 if angular else .001):
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
            self.calibrate_return()
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
