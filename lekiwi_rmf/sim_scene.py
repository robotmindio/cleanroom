"""Gazebo task scenes: tracked spec, deterministic reset, and ground truth as an independent verifier.

config/sim_scenes.yaml declares each scene. `trial_poses` derives every object's pose for a
trial from the scene seed and the trial index alone, so trial N starts from the same state on
every run. `SceneController` spawns the scene objects once, then per reset moves the arm to
its start state through /arm_controller/follow_joint_trajectory (the guarded path every arm
motion uses), places the objects, waits for physics to settle and confirms objects, robot base
and arm from the simulator's ground truth. `judge` evaluates a success predicate on that
ground truth.

Ground truth and entity placement go through the `gz` command line against the running world
(`gz model -m NAME -p` reads the physics pose of a model by name; `gz service` creates and
places entities), from the caller's environment, which must reach the simulation's Gazebo
transport (the same GZ_PARTITION, if one is set). The ROS bridge of the world's pose stream
does not carry entity names, so it cannot identify objects.

A simulator verifies only what it models: a predicate reads declared variables, and a scene
that does not model one of them cannot verify that success (spec 20.4).
"""

import hashlib
import json
import math
from pathlib import Path
import random
import re
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

import yaml

from lekiwi_rmf.object_cameras import config_directory

# The world launch/sim.launch.py loads; a scene for another world could not be reset in it.
SIM_WORLD = 'cleanroom'
VARIABLES = ('rigid_geometry', 'articulation', 'friction_contact', 'deformable', 'thermal', 'electrical')
SHAPES = {'box': 3, 'cylinder': 2, 'sphere': 1}
NAME = re.compile(r'^[a-z][a-z0-9_]{0,31}$')
# A pose predicate reads rigid body poses and nothing else.
PREDICATE_VARIABLES = {'rigid_geometry'}
PREDICATE_FIELDS = {'object', 'region', 'min_z', 'max_z', 'min_displacement_m'}


def _number(value, name, low=-math.inf, high=math.inf):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be a number in [{low}, {high}]')
    return float(value)


def _object(name, value):
    keys = {'shape', 'size', 'mass', 'color', 'friction', 'pose', 'randomize'}
    if not NAME.match(name) or not isinstance(value, dict) or set(value) != keys:
        raise ValueError(f'object {name!r}: fields must be exactly {sorted(keys)}')
    if value['shape'] not in SHAPES or not isinstance(value['size'], list) or len(value['size']) != SHAPES[value['shape']]:
        raise ValueError(f'object {name}: shape {sorted(SHAPES)} with box [x,y,z], cylinder [radius, length], sphere [radius]')
    for item in value['size']:
        _number(item, f'{name} size', 0.005, 1.0)
    _number(value['mass'], f'{name} mass', 0.001, 5.0)
    _number(value['friction'], f'{name} friction', 0.0, 5.0)
    if not isinstance(value['color'], list) or len(value['color']) != 4:
        raise ValueError(f'object {name}: color is [r, g, b, a]')
    for item in value['color']:
        _number(item, f'{name} color', 0.0, 1.0)
    if not isinstance(value['pose'], list) or len(value['pose']) != 4:
        raise ValueError(f'object {name}: pose is [x, y, z, yaw] in the world frame')
    for item in value['pose']:
        _number(item, f'{name} pose', -50.0, 50.0)
    if not isinstance(value['randomize'], dict) or set(value['randomize']) - {'x', 'y', 'yaw'}:
        raise ValueError(f'object {name}: randomize holds half-widths for x, y, yaw')
    for axis, width in value['randomize'].items():
        _number(width, f'{name} randomize {axis}', 0.0, 1.0 if axis != 'yaw' else math.pi)
    return value


def load_scene(scene_id, path=None):
    """One validated scene and the SHA-256 of its canonical document, which every result records."""
    document = yaml.safe_load(Path(path or config_directory() / 'sim_scenes.yaml').read_text())
    if not isinstance(document, dict) or set(document) != {'scenes'} or not isinstance(document['scenes'], dict):
        raise ValueError('sim scenes must be a mapping with only `scenes`')
    if scene_id not in document['scenes']:
        raise ValueError(f'unknown sim scene {scene_id}')
    scene = document['scenes'][scene_id]
    keys = {'world', 'description', 'modeled', 'seed', 'settle_sec', 'tolerance', 'robot', 'objects'}
    if not isinstance(scene, dict) or set(scene) != keys:
        raise ValueError(f'scene {scene_id}: fields must be exactly {sorted(keys)}')
    if not isinstance(scene['modeled'], list) or not set(scene['modeled']) <= set(VARIABLES) or not scene['modeled']:
        raise ValueError(f'scene {scene_id}: modeled lists variables from {VARIABLES}')
    if type(scene['seed']) is not int or scene['world'] != SIM_WORLD:
        raise ValueError(f'scene {scene_id}: needs an integer seed and the launched world {SIM_WORLD}')
    _number(scene['settle_sec'], 'settle_sec', 0.0, 30.0)
    tolerance = scene['tolerance']
    if not isinstance(tolerance, dict) or set(tolerance) != {'position_m', 'yaw_rad', 'joint_rad', 'robot_position_m',
                                                            'robot_yaw_rad'}:
        raise ValueError(f'scene {scene_id}: tolerance needs position_m, yaw_rad, joint_rad, robot_position_m, robot_yaw_rad')
    for key, value in tolerance.items():
        _number(value, key, 0.0, 1.0)
    robot = scene['robot']
    if (not isinstance(robot, dict) or set(robot) != {'model', 'start', 'arm_start'} or not isinstance(robot['start'], list)
            or len(robot['start']) != 3):
        raise ValueError(f'scene {scene_id}: robot needs model, start [x, y, yaw] and arm_start')
    for item in robot['start']:
        _number(item, 'robot start', -50.0, 50.0)
    group_state(robot['arm_start'])
    if not isinstance(scene['objects'], dict) or not 1 <= len(scene['objects']) <= 16:
        raise ValueError(f'scene {scene_id}: needs 1..16 objects')
    for name, value in scene['objects'].items():
        _object(name, value)
    canonical = json.dumps({scene_id: scene}, sort_keys=True, separators=(',', ':'))
    return {'id': scene_id, **scene}, hashlib.sha256(canonical.encode()).hexdigest()


def group_state(name, path=None):
    """Joint targets of an SRDF arm group state, the arm's declared start."""
    states = ET.parse(path or config_directory() / 'lekiwi.srdf').getroot().findall(
        f"group_state[@name='{name}'][@group='arm']")
    if not re.fullmatch(r'[a-z][a-z0-9_]*', str(name)) or len(states) != 1:
        raise ValueError(f'the SRDF defines no single arm group state {name!r}')
    return {joint.get('name'): float(joint.get('value')) for joint in states[0].findall('joint')}


def trial_poses(scene, trial):
    """Each object's [x, y, z, yaw] for `trial`: a pure function of the scene seed and the index."""
    if type(trial) is not int or trial < 1:
        raise ValueError('trials are numbered from 1')
    generator = random.Random(f'{scene["seed"]}:{trial}')
    poses = {}
    for name in sorted(scene['objects']):
        value = scene['objects'][name]
        x, y, z, yaw = value['pose']
        spread = value['randomize']
        poses[name] = [x + generator.uniform(-1, 1) * spread.get('x', 0.0), y + generator.uniform(-1, 1) * spread.get('y', 0.0),
                       z, math.remainder(yaw + generator.uniform(-1, 1) * spread.get('yaw', 0.0), math.tau)]
    return poses


def entity_name(scene, name):
    return f'{scene["id"]}_{name}'


def object_sdf(scene, name):
    """A single-link rigid body with inertia from its shape, mass and colour."""
    value = scene['objects'][name]
    mass, size = value['mass'], value['size']
    if value['shape'] == 'box':
        x, y, z = size
        geometry = f'<box><size>{x} {y} {z}</size></box>'
        inertia = (mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12)
    elif value['shape'] == 'cylinder':
        radius, length = size
        geometry = f'<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>'
        side = mass * (3 * radius * radius + length * length) / 12
        inertia = (side, side, mass * radius * radius / 2)
    else:
        radius, = size
        geometry = f'<sphere><radius>{radius}</radius></sphere>'
        inertia = (2 * mass * radius * radius / 5,) * 3
    color = ' '.join(str(item) for item in value['color'])
    friction = value['friction']
    return (f'<?xml version="1.0"?><sdf version="1.10"><model name="{entity_name(scene, name)}"><link name="link">'
            f'<inertial><mass>{mass}</mass><inertia><ixx>{inertia[0]}</ixx><iyy>{inertia[1]}</iyy><izz>{inertia[2]}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<collision name="collision"><geometry>{geometry}</geometry><surface><friction><ode><mu>{friction}</mu>'
            f'<mu2>{friction}</mu2></ode></friction></surface></collision>'
            f'<visual name="visual"><geometry>{geometry}</geometry><material><ambient>{color}</ambient>'
            f'<diffuse>{color}</diffuse></material></visual></link></model></sdf>')


def sim_predicate(text, scene):
    """A success predicate over ground-truth object poses, stated by the evaluator, never by the skill."""
    value = json.loads(text) if isinstance(text, str) else text
    if not isinstance(value, dict) or set(value) - PREDICATE_FIELDS or value.get('object') not in scene['objects']:
        raise ValueError(f'a sim predicate names a scene object and only {sorted(PREDICATE_FIELDS)}')
    missing = PREDICATE_VARIABLES - set(scene['modeled'])
    if missing:
        raise ValueError(f'scene {scene["id"]} does not model {sorted(missing)}; it cannot verify this success')
    region = value.get('region')
    if region is not None and (not isinstance(region, list) or len(region) != 4
                               or any(type(item) not in (int, float) for item in region)
                               or not region[0] < region[2] or not region[1] < region[3]):
        raise ValueError('region is [x0, y0, x1, y1] in the world frame')
    for key in ('min_z', 'max_z', 'min_displacement_m'):
        if key in value:
            _number(value[key], key, -50.0, 50.0)
    if not set(value) - {'object'}:
        raise ValueError('a sim predicate needs a region, a height bound or a displacement')
    return {'object': value['object'], 'region': region, 'min_z': value.get('min_z'), 'max_z': value.get('max_z'),
            'min_displacement_m': value.get('min_displacement_m'), 'reads': sorted(PREDICATE_VARIABLES)}


def judge(predicate, pose, start):
    """Whether one ground-truth pose satisfies the predicate, with the measured values."""
    if pose is None:
        return False, {'reason': 'the object is absent from ground truth'}
    x, y, z, _ = pose
    checks = {}
    if predicate['region'] is not None:
        x0, y0, x1, y1 = predicate['region']
        checks['in_region'] = x0 <= x <= x1 and y0 <= y <= y1
    if predicate['min_z'] is not None:
        checks['above_min_z'] = z >= predicate['min_z']
    if predicate['max_z'] is not None:
        checks['below_max_z'] = z <= predicate['max_z']
    if predicate['min_displacement_m'] is not None:
        checks['displaced'] = start is not None and math.dist(pose[:3], start[:3]) >= predicate['min_displacement_m']
    return all(checks.values()), {'pose': [round(item, 5) for item in pose], 'checks': checks}


def yaw_of(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def confirm(scene, commanded, measured, robot, arm):
    """Reasons the measured initial state differs from the declared one; empty when it is confirmed."""
    tolerance, reasons = scene['tolerance'], []
    for name, target in commanded.items():
        pose = measured.get(name)
        if pose is None:
            reasons.append(f'{name} is absent from ground truth')
            continue
        if math.dist(pose[:2], target[:2]) > tolerance['position_m']:
            reasons.append(f'{name} settled {math.dist(pose[:2], target[:2]):.3f} m from its reset pose')
        if abs(math.remainder(pose[3] - target[3], math.tau)) > tolerance['yaw_rad']:
            reasons.append(f'{name} settled at another yaw')
    x, y, yaw = scene['robot']['start']
    if robot is None:
        reasons.append('the robot is absent from ground truth')
    elif math.dist(robot[:2], (x, y)) > tolerance['robot_position_m'] or \
            abs(math.remainder(robot[3] - yaw, math.tau)) > tolerance['robot_yaw_rad']:
        reasons.append('the robot base is not at the scene start; a reset never moves the base')
    start = group_state(scene['robot']['arm_start'])
    for joint, target in start.items():
        if joint not in arm or abs(arm[joint] - target) > tolerance['joint_rad']:
            reasons.append(f'arm joint {joint} is not at {scene["robot"]["arm_start"]}')
    return reasons


def parse_model_pose(text):
    """[x, y, z, yaw] from `gz model -m NAME -p` output, or None when the model does not exist."""
    match = re.search(r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]*)\]\s*\[([^\]]*)\]', text)
    if match is None:
        return None
    position, rotation = ([float(value) for value in group.split()] for group in match.groups())
    if len(position) != 3 or len(rotation) != 3 or not all(map(math.isfinite, position + rotation)):
        raise ValueError('gz reported a malformed pose')
    return [*position, rotation[2]]


def pose_text(pose):
    x, y, z, yaw = pose
    return (f'position: {{x: {x!r}, y: {y!r}, z: {z!r}}} '
            f'orientation: {{z: {math.sin(yaw / 2)!r}, w: {math.cos(yaw / 2)!r}}}')


class SceneController:
    """Spawns, resets and reads one scene of the running simulation, from a caller's ROS node."""

    def __init__(self, node, scene, timeout=10.0, run=subprocess.run):
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        self.node, self.scene, self.timeout, self._run = node, scene, timeout, run
        self.FollowJointTrajectory = FollowJointTrajectory
        self.arm = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self._lock = threading.Lock()
        self._joints, self._joint_time = {}, None
        node.create_subscription(JointState, '/joint_states', self._joint_state, qos_profile_sensor_data)

    def _joint_state(self, message):
        with self._lock:
            self._joints.update(zip(message.name, message.position))
            self._joint_time = time.monotonic()

    def _gz(self, *arguments):
        try:
            completed = self._run(['gz', *arguments], capture_output=True, text=True, timeout=self.timeout, check=False)
        except (OSError, subprocess.TimeoutExpired) as error:
            raise RuntimeError(f'gz {arguments[0]} failed: {error}') from error
        if completed.returncode:
            raise RuntimeError(f'gz {arguments[0]} failed: {completed.stderr.strip()[-500:]}')
        return completed.stdout

    def model_pose(self, name):
        return parse_model_pose(self._gz('model', '-m', name, '-p'))

    def _service(self, service, request_type, request):
        reply = self._gz('service', '-s', f'/world/{self.scene["world"]}/{service}', '--reqtype', request_type,
                         '--reptype', 'gz.msgs.Boolean', '--timeout', str(int(self.timeout * 1000)), '--req', request)
        return re.search(r'data:\s*true', reply) is not None

    def ready(self):
        if self.model_pose(self.scene['robot']['model']) is None:
            raise RuntimeError(f'the simulation has no model {self.scene["robot"]["model"]}; is the sim running and '
                               'reachable from this environment?')
        if not self.arm.wait_for_server(timeout_sec=self.timeout):
            raise RuntimeError('/arm_controller/follow_joint_trajectory is unavailable')

    def ground_truth(self):
        """Object poses by scene name, the robot base pose and the arm joints, read together."""
        objects = {name: self.model_pose(entity_name(self.scene, name)) for name in self.scene['objects']}
        robot = self.model_pose(self.scene['robot']['model'])
        with self._lock:
            return objects, robot, dict(self._joints), time.monotonic()

    def _arm_to_start(self):
        from trajectory_msgs.msg import JointTrajectoryPoint
        start = group_state(self.scene['robot']['arm_start'])
        goal = self.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(start)
        point = JointTrajectoryPoint(positions=[start[name] for name in start])
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]
        handle = self._wait(self.arm.send_goal_async(goal), 'the arm controller')
        if not handle.accepted:
            return 'the arm controller rejected the start-state trajectory'
        result = self._wait(handle.get_result_async(), 'the start-state trajectory', 30.0)
        return None if result.result.error_code == 0 else f'start-state trajectory failed: {result.result.error_string}'

    def _wait(self, future, what, timeout=None):
        deadline = time.monotonic() + (timeout or self.timeout)
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f'{what} did not answer within {timeout or self.timeout:.0f} s')
            time.sleep(.02)
        return future.result()

    def reset(self, trial):
        """Arm to start, objects to the trial's poses, settle, then confirm everything from ground truth."""
        commanded = trial_poses(self.scene, trial)
        reasons = []
        arm_failure = self._arm_to_start()
        if arm_failure:
            reasons.append(arm_failure)
        for name, pose in commanded.items():
            entity = entity_name(self.scene, name)
            if self.model_pose(entity) is None:
                sdf = object_sdf(self.scene, name)
                if not self._service('create', 'gz.msgs.EntityFactory', f"sdf: '{sdf}' pose: {{{pose_text(pose)}}}"):
                    reasons.append(f'the simulator refused to spawn {name}')
            elif not self._service('set_pose', 'gz.msgs.Pose', f"name: '{entity}' {pose_text(pose)}"):
                reasons.append(f'the simulator refused to place {name}')
        settled = self.node.get_clock().now().nanoseconds + int(self.scene['settle_sec'] * 1e9)
        deadline = time.monotonic() + self.scene['settle_sec'] * 20 + self.timeout
        while self.node.get_clock().now().nanoseconds < settled:  # simulation time, which may run slower
            if time.monotonic() > deadline:
                raise RuntimeError('simulation time is not advancing')
            time.sleep(.02)
        # The arm is judged from a joint state measured after settling, never from one before the motion.
        settled_at = time.monotonic()
        while (self._joint_time or 0) <= settled_at:
            if time.monotonic() > settled_at + self.timeout:
                reasons.append('no fresh /joint_states after the reset')
                break
            time.sleep(.02)
        measured, robot, joints, _ = self.ground_truth()
        reasons += confirm(self.scene, commanded, measured, robot, joints)
        return {'trial': trial, 'seed': self.scene['seed'], 'commanded': commanded, 'measured': measured,
                'robot': robot, 'arm': {name: joints.get(name) for name in group_state(self.scene['robot']['arm_start'])},
                'confirmed': not reasons, 'reasons': reasons}
