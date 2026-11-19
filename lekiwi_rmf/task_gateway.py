#!/usr/bin/env python3
"""Durable admission and observation of existing ROS actions; never a motion controller."""

import json
import hashlib
import math
from pathlib import Path
import re
from queue import Empty, SimpleQueue
import time
import uuid

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from unique_identifier_msgs.msg import UUID
import yaml

from lekiwi_rmf.action import Explore, RunSkill, Search
from lekiwi_rmf.exploration import load_navigation_footprint, map_geometry, task_limits
from lekiwi_rmf.exploration_owner_guard import owned_goal_id
from lekiwi_rmf.latency_budget import LatencyBudget
from lekiwi_rmf.srv import (
    ApplyPerceptionProfile, CancelTask, GetTask, QuerySpatial, SetPerceptionQuery, SubmitTask, TaskApi,
)
from lekiwi_rmf.perception_query import limits as query_limits
from lekiwi_rmf.task_journal import CAPABILITY_RESOURCES, REFUSALS, TERMINAL, Refused, TaskJournal
from lekiwi_rmf.navigation_pause import CODES as NAVIGATION_PAUSE_CODES, NavigationPause
from lekiwi_rmf.navigation_targets import (
    DEFAULT_STANDOFF_M, MAX_STANDOFF_M, MIN_STANDOFF_M, NoStandoff, approach_pose,
)
from lekiwi_rmf.navigation_observer import NavigationObserver, server_instance
from lekiwi_rmf.navigation_distance import NavigationDistance
from lekiwi_rmf.map_bundle import validate_map_bundle
from lekiwi_rmf.motion_constraints import (
    GRIPPER_GROUP, GRIPPER_JOINT, GROUP, MAX_CONSTRAINTS, MAX_REGION_M, MAX_TIME_SEC as MOTION_MAX_TIME_SEC,
    OPERATIONS as MOTION_OPERATIONS, PLANNING_FRAME, SCHEMA as MOTION_SCHEMA, TOOL_LINK, group_states,
)
from lekiwi_rmf.object_cameras import config_directory
from lekiwi_rmf.spatial_anchors import SCHEMA as SPATIAL_SCHEMA
from lekiwi_rmf.arm_trajectory import ARM_JOINTS, JOINT_LIMITS
from lekiwi_rmf.skills import (
    BASE_COMMANDS, KINDS, SKILL_ID, UNDECLARED_EFFECTS, catalog, load_registry, skill_parameters,
)


# The arm planning group: a reach moves these joints and holds the gripper where it is.
REACH_JOINTS = tuple(name for name in ARM_JOINTS if name != GRIPPER_JOINT)
# Named targets are MoveIt's own SRDF group states, never a second table.
SRDF = config_directory() / 'lekiwi.srdf'
ARM_STATES = {name: joints for name, joints in group_states(SRDF.read_text()).get(GROUP, {}).items()
              if set(joints) == set(REACH_JOINTS)}
GRIPPER_STATES = {name: joints[GRIPPER_JOINT]
                  for name, joints in group_states(SRDF.read_text()).get(GRIPPER_GROUP, {}).items()}
MOTION_SERVICE = '/robot/motion/solve'
SPATIAL_SERVICE = '/robot/spatial/query'
OBJECTS_SERVICE = '/robot/objects/query'
# The object map's answer schema (lekiwi_rmf/object_map.py); an answer under another is refused.
OBJECT_MAP_SCHEMA = 'lekiwi.object_map/1'
# The most entries one object query returns, newest sighting first.
SEARCH_LIMITS = ('max_duration_sec', 'max_travel_m')
# Schema 1 is SubmitTask/GetTask/CancelTask; schema 2 is the TaskApi services below.
API_SCHEMAS = (1, 2)
TASK_API = {name: f'/robot/tasks/v2/{name}'
            for name in ('validate', 'submit', 'status', 'events', 'cancel', 'transfer', 'compensate')}
# The joint and base-command mapping a plan was made for; a different body is refused, never guessed.
BODY_PROFILE = 'lekiwi:' + hashlib.sha256(json.dumps([*ARM_JOINTS, *BASE_COMMANDS]).encode()).hexdigest()[:16]
REFERENCE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,127}$')
TASK_ID = re.compile(r'^[A-Za-z0-9._-]{1,128}$')
CHECKPOINT_FIELDS = {'summary', 'goal', 'interpretation', 'references', 'plan', 'remote_commands',
                     'generations', 'resources', 'progress', 'pending_verifications'}
COMPENSATION_INTENTS = {'restore': ('reversible',), 'repair': ('reversible', 'compensable'),
                        'isolate': ('reversible', 'compensable', 'irreversible'),
                        'record': ('reversible', 'compensable', 'irreversible')}
# Stable `reason_code` values beside the readable detail. Every capability can end with the
# terminal ones; a capability adds its executor's own under `reasons`.
TERMINAL_CODES = ('succeeded', 'canceled', 'canceled_before_dispatch', 'goal_rejected', 'capability_disabled',
                  'executor_error', 'executor_aborted', 'incomplete', 'outcome_unknown')
PAUSE_CODES = (*NAVIGATION_PAUSE_CODES, 'awaiting_navigation_stop', 'navigation_stopped', 'executor_paused')
UNKNOWN_CODES = ('result_unconfirmed', 'acceptance_unconfirmed', 'arm_stop_unconfirmed', 'pause_stop_unconfirmed')


def check_value(spec, value, field, now=None):
    """Enforce exactly what a parameter descriptor states; the manifest publishes the same descriptor."""
    kind = spec['type']
    if kind == 'object':
        if not isinstance(value, dict):
            raise Refused('invalid_request', f'{field} must be an object', field=field)
        if 'properties' in spec:
            check_object(spec, value, f'{field}.', now)
        return
    valid = {'boolean': type(value) is bool, 'integer': type(value) is int, 'string': isinstance(value, str),
             'number': type(value) in (float, int) and math.isfinite(value)}[kind]
    if not valid:
        raise Refused('invalid_request', f'{field} must be a {"finite " * (kind == "number")}{kind}', field=field)
    if kind == 'string':
        # Lengths count the text with its whitespace collapsed; a pattern matches the whole string.
        length = len(' '.join(value.split()))
        if not spec.get('min_length', 0) <= length <= spec.get('max_length', math.inf):
            raise Refused('invalid_request', f'{field} must have {spec.get("min_length", 0)}..'
                          f'{spec.get("max_length")} characters', field=field)
        if 'pattern' in spec and not re.fullmatch(spec['pattern'], value):
            raise Refused('invalid_request', f'{field} must match {spec["pattern"]}', field=field)
    elif kind != 'boolean':
        if (value < spec.get('min', -math.inf) or value > spec.get('max', math.inf)
                or value <= spec.get('exclusive_min', -math.inf)):
            lower = f'({spec["exclusive_min"]}' if 'exclusive_min' in spec else f'[{spec.get("min", "-inf")}'
            raise Refused('invalid_request', f'{field} must be within {lower}, {spec.get("max", "inf")}]'
                          f'{" " + spec["unit"] if "unit" in spec else ""}', field=field)
        if now is not None and 'max_ahead_sec' in spec and not now < value <= now + spec['max_ahead_sec']:
            raise Refused('precondition_failed', f'{field} must be within the next {spec["max_ahead_sec"]} seconds',
                          field=field)
    if 'enum' in spec and value not in spec['enum']:
        raise Refused('invalid_request', f'{field} must be one of {spec["enum"]}', field=field)


def check_object(spec, values, prefix='', now=None):
    """An object's fields, its `alternatives` (exactly one group of fields) and its `excludes`."""
    properties = spec['properties']
    unknown = sorted(set(values) - set(properties))
    if unknown:
        raise Refused('invalid_request', f'unknown parameter {prefix}{unknown[0]}; accepted: {list(properties)}',
                      field=f'{prefix}{unknown[0]}')
    for name, item in properties.items():
        if name in values:
            check_value(item, values[name], f'{prefix}{name}', now)
        elif item.get('required'):
            raise Refused('invalid_request', f'{prefix}{name} is required', field=f'{prefix}{name}')
    alternatives = spec.get('alternatives')
    if alternatives:
        given = [item for item in alternatives
                 if any(name in values for name in (*item['required'], *item.get('optional', ())))]
        if len(given) != 1 or not set(given[0]['required']) <= values.keys():
            choices = ' or '.join('{' + ', '.join(item['required']) + '}' for item in alternatives)
            raise Refused('invalid_request', f'{prefix or "parameters "}requires exactly one of {choices}',
                          field=prefix.rstrip('.') or None)
    for name, others in spec.get('excludes', {}).items():
        for other in others:
            if (values.get(name, properties[name]['default']) != properties[name]['default']
                    and values.get(other, properties[other]['default']) != properties[other]['default']):
                raise Refused('invalid_request', f'{prefix}{name} cannot be combined with {prefix}{other}',
                              field=f'{prefix}{other}')


def _navigate_goal(values):
    target = values.get('target', {}).get('pose', values)
    if 'x' not in target:
        raise ValueError('an anchored, point or object target becomes a pose at admission')
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x, pose.pose.position.y = float(target['x']), float(target['y'])
    yaw = target.get('yaw', 0.0)
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return NavigateToPose.Goal(pose=pose)


def _explore_goal(values):
    return Explore.Goal(revisit_known=values.get('revisit_known', False),
                        max_duration_sec=float(values.get('max_duration_sec', 0.0)),
                        max_radius_m=float(values.get('max_radius_m', 0.0)),
                        resume_task_id=values.get('resume_task_id', ''))


def _search_goal(values):
    region = values.get('region', {'x': 0.0, 'y': 0.0, 'radius_m': 0.0})
    return Search.Goal(query=values['query'], region_x=float(region['x']), region_y=float(region['y']),
                       region_radius_m=float(region['radius_m']),
                       max_duration_sec=float(values.get('max_duration_sec', 0.0)),
                       max_travel_m=float(values.get('max_travel_m', 0.0)),
                       target_coverage=float(values.get('target_coverage', 0.0)))


def _skill_goal(values):
    return RunSkill.Goal(skill_id=values['skill_id'],
                         parameters_json=json.dumps(values.get('parameters', {}), sort_keys=True, allow_nan=False))


def _joint_goal(group, joints):
    """MoveIt plans from the current state to these joints, checks the current scene and executes."""
    goal = MoveGroup.Goal()
    goal.request.group_name = group
    goal.request.num_planning_attempts, goal.request.allowed_planning_time = 3, 5.0
    goal.request.start_state.is_diff = True
    goal.request.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=name, position=float(position), tolerance_above=0.01, tolerance_below=0.01,
                        weight=1.0) for name, position in joints.items()])]
    goal.planning_options.plan_only = False
    return goal


def _reach_goal(values):
    # A joint target, normally a /robot/motion/solve solution, or one of the SRDF's named arm states.
    joints = values['joints'] if 'joints' in values else ARM_STATES[values['named']]
    return _joint_goal(GROUP, {name: joints[name] for name in REACH_JOINTS})


def _gripper_goal(values):
    return _joint_goal(GRIPPER_GROUP, {GRIPPER_JOINT: values['position_rad'] if 'position_rad' in values
                                       else GRIPPER_STATES[values['state']]})


def reach_result(result):
    """A MoveGroup result without its trajectories; success is MoveIt's own error code."""
    code = (result.get('error_code') or {}).get('val')
    points = ((result.get('executed_trajectory') or {}).get('joint_trajectory') or {}).get('points') or []
    names = ((result.get('executed_trajectory') or {}).get('joint_trajectory') or {}).get('joint_names') or []
    final = dict(zip(names, points[-1]['positions'])) if points else None
    return {'moveit_error_code': code, 'error_code': 0 if code == MoveItErrorCodes.SUCCESS else (code or -1),
            'planning_time_sec': result.get('planning_time'), 'final_joints': final}


def arm_stopped(result):
    """Whether a finished skill's arm controller confirmed its last chunk terminal; older runners omit it."""
    try:
        report = json.loads(result.get('result_json') or '{}')
    except (ValueError, RecursionError):
        return False
    return isinstance(report, dict) and report.get('arm_stopped', True) is not False


def _navigate_effective(gateway, values):
    pose = (gateway.resolve_target(values['target']) if 'target' in values
            else {'x': values['x'], 'y': values['y'], 'yaw': values.get('yaw', 0.0)})
    return {**{name: value for name, value in values.items() if name != 'target'}, **pose,
            'max_duration_sec': gateway._timeout}


def _explore_effective(gateway, values):
    limits = gateway._exploration_defaults
    if limits is None or values.get('resume_task_id'):
        return None
    duration, radius = task_limits(values.get('max_duration_sec', 0.0), values.get('max_radius_m', 0.0),
                                   limits['max_duration_sec'], limits['max_radius_m'])
    return {'max_duration_sec': duration, 'max_radius_m': radius,
            'revisit_known': values.get('revisit_known', False), 'resume_task_id': ''}


def _search_effective(gateway, values):
    defaults = gateway._search_defaults
    if defaults is None:
        return None
    return {**values, **{name: values.get(name, 0.0) or defaults[name] for name in (*SEARCH_LIMITS, 'target_coverage')}}


def _search_effect(state, result):
    if result.get('found') and result.get('evidence_ids'):
        return {'kind': 'label_detected', 'evidence_ids': result['evidence_ids']}
    return None


def _skill_advertisement(gateway):
    return {'effective_defaults': {'parameters': {}},
            'skills': [{**skill, 'requires_schema': 1 if skill['effects']['reversibility'] == 'reversible' else 2}
                       for skill in gateway._skills],
            # The implementation kinds the runner executes, so a client acquires only runnable skills.
            'implementation_kinds': list(KINDS),
            'effects': {'per_skill': True, 'undeclared': UNDECLARED_EFFECTS}, 'requires_schema': 'per_skill'}


_NUMBER, _METRES = {'type': 'number'}, {'type': 'number', 'unit': 'm'}
_POSE = {'x': {**_METRES, 'required': True}, 'y': {**_METRES, 'required': True},
         'yaw': {'type': 'number', 'unit': 'rad', 'default': 0.0}}
_REFERENCE = {'type': 'string', 'min_length': 1, 'max_length': 128, 'required': True}
_BUDGET = {'type': 'number', 'min': 0, 'default': 0.0}
_STANDOFF = {**_METRES, 'min': MIN_STANDOFF_M, 'max': MAX_STANDOFF_M, 'default': DEFAULT_STANDOFF_M}
# One descriptor per capability. Request validation, the capability manifest and execution all
# read it, so what is advertised is what is enforced:
#   action       the ROS action that executes it; `feedback` names the field telling what a goal is doing
#   switch       the gateway parameter that enables it in this deployment, and its default
#   schema       `properties` {type, unit, min, max, exclusive_min, default, required, enum, ...},
#                `alternatives` (exactly one group of fields) and `excludes` (fields refused together)
#   bounds       tracked limits of this deployment, layered over `schema` at admission and in the manifest
#   goal         the action goal for validated parameters
#   check        refusals that need the gateway's state, before anything is held
#   effective    the parameters that will run, resolved once the resources are free; they are dispatched
#   effects      reversibility and the compensation an owner may plan; None when a skill declares its own
#   reasons      reason codes of its executor, beside the terminal ones every capability shares
#   result       what is kept of the native result; `stopped` whether it proves the executor stopped
#   effect       the evidence of an effect a finished task carries, if any
#   advertise    manifest keys beyond the derived ones: effective defaults and what a client may name
# Adding a capability is one entry here plus its resources in task_journal.CAPABILITY_RESOURCES.
CAPABILITIES = {
    'navigate': {
        'action': ('/navigate_to_pose', NavigateToPose), 'switch': ('allow_navigation', True), 'frame': 'map',
        'schema': {
            'properties': {
                'x': _METRES, 'y': _METRES, 'yaw': _POSE['yaw'],
                'deadline_unix_sec': {'type': 'number', 'unit': 's', 'exclusive_min': 0, 'max_ahead_sec': 300},
                'max_travel_m': {**_METRES, 'exclusive_min': 0, 'max': 10},
                'target': {'type': 'object', 'properties': {
                    'pose': {'type': 'object', 'properties': _POSE},
                    'anchored': {'type': 'object', 'properties': {
                        'map_pose': {'type': 'object', 'required': True, 'properties': _POSE},
                        'site_ref': _REFERENCE, 'map_ref': _REFERENCE, 'revision': _REFERENCE}},
                    'point': {'type': 'object', 'properties': {'x': _POSE['x'], 'y': _POSE['y']}},
                    'standoff_m': _STANDOFF,
                    'object': {'type': 'object', 'properties': {
                        'object_id': {'type': 'string', 'pattern': '[0-9a-f]{32}', 'required': True},
                        'standoff_m': _STANDOFF}}},
                    'alternatives': [{'required': ['pose']}, {'required': ['anchored']},
                                     {'required': ['point'], 'optional': ['standoff_m']},
                                     {'required': ['object']}]}},
            'alternatives': [{'required': ['x', 'y'], 'optional': ['yaw']}, {'required': ['target']}]},
        'goal': _navigate_goal, 'effective': _navigate_effective,
        'effects': {'reversibility': 'reversible', 'compensation': 'Navigate back to the starting pose.'},
        'effect': lambda state, _result: {'kind': 'pose_reached', 'source': 'nav2'} if state == 'succeeded' else None,
        'reasons': ('budget_exhausted',),
        'advertise': lambda gateway: {'effective_defaults': {'yaw': 0.0, 'max_duration_sec': gateway._timeout},
                                      'targets': gateway.target_kinds()},
        'description': 'Map-frame x, y; yaw in radians. Or one target: {pose}, {anchored} (a pose remembered under '
                       'an earlier map revision, reprojected into the current one), {point, standoff_m} (stand '
                       'that far from a map point, facing it) or {object: {object_id, standoff_m}} (the same '
                       f'around an entry of {OBJECTS_SERVICE}: its last observed estimate, not a verified '
                       'place). effective_parameters report the pose that runs and what it was resolved from. '
                       'Pauses preserve the active-time budget.'},
    'explore': {
        'action': ('/robot/explore', Explore), 'switch': ('allow_exploration', True), 'feedback': 'stage',
        'schema': {
            'properties': {
                'revisit_known': {'type': 'boolean', 'default': False},
                'max_duration_sec': {**_BUDGET, 'unit': 's'}, 'max_radius_m': {**_BUDGET, 'unit': 'm'},
                'resume_task_id': {'type': 'string', 'pattern': '([0-9a-f]{32})?', 'default': ''}},
            'excludes': {'resume_task_id': ['max_duration_sec', 'max_radius_m']}},
        'bounds': lambda gateway: {name: {'max': value} for name, value in (gateway._exploration_defaults or {}).items()},
        'goal': _explore_goal, 'effective': _explore_effective,
        'effects': {'reversibility': 'reversible',
                    'compensation': 'Map growth is data; navigate back to the starting pose.'},
        'advertise': lambda gateway: {'effective_defaults': (
            {**gateway._exploration_defaults, 'revisit_known': False, 'resume_task_id': ''}
            if gateway._exploration_defaults is not None else None)},
        'description': ('Omit parameters to explore unmapped space. revisit_known=true also revisits mapped space. '
                        'Omitted/zero limits use the tracked robot limits; positive limits may only reduce them. '
                        'Duration is active seconds, radius is metres around the starting pose. '
                        'resume_task_id is a saved native exploration UUID and preserves its original bounds.')},
    'search': {
        'action': ('/robot/search', Search), 'switch': ('allow_search', True), 'feedback': 'stage', 'frame': 'map',
        'schema': {'properties': {
            'query': {'type': 'string', 'min_length': 1, 'max_length': 128, 'required': True},
            'region': {'type': 'object', 'unit': 'm', 'default': None, 'properties': {
                'x': _POSE['x'], 'y': _POSE['y'], 'radius_m': {**_METRES, 'exclusive_min': 0, 'required': True}}},
            'max_duration_sec': {**_BUDGET, 'unit': 's'}, 'max_travel_m': {**_BUDGET, 'unit': 'm'},
            'target_coverage': {**_BUDGET, 'unit': 'fraction', 'max': 1}}},
        'bounds': lambda gateway: ({name: {'max': gateway._search_defaults[name]} for name in SEARCH_LIMITS}
                                   if gateway._search_defaults else {}),
        'goal': _search_goal, 'effective': _search_effective,
        'effects': {'reversibility': 'reversible',
                    'compensation': 'Observations are data; navigate back to the starting pose.'},
        'effect': _search_effect,
        'reasons': ('found', 'coverage_reached', 'inaccessible', 'budget_exhausted', 'incapable', 'unmapped'),
        'advertise': lambda gateway: {
            'effective_defaults': (dict(gateway._search_defaults, region=None)
                                   if gateway._search_defaults is not None else None),
            'labels': '/robot/perception/profile',
            'outcomes': ['found', 'coverage_reached', 'inaccessible', 'budget_exhausted', 'incapable', 'unmapped',
                         'canceled', 'failed']},
        'description': ('Moves the base between viewpoints chosen by expected information gain until the query '
                        'label is detected, target_coverage of the searchable mapped region has fresh evidence '
                        'of absence, or a budget ends. query is a native label of the perception profile; '
                        'region {x, y, radius_m} is map-frame, default the tracked motion bound. Results carry '
                        'evidence_ids, coverage_fraction and a negative-evidence report; "not found" holds only '
                        'for the covered cells. Zero/omitted limits use the tracked limits; positive limits may '
                        'only reduce them.')},
    'skill': {
        'action': ('/robot/skills/run', RunSkill), 'switch': ('allow_skills', False), 'feedback': 'stage',
        # Only the shape; admission resolves the registry entry, its pins and its declared parameters.
        'schema': {'properties': {
            'skill_id': {'type': 'string', 'pattern': SKILL_ID.pattern, 'required': True},
            'parameters': {'type': 'object', 'default': {}}, 'version': {'type': 'integer'},
            'artifact_sha256': {'type': 'string'}}},
        'goal': _skill_goal, 'check': lambda gateway, values: gateway._check_skill(values),
        'effects': None, 'stopped': arm_stopped, 'advertise': _skill_advertisement,
        'description': 'Run a qualified registered skill implementation on the arm; version and '
                       'artifact_sha256 pin the entry a plan was made for. Success means the implementation '
                       'finished, not that the world reached the goal; verify the effect independently. '
                       'Each skill lists the resources it commands or reads (arm, base, camera); '
                       'admission holds the capability resources whatever the skill uses.'},
    'reach': {
        'action': ('/move_action', MoveGroup), 'switch': ('allow_reach', False), 'feedback': 'state',
        'schema': {
            'properties': {
                'joints': {'type': 'object', 'unit': 'rad', 'properties': {
                    name: {'type': 'number', 'unit': 'rad', 'min': JOINT_LIMITS[name][0], 'max': JOINT_LIMITS[name][1],
                           'required': True} for name in REACH_JOINTS}},
                'named': {'type': 'string', 'enum': sorted(ARM_STATES)}},
            'alternatives': [{'required': ['joints']}, {'required': ['named']}]},
        'goal': _reach_goal, 'result': reach_result,
        'effective': lambda _gateway, values: {'joints': ARM_STATES[values['named']]} if 'named' in values else None,
        'effects': {'reversibility': 'reversible',
                    'compensation': 'Move the arm back to its starting joint positions.'},
        'advertise': lambda _gateway: {
            'effective_defaults': {}, 'joints': list(REACH_JOINTS),
            'limits': {name: list(JOINT_LIMITS[name]) for name in REACH_JOINTS},
            'named_states': ARM_STATES, 'solver': MOTION_SERVICE},
        'description': 'Move the arm to a joint target, normally a feasible solution from '
                       f'{MOTION_SERVICE}, or to a named SRDF arm state. MoveIt plans from the current state '
                       'and checks the current scene again; success means the motion finished, not that a '
                       'constraint holds. Verify it with the solver.'},
    'gripper': {
        'action': ('/move_action', MoveGroup), 'switch': ('allow_gripper', False), 'feedback': 'state',
        'schema': {
            'properties': {
                'state': {'type': 'string', 'enum': sorted(GRIPPER_STATES)},
                'position_rad': {'type': 'number', 'unit': 'rad', 'min': JOINT_LIMITS[GRIPPER_JOINT][0],
                                 'max': JOINT_LIMITS[GRIPPER_JOINT][1]}},
            'alternatives': [{'required': ['state']}, {'required': ['position_rad']}]},
        'bounds': lambda gateway: {'position_rad': dict(zip(('min', 'max'), gateway._gripper_limits))},
        'goal': _gripper_goal, 'result': reach_result,
        'effective': lambda gateway, values: ({'position_rad': gateway._gripper_states[values['state']]}
                                              if 'state' in values else None),
        # Opening releases what the claw holds and closing may grasp: moving it back restores neither.
        'effects': {'reversibility': 'compensable',
                    'compensation': 'Move the gripper back to its starting position and verify what it holds.'},
        'advertise': lambda gateway: {'effective_defaults': {}, 'joint': GRIPPER_JOINT,
                                      'named_states': gateway._gripper_states},
        'description': 'Move the claw to a named SRDF gripper state or to position_rad within its limits, through '
                       'MoveIt and the arm controller like a reach. Success means the motion finished, not that '
                       'anything is held; verify a grasp or a release independently.'},
}


def task_values(capability, text):
    """Keep untrusted commands finite, small and limited to what the capability's descriptor states."""
    if capability not in CAPABILITIES:
        raise Refused('invalid_request', 'unsupported capability', field='capability')
    if len(text.encode()) > 4096:
        raise Refused('invalid_request', 'parameters exceed 4096 bytes')
    try:
        values = json.loads(text)
    except RecursionError as error:
        raise Refused('invalid_request', 'parameters JSON is nested too deeply') from error
    if not isinstance(values, dict):
        raise Refused('invalid_request', 'parameters must be an object')
    check_object(CAPABILITIES[capability]['schema'], values)
    return values


def task_goal(capability, text):
    """The action goal of validated parameters, with their canonical JSON."""
    values = task_values(capability, text)
    return CAPABILITIES[capability]['goal'](values), json.dumps(values, sort_keys=True, allow_nan=False)


def reason_code(capability, state, result):
    """The stable reason of a terminal native result: the executor's own outcome when it declares one."""
    if result.get('outcome') in CAPABILITIES[capability].get('reasons', ()):
        return result['outcome']
    if state != 'failed':
        return state
    if result.get('error_code', 0):
        return 'executor_error'
    return 'incomplete' if result.get('complete') is False else 'executor_aborted'


class TaskGateway(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_tasks', **kwargs)
        domain = self.context.get_domain_id()
        suffix = f'_{domain}' if domain else ''
        if self.get_parameter('use_sim_time').value:
            suffix += '_sim'
        self.declare_parameter('database_path', str(Path.home() / f'.ros/lekiwi_tasks{suffix}.sqlite3'))
        self.declare_parameter('navigation_timeout_sec', 180.0)
        # The arm capabilities are off unless config/robot_tasks.yaml, their tracked switch, enables them.
        # The explorer's tracked allow_search owns availability of the search action server.
        self._enabled = {name: self.declare_parameter(*spec['switch']).value for name, spec in CAPABILITIES.items()}
        # Empty selects config/robot_skills.yaml, as in the skill runner; set both or neither.
        self.declare_parameter('registry_file', '')
        self.declare_parameter('map_bundle', '')
        # The Nav2 profile whose footprint admits a standoff pose; the launch passes the one Nav2 runs.
        self.declare_parameter('navigation_params_file', str(config_directory() / 'nav2_params.yaml'))
        # An object target is refused when its sigma_m exceeds this share of the standoff distance.
        self._sigma_ratio = self.declare_parameter('object_target_max_sigma_ratio', 0.5).value
        if type(self._sigma_ratio) not in (float, int) or not 0 < self._sigma_ratio <= 1:
            raise ValueError('object_target_max_sigma_ratio must be within (0, 1]')
        # The calibration MoveIt applies to the claw's limits and endpoints; empty in simulation.
        self.declare_parameter('arm_calibration_file', '')
        try:
            # Experimental skills run only through an acquisition protocol, never a user task.
            self._registry = load_registry(self.get_parameter('registry_file').value or None)
        except (OSError, ValueError) as error:
            self.get_logger().error(f'skill registry unavailable: {error}')
            self._enabled['skill'], self._registry = False, {}
        self._skills = catalog(self._registry)
        self._timeout = self.get_parameter('navigation_timeout_sec').value
        if type(self._timeout) not in (float, int) or not math.isfinite(self._timeout) or self._timeout <= 0:
            raise ValueError('navigation_timeout_sec must be positive and finite')
        self._journal = TaskJournal(self.get_parameter('database_path').value)
        self._skill_effects = {skill['skill_id']: skill['effects'] for skill in self._skills}
        # Dispatched tasks observed live by this process; recovered ones stay uncertain until then.
        self._live = set()
        self._resource_snapshot = None
        self._pending = {}
        self._replies = SimpleQueue()
        self._queried = {}
        self._sent = set()
        self._started = {}
        # Monotonic admission and dispatch times of tasks this process saw; a restart loses only samples.
        self._budget, self._admitted_at, self._dispatched_at = LatencyBudget('task'), {}, {}
        self._recovering = {row['id'] for row in self._active() if row['dispatched']}
        self._observed = {capability: set() for capability in CAPABILITIES}
        self._terminal_seen = {}
        self._navigation = NavigationPause()
        self._children = NavigationObserver(self)
        self._pausing = {row['id'] for row in self._active()
                         if row['capability'] == 'navigate' and row['pause_requested']
                         and row['dispatched'] and not row['cancel_requested']}
        self._pause_started = {identifier: time.monotonic() - 3.0 for identifier in self._pausing}
        self._unknown_results = {}
        self._unknown_cancels = {}
        self._unknown_since = {}
        self._guard = (False, 0.0)
        self._ack = (b'', 0.0)
        self._action_clients = {}
        self._result_clients = {}
        self._cancel_clients = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._events = self.create_publisher(String, '/robot/tasks/events', latched)
        self._map_publisher = self.create_publisher(String, '/robot/map_context', latched)
        bundle_path = self.get_parameter('map_bundle').value
        map_context = {'ready': False}
        if bundle_path:
            validate_map_bundle(bundle_path)
            map_context = {'ready': True, 'map_id': hashlib.sha256(Path(bundle_path).read_bytes()).hexdigest(),
                           'epoch': uuid.uuid4().hex, 'started_at': self.get_clock().now().nanoseconds / 10**9}
        self._map_publisher.publish(String(data=json.dumps(map_context)))
        self._exploration_defaults = None
        self.create_subscription(String, '/robot/explore/defaults', self._explore_defaults, latched)
        self._search_defaults = None
        self.create_subscription(String, '/robot/search/defaults', self._search_defaults_message, latched)
        self._owner = self.create_publisher(UUID, '/robot/explore/navigation_owner', 10)
        self.create_subscription(Bool, '/safety/base_motion_permitted',
                                 lambda msg: self._navigation.permission(msg.data, time.monotonic()), latched)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 lambda msg: self._navigation.collision(msg.action_type == CollisionMonitorState.STOP,
                                                                       msg.polygon_name, time.monotonic()), 1)
        self.create_subscription(Bool, '/robot/explore/navigation_guard_ready', self._guard_ready, 1)
        self.create_subscription(UUID, '/robot/explore/navigation_guarded', self._guard_ack, 10)
        clients = {}
        for capability, spec in CAPABILITIES.items():
            name, action = spec['action']
            if name not in clients:
                # Capabilities executed by one action server share its clients and observations.
                clients[name] = (ActionClient(self, action, name),
                                 self.create_client(action.Impl.GetResultService, f'{name}/_action/get_result'),
                                 self.create_client(CancelGoal, f'{name}/_action/cancel_goal'))
                users = [other for other, item in CAPABILITIES.items() if item['action'][0] == name]
                self.create_subscription(action.Impl.FeedbackMessage, f'{name}/_action/feedback', self._feedback, 10)
                self.create_subscription(GoalStatusArray, f'{name}/_action/status',
                                         lambda message, users=users: self._status(message, users), latched)
            (self._action_clients[capability], self._result_clients[capability],
             self._cancel_clients[capability]) = clients[name]
        self.create_service(SubmitTask, '/robot/tasks/submit', self._submit)
        self.create_service(GetTask, '/robot/tasks/get', self._get)
        self.create_service(CancelTask, '/robot/tasks/cancel', self._cancel)
        self.create_service(Trigger, '/robot/capabilities', self._capabilities)
        for name, handler in (('validate', self._api_validate), ('submit', self._api_submit),
                              ('status', self._api_status), ('events', self._api_events),
                              ('cancel', self._api_cancel), ('transfer', self._api_transfer),
                              ('compensate', self._api_compensate)):
            self.create_service(TaskApi, TASK_API[name], self._api(handler))
        # Holders of exclusive resources, for nodes that enforce them outside this gateway.
        self._resources = self.create_publisher(String, '/robot/resources', latched)
        self._publish_resources()
        self.create_timer(0.2, self._publish_resources, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._observation_status = self.create_client(Trigger, '/robot/observations/status')
        # Only to report availability; clients call the detector's query service directly.
        self._perception_query = self.create_client(SetPerceptionQuery, '/robot/perception/set_query')
        self._perception_profile = self.create_client(ApplyPerceptionProfile, '/robot/perception/apply_profile')
        # Only to report availability; planners call the motion solver directly.
        self._motion = self.create_client(TaskApi, MOTION_SERVICE)
        self._gripper_states, self._gripper_limits = self._gripper_model()
        self._resolver_setup()
        self._distance = NavigationDistance()
        self.create_subscription(Odometry, '/odom', self._distance.update, 1)
        self.create_service(Trigger, '/robot/navigation/odometry', self._odometry)
        self._diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._health, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.2, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _health(self):
        message = DiagnosticArray(status=[self._budget.status()])
        message.header.stamp = self.get_clock().now().to_msg()
        self._diagnostics.publish(message)

    def _active(self):
        return self._journal.active()

    def _row(self, identifier):
        return self._journal.row(identifier)

    def _update(self, identifier, state, detail, result=None, code=None):
        if result is not None:
            metadata = json.loads(self._row(identifier)['result']).get('_task')
            if metadata is not None:
                result = {**result, '_task': metadata}
        if not self._journal.update(identifier, state, detail, result, code):
            return
        row = self._row(identifier)
        self._events.publish(String(data=json.dumps(dict(row), default=lambda value: bytes(value).hex())))
        if state in TERMINAL:
            dispatched = self._dispatched_at.pop(identifier, None)
            self._admitted_at.pop(identifier, None)
            if dispatched is not None:
                self._budget.record('execution', time.monotonic() - dispatched)
            self._release_requests(identifier, row)

    def _release_requests(self, identifier, row):
        for kind, client in (('send', None), ('result', self._result_clients[row['capability']]),
                             ('cancel', self._cancel_clients[row['capability']])):
            key = (kind, identifier)
            future = self._pending.pop(key, None)
            if future is not None:
                if client is not None:
                    client.remove_pending_request(future)
                future.cancel()
            self._queried.pop(key, None)
        self._recovering.discard(identifier)
        self._sent.discard(identifier)
        self._started.pop(identifier, None)
        self._terminal_seen.pop(identifier, None)
        self._pausing.discard(identifier)
        self._pause_started.pop(identifier, None)
        self._unknown_results.pop(identifier, None)
        self._unknown_cancels.pop(identifier, None)
        self._unknown_since.pop(identifier, None)

    def _effects(self, capability, values):
        return CAPABILITIES[capability]['effects'] or self._skill_effects.get(values.get('skill_id'), UNDECLARED_EFFECTS)

    def _schema(self, capability):
        """The capability's parameter descriptor under this deployment's tracked limits.

        Admission enforces it and the manifest publishes it, so the two cannot differ.
        """
        spec = CAPABILITIES[capability]
        bounds = spec['bounds'](self) if 'bounds' in spec else {}
        return {**spec['schema'], 'properties': {name: {**item, **bounds.get(name, {})}
                                                 for name, item in spec['schema']['properties'].items()}}

    def _admit(self, task_id, capability, parameters_json, request=None, dry_run=False):
        """Every admission check for SubmitTask (no request) and the negotiated API, before any dispatch.

        Returns (row, summary): an existing row with no summary for an idempotent retry,
        no row for a dry run, or the newly admitted row.
        """
        if not isinstance(task_id, str) or not TASK_ID.match(task_id):
            raise Refused('invalid_request', 'task_id requires 1..128 ASCII letters, numbers, dots, hyphens or underscores')
        try:
            values = task_values(capability, parameters_json)
        except Refused:
            raise
        except (ValueError, TypeError, OverflowError) as error:
            raise Refused('invalid_request', str(error)) from error
        parameters = json.dumps(values, sort_keys=True, allow_nan=False)
        spec = CAPABILITIES[capability]
        effects = self._effects(capability, values)
        if request is None and effects['reversibility'] != 'reversible':
            # Schema 1 reports an implementation that finished as succeeded; with effects
            # that cannot be undone a client must see execution and effect separately.
            raise Refused('schema_incompatible', f'{capability} has {effects["reversibility"]} effects; '
                          'submit it through the task API schema 2')
        compensates = (request or {}).get('compensates') or {}
        digest = hashlib.sha256(json.dumps(
            {'capability': capability, 'parameters': values, 'goal': request.get('goal'),
             'compensates': compensates.get('task_id', '')}, sort_keys=True).encode()).hexdigest() if request else ''
        row = self._row(task_id)
        if row is not None:
            same = (row['digest'] == digest if row['digest'] and digest
                    else (row['capability'], row['parameters']) == (capability, parameters))
            if not same:
                raise Refused('identity_conflict', 'task_id already has different parameters', task_id=task_id)
            # A retry after a lost reply returns what was admitted; it never sends a second goal.
            return row, None
        # The tracked limits and deadlines of this moment, exactly as the manifest states them.
        check_object(self._schema(capability), values, now=time.time())
        if not self._enabled[capability]:
            raise Refused('capability_unavailable', 'capability disabled by this deployment profile')
        if 'check' in spec:
            spec['check'](self, values)
        resources = CAPABILITY_RESOURCES[capability]
        if request is not None:
            self._check_request(capability, request, resources, values)
        self._journal.require_free(resources)
        if any(self._observed.values()) or self._children.busy():
            raise Refused('resource_busy', 'another ROS client has an active navigation, exploration, search, skill or arm goal',
                          holders=[{'resource': 'base', 'task_id': '', 'owner': 'external ROS client'}])
        if not self._action_clients[capability].server_is_ready():
            raise Refused('capability_unavailable', 'capability action server unavailable',
                          retryable=True, next_condition='wait_capability_ready')
        effective = spec['effective'](self, values) if 'effective' in spec else None
        metadata = {'parameters': values, 'effective_parameters': effective}
        if capability == 'navigate' and 'max_travel_m' in values:
            meter = self._distance.status()
            if not meter['ready']:
                raise Refused('precondition_failed', 'bounded navigation requires fresh odometry',
                              retryable=True, next_condition='wait_odometry')
            metadata['odometry_origin'] = {'epoch': meter['epoch'], 'meters': meter['meters']}
        if request is not None:
            metadata['request'] = {key: request[key] for key in
                                   ('goal', 'body_profile', 'frame', 'units', 'authority') if key in request}
        summary = {'capability': capability, 'parameters': values, 'effective_parameters': effective,
                   'resources': list(resources), 'effects': effects, 'body_profile': BODY_PROFILE, 'digest': digest}
        if dry_run:
            return None, summary
        goal_id = bytes(owned_goal_id().uuid) if capability == 'navigate' else uuid.uuid4().bytes
        row = self._journal.admit(task_id, capability, parameters, goal_id, result={'_task': metadata},
                                  resources=resources, owner=(request or {}).get('owner', ''),
                                  api_schema=2 if request is not None else 1, digest=digest,
                                  compensates=compensates.get('task_id', ''))
        self._admitted_at[task_id] = time.monotonic()
        self._publish_resources()
        return row, summary

    def _check_skill(self, values):
        """Refuse what the runner would reject, before any resource is held."""
        skill = self._registry.get(values['skill_id'])
        if skill is None or skill.status != 'qualified':
            raise Refused('competence_insufficient', f'{values["skill_id"]} is not a registered skill of this robot'
                          if skill is None else f'{skill.skill_id} is {skill.status}, not qualified',
                          skill_id=values['skill_id'], status=skill.status if skill else None)
        for pin in ('version', 'artifact_sha256'):
            if pin in values and values[pin] != getattr(skill, pin):
                raise Refused('competence_insufficient', f'{skill.skill_id} is registered with {pin} '
                              f'{getattr(skill, pin)}, not {values[pin]}', skill_id=skill.skill_id,
                              version=skill.version, artifact_sha256=skill.artifact_sha256)
        try:
            skill_parameters(skill, json.dumps(values.get('parameters', {}), allow_nan=False))
        except ValueError as error:
            raise Refused('invalid_request', str(error)) from error

    def _check_request(self, capability, request, resources, values):
        """Refuse a plan made for another body, frame, unit system or resource set."""
        body = request.get('body_profile')
        if body is not None and body != BODY_PROFILE:
            raise Refused('body_profile_mismatch', f'request is for body {body}; this robot is {BODY_PROFILE}')
        frame, native = request.get('frame'), CAPABILITIES[capability].get('frame')
        if frame is not None and frame != native:
            raise Refused('map_unaligned', f'{capability} takes {native or "no"} frame coordinates, not {frame!r}',
                          next_condition='fix_request', field='frame')
        if request.get('units', 'si') != 'si':
            raise Refused('unit_mismatch', 'parameters are SI: metres, radians, seconds', field='units')
        declared = request.get('resources')
        if declared is not None and (len(set(declared)) != len(declared) or set(declared) != set(resources)):
            raise Refused('resource_mismatch', f'{capability} uses exactly {sorted(resources)}', field='resources')
        compensates = request.get('compensates')
        if compensates is not None:
            self._journal.fence(compensates['task_id'], compensates['generation'], request['owner'], reconciled=True)
            self._journal.compensation_step(compensates['task_id'], capability, values)

    def _submit(self, request, response):
        try:
            row, _ = self._admit(request.task_id, request.capability, request.parameters_json)
            response.accepted, response.state, response.detail = True, row['state'], row['detail']
        except (ValueError, TypeError, OverflowError) as error:
            response.accepted, response.state, response.detail = False, 'rejected', str(error)
        return response

    def _api(self, handler):
        """One negotiated wrapper: an unsupported schema or payload is refused, never reinterpreted."""
        def call(request, response):
            response.schema = 2
            try:
                if request.schema != 2:
                    raise Refused('schema_incompatible', f'this service speaks schema 2; the gateway supports {list(API_SCHEMAS)}',
                                  supported=list(API_SCHEMAS))
                if len(request.request_json.encode()) > 32768:
                    raise Refused('invalid_request', 'request exceeds 32768 bytes')
                values = json.loads(request.request_json or '{}')
                if not isinstance(values, dict):
                    raise Refused('invalid_request', 'request must be a JSON object')
                result = handler(values)
                response.ok, response.response_json = True, json.dumps(result, allow_nan=False, sort_keys=True)
            except Refused as error:
                response.ok, response.error, response.detail = False, error.code, str(error)
                response.response_json = json.dumps(error.data, allow_nan=False, sort_keys=True)
            except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as error:
                response.ok, response.error, response.detail = False, 'invalid_request', str(error)
                response.response_json = json.dumps(Refused('invalid_request', '').data, sort_keys=True)
            return response
        return call

    @staticmethod
    def _envelope(values):
        allowed = {'task_id', 'capability', 'parameters', 'owner', 'goal', 'body_profile', 'frame', 'units',
                   'resources', 'authority', 'compensates'}
        if set(values) - allowed:
            raise Refused('invalid_request', f'unknown request fields {sorted(set(values) - allowed)}')
        if values.get('capability') not in CAPABILITIES:
            raise Refused('invalid_request', 'unsupported capability', field='capability')
        parameters = values.get('parameters', {})
        if not isinstance(parameters, dict):
            raise Refused('invalid_request', 'parameters must be an object')
        owner = values.get('owner')
        if not isinstance(owner, str) or not REFERENCE.match(owner):
            raise Refused('invalid_request', 'owner requires a 1..128 character agent or worker reference')
        authority = values.get('authority')
        if authority is not None and (not isinstance(authority, str) or not REFERENCE.match(authority)):
            raise Refused('invalid_request', 'authority is a policy or approval reference, never a credential')
        goal = values.get('goal')
        if goal is not None and (not isinstance(goal, dict) or set(goal) - {'id', 'revision'}
                                 or not isinstance(goal.get('id'), str) or not REFERENCE.match(goal['id'])
                                 or type(goal.get('revision', 0)) is not int or goal.get('revision', 0) < 0):
            raise Refused('invalid_request', 'goal requires an id and an optional nonnegative integer revision')
        compensates = values.get('compensates')
        if compensates is not None and (not isinstance(compensates, dict) or set(compensates) != {'task_id', 'generation'}
                                        or not isinstance(compensates['task_id'], str)
                                        or type(compensates['generation']) is not int):
            raise Refused('invalid_request', 'compensates requires the original task_id and generation')
        if any(key in values and not isinstance(values[key], str) for key in ('body_profile', 'frame', 'units')):
            raise Refused('invalid_request', 'body_profile, frame and units are strings')
        resources = values.get('resources')
        if resources is not None and (not isinstance(resources, list) or not all(isinstance(r, str) for r in resources)):
            raise Refused('invalid_request', 'resources is a list of resource names')
        return values.get('task_id'), values['capability'], json.dumps(parameters, allow_nan=False), values

    def _api_validate(self, values):
        row, summary = self._admit(*self._envelope(values), dry_run=True)
        return {'valid': True, 'existing': self._view(row) if row is not None else None, 'request': summary}

    def _api_submit(self, values):
        row, summary = self._admit(*self._envelope(values))
        return {'admitted': summary is not None, 'existing': summary is None, 'task': self._view(row)}

    def _task_row(self, values):
        identifier = values.get('task_id')
        if not isinstance(identifier, str):
            raise Refused('invalid_request', 'task_id must be a string')
        generation = values.get('generation')
        if generation is not None and type(generation) is not int:
            raise Refused('invalid_request', 'generation must be an integer')
        row = self._row(identifier)
        if row is None:
            raise Refused('not_found', 'task not found')
        return row, generation

    def _api_status(self, values):
        row, generation = self._task_row(values)
        if generation is not None:
            # Reading the journal with its own generation is how a successor reconciles.
            self._journal.reconcile(row['id'], generation)
        return {'task': self._view(self._row(row['id']))}

    def _api_events(self, values):
        cursor, limit, identifier = values.get('cursor', 0), values.get('limit', 100), values.get('task_id', '')
        if type(cursor) is not int or cursor < 0 or type(limit) is not int or not 1 <= limit <= 100:
            raise Refused('invalid_request', 'cursor is a nonnegative integer and limit is 1..100')
        if not isinstance(identifier, str):
            raise Refused('invalid_request', 'task_id must be a string')
        if identifier and values.get('generation') is not None:
            row, generation = self._task_row(values)
            self._journal.reconcile(row['id'], generation)
        return self._journal.events_since(cursor, identifier, limit)

    def _api_cancel(self, values):
        identifier = values.get('task_id', '')
        if not isinstance(identifier, str):
            raise Refused('invalid_request', 'task_id must be a string')
        requester = values.get('requester', '')
        if requester and (not isinstance(requester, str) or not REFERENCE.match(requester)):
            raise Refused('invalid_request', 'requester is a 1..128 character reference')
        if not identifier:
            active = self._active()
            identifier = active[0]['id'] if active else ''
        row, generation = self._task_row({**values, 'task_id': identifier})
        if generation is not None:
            # A fenced caller cannot stop work it no longer owns; an unfenced stop is always accepted.
            self._journal.fence(row['id'], generation)
        accepted = row['state'] not in TERMINAL
        if accepted:
            self._journal.request_cancel(row['id'], requester=requester or 'task_api')
            self._update(row['id'], 'canceling', 'cancellation requested; awaiting terminal ROS result')
        view = self._view(self._row(row['id']))
        return {'accepted': accepted, 'receipt': view['cancel'], 'stop': view['stop'], 'task': view}

    def _api_transfer(self, values):
        if set(values) - {'task_id', 'generation', 'owner', 'new_owner', 'checkpoint'}:
            raise Refused('invalid_request', 'transfer takes task_id, generation, owner, new_owner and checkpoint')
        row, generation = self._task_row(values)
        owner, successor, checkpoint = values.get('owner'), values.get('new_owner'), values.get('checkpoint')
        if generation is None or not all(isinstance(value, str) and REFERENCE.match(value) for value in (owner, successor)):
            raise Refused('invalid_request', 'transfer requires generation, owner and new_owner references')
        if (not isinstance(checkpoint, dict) or set(checkpoint) - CHECKPOINT_FIELDS
                or not isinstance(checkpoint.get('summary'), str) or not 1 <= len(checkpoint['summary']) <= 2000
                or len(json.dumps(checkpoint, allow_nan=False)) > 16384):
            raise Refused('invalid_request', f'checkpoint is at most 16 KiB of {sorted(CHECKPOINT_FIELDS)} '
                          'with a readable summary; never credentials')
        row = self._journal.transfer(row['id'], generation, owner, successor, checkpoint)
        self._publish_resources()
        return {'task': self._view(row), 'generation': row['generation'], 'previous_generation': generation}

    def _api_compensate(self, values):
        if set(values) - {'task_id', 'generation', 'owner', 'plan'}:
            raise Refused('invalid_request', 'compensate takes task_id, generation, owner and plan')
        row, generation = self._task_row(values)
        plan = values.get('plan')
        if generation is None or not isinstance(values.get('owner'), str):
            raise Refused('invalid_request', 'compensate requires generation and owner')
        if (not isinstance(plan, dict) or set(plan) - {'intent', 'steps', 'note'} or plan.get('intent') not in COMPENSATION_INTENTS
                or not isinstance(plan.get('steps', []), list) or len(plan.get('steps', [])) > 8
                or not isinstance(plan.get('note', ''), str) or len(plan.get('note', '')) > 2000):
            raise Refused('invalid_request', f'plan has intent {sorted(COMPENSATION_INTENTS)}, at most 8 steps and a note')
        steps = plan.get('steps', [])
        for step in steps:
            if (not isinstance(step, dict) or set(step) != {'capability', 'parameters'}
                    or step['capability'] not in CAPABILITIES):
                raise Refused('invalid_request', 'each step is {capability, parameters}')
            try:
                task_values(step['capability'], json.dumps(step['parameters'], allow_nan=False))
            except (ValueError, TypeError) as error:
                raise Refused('invalid_request', f'step {step["capability"]}: {error}') from error
        if row['state'] not in TERMINAL:
            raise Refused('precondition_failed', 'cancel the task and wait for its confirmed stop before compensating',
                          retryable=True, next_condition='wait_task_terminal')
        reversibility = self._effects(row['capability'], json.loads(row['parameters']))['reversibility']
        if reversibility not in COMPENSATION_INTENTS[plan['intent']]:
            raise Refused('irreversible_effect', f'a {reversibility} effect cannot be {plan["intent"]}d; '
                          'stop, isolate and record it instead')
        compensation = {'plan': {'intent': plan['intent'], 'steps': steps, 'note': plan.get('note', '')},
                        'status': 'planned' if steps else 'recorded', 'recorded_at': time.time(),
                        'reversibility': reversibility, 'automatic': False,
                        # Compensation repairs state; it never achieves or replaces the original goal.
                        'counts_as_goal_success': False,
                        'material_rollback': reversibility == 'reversible' and plan['intent'] == 'restore',
                        'original_state': row['state'], 'tasks': {}}
        row = self._journal.record_compensation(row['id'], generation, values['owner'], compensation)
        return {'task': self._view(row)}

    def _view(self, row):
        """Separate dimensions of one task: command phase, stop, effect evidence and goal verification."""
        result = json.loads(row['result'])
        metadata = result.get('_task', {})
        state, identifier = row['state'], row['id']
        terminal = state in TERMINAL
        if not terminal and not row['dispatched']:
            phase = 'admitted'
        elif state == 'unknown':
            phase = 'uncertain'
        elif not terminal and row['navigation_stopped']:
            # Its native attempt already ended; the journal holds the pause, not a live executor.
            phase = 'paused'
        elif not terminal and identifier in self._recovering and identifier not in self._live:
            phase = 'uncertain'
        elif state == 'paused':
            phase = 'paused'
        elif not terminal:
            phase = 'running'
        elif not row['dispatched']:
            phase = 'not_started'
        else:
            phase = 'stopped' if state == 'canceled' else 'execution_finished'
        outcome = None
        if terminal:
            outcome = 'unknown' if result.get('outcome_known') is False else state
        if terminal:
            stop = 'confirmed'
        elif phase == 'uncertain':
            stop = 'uncertain'
        elif row['cancel_requested']:
            stop = 'stopping'
        else:
            stop = 'not_requested'
        observed = CAPABILITIES[row['capability']].get('effect', lambda _state, _result: None)(state, result)
        effect = {'observed': observed is not None, 'verified_by_robot': False, **(observed or {})}
        effects = self._effects(row['capability'], json.loads(row['parameters']))
        compensation = json.loads(row['compensation'] or 'null')
        request = metadata.get('request', {})
        receipt = json.loads(row['cancel_receipt'] or 'null')
        if receipt is None and row['cancel_requested']:
            receipt = {'receipt_id': None, 'task_id': identifier,
                       'reason': 'deadline' if row['cancel_requested'] == 2 else 'requested'}
        return {
            'task_id': identifier, 'capability': row['capability'], 'state': state, 'detail': row['detail'],
            # The stable reason of a pause, an unconfirmed state or a terminal result; `detail` is for people.
            'reason_code': result.get('reason_code'),
            'api_schema': row['api_schema'], 'owner': row['owner'], 'generation': row['generation'],
            'reconciled': row['reconciled_generation'] == row['generation'],
            'resources': json.loads(row['resources']), 'holding_resources': not terminal,
            'execution': {'phase': phase, 'dispatched': bool(row['dispatched']), 'outcome': outcome},
            'cancel': receipt, 'stop': stop, 'effect': effect,
            # Only the authorized verifier can credit the goal; the robot never does.
            'goal': {**request.get('goal', {}), 'verified': None, 'verifier': 'recuriant'},
            'effects': effects, 'compensation': compensation,
            'compensation_required': (terminal and bool(row['dispatched']) and state != 'succeeded'
                                      and not result.get('goal_rejected')
                                      and effects['reversibility'] != 'reversible' and compensation is None),
            'compensates': row['compensates'] or None,
            'checkpoint': json.loads(row['checkpoint'] or 'null'),
            'parameters': metadata.get('parameters'), 'effective_parameters': metadata.get('effective_parameters'),
            'times': self._journal.times(identifier), 'cursor': self._journal.last_event(identifier),
            'journal_id': self._journal.journal_id,
            'result': {key: value for key, value in result.items() if key != '_task'}}

    def _publish_resources(self):
        state = self._journal.resource_state()
        if state != self._resource_snapshot:
            self._resource_snapshot = state
            self._resources.publish(String(data=json.dumps(
                {'schema': 1, 'journal_id': self._journal.journal_id, 'resources': state}, sort_keys=True)))

    def _get(self, request, response):
        row = self._row(request.task_id)
        response.found = row is not None
        if row is not None:
            response.capability, response.state, response.detail, response.result_json = row['capability'], row['state'], row['detail'], row['result']
        return response

    def _cancel(self, request, response):
        active = self._active()
        identifier = request.task_id or (active[0]['id'] if active else '')
        row = self._row(identifier)
        if row is None:
            response.detail = 'task not found'
            return response
        if row['state'] not in TERMINAL:
            self._journal.request_cancel(identifier, requester='CancelTask')
            self._update(identifier, 'canceling', 'cancellation requested; awaiting terminal ROS result')
        row = self._row(identifier)
        response.accepted, response.state, response.detail = True, row['state'], row['detail']
        return response

    def _explore_defaults(self, message):
        try:
            if len(message.data.encode()) > 4096:
                raise ValueError('exploration defaults exceed 4096 bytes')
            values = json.loads(message.data)
            if (not isinstance(values, dict) or set(values) != {'max_duration_sec', 'max_radius_m'}
                    or any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0
                           for value in values.values())):
                raise ValueError('invalid exploration defaults')
        except (ValueError, TypeError, RecursionError, OverflowError):
            self._exploration_defaults = None
            self.get_logger().error('invalid exploration defaults contract')
            return
        self._exploration_defaults = values

    def _search_defaults_message(self, message):
        try:
            if len(message.data.encode()) > 4096:
                raise ValueError('search defaults exceed 4096 bytes')
            values = json.loads(message.data)
            if (not isinstance(values, dict) or set(values) != {*SEARCH_LIMITS, 'target_coverage'}
                    or any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0
                           for value in values.values()) or values['target_coverage'] > 1):
                raise ValueError('invalid search defaults')
        except (ValueError, TypeError, RecursionError, OverflowError):
            self._search_defaults = None
            self.get_logger().error('invalid search defaults contract')
            return
        self._search_defaults = values

    def _capability(self, name):
        """One manifest entry, derived from the descriptor that validates and executes the capability."""
        spec, schema = CAPABILITIES[name], self._schema(name)
        properties, effects = schema['properties'], spec['effects']
        preconditions = [
            {'condition': 'capability_enabled', 'switch': spec['switch'][0], 'satisfied': bool(self._enabled[name])},
            {'condition': 'action_server_ready', 'action': spec['action'][0],
             'satisfied': self._action_clients[name].server_is_ready()}]
        return {
            'name': name, 'available': all(item['satisfied'] for item in preconditions),
            'parameters': list(properties),
            'required_parameters': [*(key for key, item in properties.items() if item.get('required')),
                                    *schema.get('alternatives', [{'required': []}])[0]['required']],
            'defaults': {key: item['default'] for key, item in properties.items() if 'default' in item},
            'description': spec['description'], 'resources': list(CAPABILITY_RESOURCES[name]),
            'units': {key: item['unit'] for key, item in properties.items() if 'unit' in item},
            'frame': spec.get('frame'), 'effects': effects,
            'requires_schema': 1 if effects and effects['reversibility'] == 'reversible' else 2,
            # What validation enforces, what the capability holds and needs, and how it can end.
            'schema': {**schema, 'resources': list(CAPABILITY_RESOURCES[name]), 'preconditions': preconditions,
                       'reason_codes': [*TERMINAL_CODES, *spec.get('reasons', ())]},
            **spec['advertise'](self)}

    @staticmethod
    def _body_profile():
        return {'id': BODY_PROFILE, 'arm_joints': list(ARM_JOINTS), 'base_commands': list(BASE_COMMANDS)}

    def _capabilities(self, _request, response):
        response.success = True
        response.message = json.dumps({'schema': 1, 'capabilities': [self._capability(name) for name in CAPABILITIES],
            'terminal_states': sorted(TERMINAL),
            # `reason_code` values of a task by kind of state, and what each refusal code means for a retry.
            'reason_codes': {'terminal': sorted({*TERMINAL_CODES, *(code for spec in CAPABILITIES.values()
                                                                    for code in spec.get('reasons', ()))}),
                             'paused': list(PAUSE_CODES), 'unknown': list(UNKNOWN_CODES)},
            'refusals': {code: {'retryable': retryable, 'next_condition': condition}
                         for code, (retryable, condition) in REFUSALS.items()},
            'task_api': {'schemas': list(API_SCHEMAS), 'type': 'lekiwi_rmf/srv/TaskApi', 'services': TASK_API,
                         'body_profile': self._body_profile(), 'journal_id': self._journal.journal_id,
                         'resources': self._journal.resource_state(), 'resource_topic': '/robot/resources',
                         'description': ('Schema 2 separates admission, execution, stop and effect evidence; '
                                         'grants a generation (fencing token) per owner; serializes tasks whose '
                                         'resources overlap; returns cancel receipts; and replays journal events '
                                         'from a cursor. Capabilities with requires_schema 2 are refused through '
                                         'SubmitTask.')},
            # Each observation record states its own `grounding` and `position`; the gateway sees neither.
            'observations': {'available': self._observation_status.service_is_ready(),
                             'schema': 1, 'grounding': 'per_observation',
                             'services': {'query': '/robot/observations/query',
                                          'evidence': '/robot/observations/evidence',
                                          'prepare': '/robot/observations/prepare',
                                          'pin': '/robot/observations/pin',
                                          'status': '/robot/observations/status'},
                             'events': '/robot/observations/events',
                             # Open-vocabulary detectors localize the active query; closed ones reject it.
                             # Every open observation records the query revision it ran under.
                             'query': {'available': self._perception_query.service_is_ready(),
                                       'service': '/robot/perception/set_query',
                                       'type': 'lekiwi_rmf/srv/SetPerceptionQuery',
                                       'profile': '/robot/perception/profile', **query_limits(),
                                       'description': 'Set the labels the detector localizes from the next frame. '
                                                      'Empty labels restore the default query; expected_revision '
                                                      'applies the change only while that revision is in force. '
                                                      'part_of ties a label to its whole; concept_revisions add '
                                                      'learned concepts matched by exemplar.'},
                             # Thresholds, suppression, resolution and budget form a proposal profile per
                             # recognizer; every observation records the revision it ran under.
                             'proposal_profile': {
                                 'available': self._perception_profile.service_is_ready(),
                                 'service': '/robot/perception/apply_profile',
                                 'type': 'lekiwi_rmf/srv/ApplyPerceptionProfile',
                                 'profile': '/robot/perception/profile',
                                 'description': 'Switch a recognizer to a tracked profile, an exact earlier revision, '
                                                'or a candidate selected by a recorded calibration study. '
                                                'expected_revision is required; the response reads back the '
                                                'profile in force.'}},
            # Only availability is observed here; clients query the object map and spatial memory directly.
            'objects': {'available': self._objects.service_is_ready(), 'schema': OBJECT_MAP_SCHEMA,
                        'service': OBJECTS_SERVICE, 'type': 'lekiwi_rmf/srv/QuerySpatial',
                        'estimates': 'local_landmark', 'verification': 'last_observed', 'qualified': 'per_object',
                        'navigate_target': 'object',
                        'description': ('Where labelled things were last estimated, in the current map revision. '
                                        'Entries are local landmark estimates, not identities: last_observed, never '
                                        'currently_verified, and qualified is false unless the position was measured '
                                        'under a validated calibration. Each entry states its method, sigma_m and '
                                        'state; look again before acting on one.')},
            'spatial': {'available': self._spatial.service_is_ready(), 'schema': SPATIAL_SCHEMA,
                        'service': SPATIAL_SERVICE, 'type': 'lekiwi_rmf/srv/QuerySpatial',
                        'pose': {'request': {'op': 'pose'}},
                        'description': 'Map revisions, anchors and projections; pose answers where the robot is '
                                       'in the current revision, for remembering a spot as an anchored target.'},
            'motion': {'available': self._motion.service_is_ready(), 'schema': MOTION_SCHEMA,
                       'service': MOTION_SERVICE, 'type': 'lekiwi_rmf/srv/TaskApi',
                       'operations': list(MOTION_OPERATIONS), 'planning_frame': PLANNING_FRAME, 'tool_link': TOOL_LINK,
                       'execute': 'reach', 'limits': {'max_constraints': MAX_CONSTRAINTS, 'max_region_m': MAX_REGION_M,
                                                      'max_time_sec': MOTION_MAX_TIME_SEC},
                       'description': ('Geometric feasibility over MoveIt: solve finds arm joints whose tool pose '
                                       'satisfies position (sphere/box around a point or an observed detection) and '
                                       'orientation (one tool axis within a cone) constraints, verified by MoveIt FK, '
                                       'collision check and optionally a motion plan; verify checks the current pose '
                                       'from fresh joint states. Answers are feasible, infeasible or unknown and hold '
                                       'only for the scene they were computed in.')},
            'examples': [{'capability': 'explore', 'parameters': {}},
                         {'capability': 'explore', 'parameters': {'revisit_known': True}}],
            'services': {'submit': '/robot/tasks/submit', 'get': '/robot/tasks/get', 'cancel': '/robot/tasks/cancel'},
            'events': '/robot/tasks/events'})
        return response

    def _guard_ready(self, message):
        self._guard = message.data, time.monotonic()

    def _odometry(self, _request, response):
        result = self._distance.status()
        response.success = result['ready']
        response.message = json.dumps(result, allow_nan=False)
        return response

    def _guard_ack(self, message):
        self._ack = bytes(message.uuid), time.monotonic()

    def _feedback(self, message):
        row = self._journal.goal(bytes(message.goal_id.uuid))
        if row is None or row['state'] in TERMINAL:
            return
        self._live.add(row['id'])
        if row['cancel_requested'] or row['navigation_stopped']:
            return
        if row['id'] in self._pausing:
            return
        # The feedback field that names what a running goal is doing; Nav2 reports poses, not a stage.
        field = CAPABILITIES[row['capability']].get('feedback')
        stage = getattr(message.feedback, field) if field else 'navigating'
        paused = stage.startswith('paused:')
        self._update(row['id'], 'paused' if paused else 'running', stage, code='executor_paused' if paused else None)

    def _status(self, message, capabilities):
        for capability in capabilities:
            self._observed[capability] = {
                bytes(status.goal_info.goal_id.uuid) for status in message.status_list
                if status.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)}
        for status in message.status_list:
            row = self._journal.goal(bytes(status.goal_info.goal_id.uuid))
            if row is not None and row['state'] not in TERMINAL and not row['navigation_stopped']:
                if status.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING):
                    self._live.add(row['id'])
                if status.status in (4, 5, 6):
                    self._terminal_seen.setdefault(row['id'], time.monotonic())
                self._lookup_result(row)

    def _lookup_result(self, row):
        key = ('result', row['id'])
        client = self._result_clients[row['capability']]
        if (key in self._pending or not client.service_is_ready()
                or time.monotonic() - self._queried.get(key, 0.0) < 1.0):
            return
        action = CAPABILITIES[row['capability']]['action'][1]
        request = action.Impl.GetResultService.Request(goal_id=UUID(uuid=list(row['goal_id'])))
        server_id = self._server_id(row['capability'])
        future = client.call_async(request)
        self._queried[key] = time.monotonic()
        self._pending[key] = future
        future.add_done_callback(lambda done: self._queue_reply('result', row, done, server_id))

    def _result(self, identifier, future, server_id=''):
        if future.cancelled():
            return
        key = ('result', identifier)
        if self._pending.get(key) is future:
            self._pending.pop(key)
        if self._row(identifier)['state'] in TERMINAL or self._row(identifier)['navigation_stopped']:
            return
        capability = self._row(identifier)['capability']
        spec = CAPABILITIES[capability]
        try:
            response = future.result()
            result = spec.get('result', dict)(dict(message_to_ordereddict(response.result)))
            json.dumps(result, allow_nan=False)
        except Exception as error:
            self.get_logger().error(f'task result unconfirmed: {error}')
            self._update(identifier, 'unknown', 'result transport failed; termination unconfirmed',
                         code='result_unconfirmed')
            return
        if response.status == GoalStatus.STATUS_UNKNOWN:
            self._recovering.add(identifier)
            if server_id and self._server_id(capability) == server_id:
                if self._unknown_results.get(identifier) != server_id:
                    self._unknown_since[identifier] = time.monotonic()
                self._unknown_results[identifier] = server_id
            self._update(identifier, 'unknown', 'ROS server has no result; delivery or termination remains unconfirmed',
                         code='result_unconfirmed')
            return
        states = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'failed'}
        if response.status not in states:
            self._update(identifier, 'unknown', 'nonterminal ROS result', code='result_unconfirmed')
            return
        state = states[response.status]
        if 'stopped' in spec and not spec['stopped'](result):
            # The runner ended but the arm may still be moving: not a stop, so nothing is released.
            detail = f'skill {state}, but the arm controller did not confirm the arm stopped'
            if self._row(identifier)['detail'] != detail:
                self._update(identifier, 'unknown', detail, {**result, 'arm_stopped': False}, 'arm_stop_unconfirmed')
            return
        if state == 'succeeded' and (result.get('complete') is False or result.get('error_code', 0)):
            state = 'failed'
        detail = result.get('message') or result.get('error_msg') or result.get('detail') or state
        code = reason_code(capability, state, result)
        if self._row(identifier)['cancel_requested'] == 2:
            state, detail, code = 'failed', f'navigation deadline reached; ROS terminal state: {state}', 'budget_exhausted'
        row = self._row(identifier)
        if row['capability'] == 'navigate' and identifier in self._pausing and not row['cancel_requested']:
            if response.status == GoalStatus.STATUS_SUCCEEDED and not result.get('error_code', 0):
                self._update(identifier, state, detail, result, code)
            else:
                self._journal.navigation_stopped(identifier)
                self._release_requests(identifier, row)
                self._update(identifier, 'paused', 'paused: navigation stopped; waiting for safety recovery',
                             code='navigation_stopped')
            return
        self._update(identifier, state, detail, result, code)

    def _delivery_failed(self, identifier, error):
        self.get_logger().error(f'task acceptance unconfirmed: {error}')
        self._recovering.add(identifier)
        self._update(identifier, 'unknown', 'action acceptance unconfirmed; not resubmitted',
                     code='acceptance_unconfirmed')

    def _accepted(self, identifier, future):
        self._pending.pop(('send', identifier), None)
        if self._row(identifier)['state'] in TERMINAL:
            return
        try:
            accepted = future.result().accepted
        except Exception as error:
            self._delivery_failed(identifier, error)
            return
        if not accepted:
            # A rejected goal never executed, so it left nothing to compensate.
            self._update(identifier, 'failed', 'ROS action server rejected the goal', {'goal_rejected': True},
                         'goal_rejected')
            return
        row = self._row(identifier)
        if not row['cancel_requested'] and identifier not in self._pausing:
            self._update(identifier, 'running', 'ROS action accepted')
        self._lookup_result(self._row(identifier))

    def _canceled(self, identifier, future, server_id=''):
        if future.cancelled():
            return
        key = ('cancel', identifier)
        if self._pending.get(key) is future:
            self._pending.pop(key)
        if self._row(identifier)['state'] in TERMINAL:
            return
        try:
            reply = future.result()
            if (server_id and reply.return_code == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID
                    and self._server_id(self._row(identifier)['capability']) == server_id):
                self._unknown_cancels[identifier] = server_id
            if reply.return_code:
                self.get_logger().warning(f'task cancellation returned {reply.return_code}; terminal result still required', throttle_duration_sec=5)
        except Exception as error:
            self.get_logger().error(f'task cancellation unconfirmed: {error}')

    def _dispatch(self, row, now):
        identifier, capability = row['id'], row['capability']
        goal_id = UUID(uuid=list(row['goal_id']))
        if not self._enabled[capability]:
            self._update(identifier, 'failed', 'capability disabled by this deployment profile before submission',
                         code='capability_disabled')
            return
        if any(self._observed.values()) or self._children.busy():
            return
        if capability == 'navigate':
            self._owner.publish(goal_id)
            if not (self._guard[0] and now - self._guard[1] < 1.0
                    and self._ack[0] == row['goal_id'] and now - self._ack[1] < 1.0):
                return
        client = self._action_clients[capability]
        if not client.server_is_ready():
            return
        parameters = row['parameters']
        # What admission resolved is what runs: captured limits, a target's pose, a named state's joints.
        effective = json.loads(row['result']).get('_task', {}).get('effective_parameters')
        if effective is not None:
            declared = CAPABILITIES[capability]['schema']['properties']
            parameters = json.dumps({name: value for name, value in effective.items() if name in declared},
                                    allow_nan=False)
        goal, _ = task_goal(capability, parameters)
        if capability == 'navigate':
            cause = self._navigation.cause(now)
            if cause:
                self._update(identifier, 'paused', f'paused: {cause[1]}', code=cause[0])
                return
            if row['remaining_sec'] is None:
                self._journal.navigation_budget(identifier, self._timeout)
        server_id = self._server_id(capability)
        if not server_id:
            return
        self._journal.mark_dispatched(identifier, server_id)
        self._sent.add(identifier)
        self._started[identifier] = now
        admitted = self._admitted_at.pop(identifier, None)
        if admitted is not None:
            self._budget.record('queue', now - admitted)
        self._dispatched_at.setdefault(identifier, now)
        try:
            future = client.send_goal_async(goal, goal_uuid=goal_id)
        except Exception as error:
            self._delivery_failed(identifier, error)
            return
        self._pending[('send', identifier)] = future
        future.add_done_callback(lambda done: self._queue_reply('accepted', row, done))

    def _tick(self):
        while True:
            try:
                reply = self._replies.get_nowait()
            except Empty:
                break
            kind, identifier, goal_id, server_id, future = reply
            if self._row(identifier)['goal_id'] != goal_id:
                # A reply for a superseded attempt is recorded as evidence, never applied as state.
                self._journal.event(identifier, 'late_reply', {'kind': kind, 'goal_id': bytes(goal_id).hex()})
                continue
            if kind == 'accepted':
                self._accepted(identifier, future)
            else:
                {'result': self._result, 'canceled': self._canceled}[kind](identifier, future, server_id)
        now = time.monotonic()
        for row in self._active():
            identifier, capability = row['id'], row['capability']
            goal_id = UUID(uuid=list(row['goal_id']))
            if capability == 'navigate' and not row['cancel_requested']:
                metadata = json.loads(row['result']).get('_task', {})
                limits = metadata.get('parameters', {})
                expired = time.time() >= limits.get('deadline_unix_sec', math.inf)
                origin = metadata.get('odometry_origin')
                if origin:
                    meter = self._distance.status()
                    expired = expired or not meter['ready'] or meter['epoch'] != origin['epoch'] or (
                        meter['meters'] - origin['meters'] >= max(0, limits['max_travel_m'] - .5))
                if expired:
                    self._journal.request_cancel(identifier, reason=2)
                    self._update(identifier, 'canceling', 'bounded navigation budget or telemetry lost; awaiting terminal ROS result')
                    row = self._row(identifier)
            if self._reconcile(row, now):
                continue
            # Result calls wait through execution; only expire after observed termination.
            for kind in ('cancel', 'result'):
                key = (kind, identifier)
                since = self._queried.get(key, now) if kind == 'cancel' else self._terminal_seen.get(identifier, now)
                if key in self._pending and now - since >= 3.0:
                    future = self._pending.pop(key)
                    client = self._cancel_clients[capability] if kind == 'cancel' else self._result_clients[capability]
                    client.remove_pending_request(future)
                    future.cancel()
                    if kind == 'result':
                        self._terminal_seen[identifier] = now
            if identifier in self._terminal_seen:
                self._lookup_result(row)
            if row['navigation_stopped']:
                if row['cancel_requested']:
                    self._update(identifier, 'canceled', 'canceled after confirmed navigation stop',
                                 code='budget_exhausted' if row['cancel_requested'] == 2 else 'canceled')
                elif not self._navigation.cause(now) and not any(self._observed.values()) and not self._children.busy():
                    self._release_requests(identifier, row)
                    self._journal.resume_navigation(identifier, bytes(owned_goal_id().uuid))
                    self._dispatch(self._row(identifier), now)
                continue
            if identifier in self._recovering:
                # Never revive a potentially expired Nav2 ownership lease after a crash.
                self._lookup_result(row)
            elif identifier not in self._sent:
                if row['cancel_requested']:
                    self._update(identifier, 'canceled', 'canceled before action submission',
                                 code='budget_exhausted' if row['cancel_requested'] == 2 else 'canceled_before_dispatch')
                    continue
                self._dispatch(row, now)
            elif capability == 'navigate' and not row['cancel_requested']:
                self._navigation_tick(row, now)
            row = self._row(identifier)
            if row['state'] in TERMINAL:
                continue
            if row['cancel_requested'] or identifier in self._pausing or identifier in self._unknown_results:
                client = self._cancel_clients[capability]
                key = ('cancel', identifier)
                if (key not in self._pending and client.service_is_ready()
                        and now - self._queried.get(key, 0.0) >= 1.0):
                    request = CancelGoal.Request()
                    request.goal_info.goal_id = goal_id
                    server_id = self._server_id(capability)
                    future = client.call_async(request)
                    self._queried[key] = now
                    self._pending[key] = future
                    future.add_done_callback(lambda done, task=row, server=server_id: self._queue_reply('canceled', task, done, server))
                self._lookup_result(row)

    def _reconcile(self, row, now):
        identifier, capability = row['id'], row['capability']
        original = row['server_id']
        current = self._unknown_results.get(identifier, '')
        if not original or not current or self._unknown_cancels.get(identifier) != current:
            return False
        replaced = original != current
        # The server that was sent the goal denies it. Once the sending process is gone nothing of its
        # is in transit, so the goal ended or never arrived; a goal this process sent may still arrive.
        # Navigation is released only by its owner guard's lease, which needs the replacement, and a
        # skill whose arm stop was never confirmed did not end by leaving that server's memory.
        if not replaced and (capability == 'navigate' or identifier in self._sent
                             or json.loads(row['result']).get('arm_stopped') is False):
            return False
        # Let the independent guard expire an orphan's lease before clearing uncertainty.
        if now - self._unknown_since.get(identifier, now) < 3.0:
            return False
        if self._server_id(capability) != current:
            return False
        # The replacement server denied this goal; its old latched status is obsolete.
        self._observed[capability].discard(row['goal_id'])
        if any(self._observed.values()) or self._children.busy():
            return False
        if not self._guard[0] or now - self._guard[1] >= 1.0:
            return False
        detail = ('action server restarted; goal absent and owned navigation stopped; outcome unavailable' if replaced
                  else 'action server has no such goal after a gateway restart; outcome unavailable')
        if row['cancel_requested'] == 2:
            detail = f'navigation deadline reached; {detail}'
        self._update(identifier, 'failed', detail, {'outcome_known': False, 'termination_confirmed': True},
                     'outcome_unknown')
        return True

    def _server_id(self, capability):
        return server_instance(self, CAPABILITIES[capability]['action'][0])

    def _queue_reply(self, kind, row, future, server_id=''):
        self._replies.put((kind, row['id'], row['goal_id'], server_id, future))

    def _navigation_tick(self, row, now):
        identifier = row['id']
        cause = self._navigation.cause(now)
        previous = self._started.get(identifier, now)
        self._started[identifier] = now
        remaining = row['remaining_sec']
        if not cause and identifier not in self._pausing:
            remaining -= max(0.0, now - previous)
            self._journal.navigation_budget(identifier, remaining)
        if remaining <= 0:
            self._journal.request_cancel(identifier, reason=2)
            self._update(identifier, 'canceling', 'navigation deadline reached; awaiting terminal ROS result')
            return
        if identifier in self._pausing and now - self._pause_started[identifier] >= 3.0:
            self._recovering.add(identifier)
            self._update(identifier, 'unknown', 'navigation pause stop unconfirmed; ownership lease no longer renewed',
                         code='pause_stop_unconfirmed')
            return
        self._owner.publish(UUID(uuid=list(row['goal_id'])))
        if cause or identifier in self._pausing:
            if identifier not in self._pausing:
                self._journal.request_navigation_pause(identifier)
            self._pause_started.setdefault(identifier, now)
            self._pausing.add(identifier)
            code, text = cause or ('awaiting_navigation_stop', 'awaiting confirmed navigation stop')
            self._update(identifier, 'paused', f'paused: {text}', code=code)

    def _gripper_model(self):
        """The claw's named positions and limits as MoveIt holds them: the SRDF and joint limits, calibrated."""
        states, limits = dict(GRIPPER_STATES), JOINT_LIMITS[GRIPPER_JOINT]
        calibration = Path(self.get_parameter('arm_calibration_file').value).expanduser()
        if not calibration.is_file():
            return states, limits
        parameters = {'robot_description_semantic': SRDF.read_text(), 'robot_description_planning': {
            'joint_limits': {GRIPPER_JOINT: dict(zip(('min_position', 'max_position'), limits))}}}
        try:
            # Only a calibrated physical arm needs MoveIt's configuration tools; without them or with
            # a calibration MoveIt would reject, the claw is unavailable and nothing else is affected.
            from lekiwi_rmf.moveit_config import apply_gripper_calibration
            apply_gripper_calibration(parameters, calibration)
        except (ImportError, ValueError) as error:
            self.get_logger().error(f'gripper unavailable: {error}')
            self._enabled['gripper'] = False
            return states, limits
        configured = parameters['robot_description_planning']['joint_limits'][GRIPPER_JOINT]
        return ({name: joints[GRIPPER_JOINT] for name, joints in
                 group_states(parameters['robot_description_semantic'])[GRIPPER_GROUP].items()},
                (configured['min_position'], configured['max_position']))

    def _resolver_setup(self):
        """A second node, spun only while a navigation target is being resolved.

        A service callback cannot wait on this node's own executor, and the map, the transforms and
        the spatial query are needed only then, so between requests they cost nothing.
        """
        self._resolver = rclpy.create_node('robot_tasks_resolver', context=self.context,
                                           use_global_arguments=False, start_parameter_services=False)
        self._resolver_executor = SingleThreadedExecutor(context=self.context)
        self._resolver_executor.add_node(self._resolver)
        self._spatial = self._resolver.create_client(QuerySpatial, SPATIAL_SERVICE)
        self._objects = self._resolver.create_client(QuerySpatial, OBJECTS_SERVICE)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self._resolver)
        self._map = None
        self._resolver.create_subscription(OccupancyGrid, '/map', lambda message: setattr(self, '_map', message),
                                           QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # A standoff pose is admitted with the explorer's own tracked clearance and the footprint Nav2 runs.
        explorer = yaml.safe_load((config_directory() / 'exploration.yaml').read_text())['robot_explorer']['ros__parameters']
        self._standoff = {name: explorer[name] for name in ('clearance_m', 'free_threshold', 'max_map_cells',
                                                           'data_timeout_sec')}
        try:
            self._footprint = load_navigation_footprint(
                Path(self.get_parameter('navigation_params_file').value).expanduser())[0]
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            self.get_logger().error(f'point targets unavailable: {error}')
            self._footprint = None

    def _resolve(self, ready, wait_sec=1.0):
        """Spin the resolver until `ready()` gives a value; None when the wait ends first."""
        deadline = time.monotonic() + wait_sec
        while True:
            value = ready()
            if value is not None or time.monotonic() >= deadline:
                return value
            self._resolver_executor.spin_once(timeout_sec=0.05)

    def target_kinds(self):
        point = self._footprint is not None and self._resolver.count_publishers('/map') > 0
        return {'pose': {'available': True},
                'anchored': {'available': self._spatial.service_is_ready(), 'service': SPATIAL_SERVICE},
                'point': {'available': point, 'map': '/map'},
                'object': {'available': point and self._objects.service_is_ready(), 'service': OBJECTS_SERVICE,
                           'map': '/map', 'max_sigma_ratio': self._sigma_ratio}}

    def resolve_target(self, target):
        """The map pose a navigation target stands for now, and how it was found; one method per kind."""
        kind = next(name for name in ('pose', 'anchored', 'point', 'object') if name in target)
        x, y, yaw, resolved = getattr(self, f'_target_{kind}')(target)
        return {'x': x, 'y': y, 'yaw': yaw, 'resolved': {'target': kind, **resolved}}

    @staticmethod
    def _target_pose(target):
        pose = target['pose']
        return pose['x'], pose['y'], pose.get('yaw', 0.0), {}

    def _query(self, client, request, name, condition, field):
        """One bounded JSON query of a spatial-memory service; refused when it does not answer in time."""
        unavailable = Refused('precondition_failed', f'{name} is not answering', retryable=True,
                              next_condition=condition, field=field)
        if not client.service_is_ready():
            raise unavailable
        future = client.call_async(QuerySpatial.Request(request_json=json.dumps(request, allow_nan=False)))
        reply = self._resolve(lambda: future.result() if future.done() else None)
        if reply is None:
            client.remove_pending_request(future)
            raise unavailable
        if not reply.success:
            raise Refused('precondition_failed', f'{name} refused the query: {reply.detail}', field=field)
        return json.loads(reply.response_json)

    def _spatial_query(self, request):
        return self._query(self._spatial, request, 'spatial memory', 'wait_spatial_memory', 'target.anchored')

    def _target_anchored(self, target):
        """A pose remembered under an earlier map revision, carried by its anchor into the current one."""
        anchored = target['anchored']
        current = self._spatial_query({'op': 'current'})
        if not current.get('ready'):
            raise Refused('precondition_failed', f'no current map revision: {current.get("reason")}', retryable=True,
                          next_condition='needs_map', field='target.anchored')
        pose, revision = anchored['map_pose'], current['revision']
        yaw = pose.get('yaw', 0.0)
        capture = {**{name: anchored[name] for name in ('site_ref', 'map_ref', 'revision')},
                   'pose': {'t': [pose['x'], pose['y'], 0.0], 'q': [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]}}
        # Naming the current revision as the target refuses a pose from a map the robot is not in.
        result = self._spatial_query({'op': 'project', 'items': [{'id': 'target', 'map_pose': capture}],
                                      'target': {name: revision[name] for name in ('map_ref', 'revision')}})['items'][0]
        if not result['ok']:
            code = result['code'] if result['code'] in ('site_mismatch', 'map_unaligned', 'revision_unknown') \
                else 'precondition_failed'
            raise Refused(code, f'the anchored pose has no place in the current map: {result["detail"]}',
                          next_condition='refresh_reference', field='target.anchored', spatial_code=result['code'])
        (x, y, _), (qx, qy, qz, qw) = result['camera_pose']['t'], result['camera_pose']['q']
        return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)), {
            name: result[name] for name in ('site_ref', 'map_ref', 'revision', 'capture_revision', 'method', 'shift_m')}

    def _robot_position(self):
        try:
            transform = self._tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
        t = transform.transform.translation
        if not -0.3 <= age <= self._standoff['data_timeout_sec'] or not (math.isfinite(t.x) and math.isfinite(t.y)):
            return None
        return t.x, t.y

    def _target_point(self, target):
        return self._approach((target['point']['x'], target['point']['y']),
                              target.get('standoff_m', DEFAULT_STANDOFF_M), 'target.point', 'target.standoff_m')

    def _target_object(self, target):
        """Near an object-map entry and facing it: the point target of its last observed estimate."""
        identifier = target['object']['object_id']
        standoff = target['object'].get('standoff_m', DEFAULT_STANDOFF_M)

        def refused(code, detail, condition, retryable=False, field='target.object', **data):
            return Refused(code, detail, retryable=retryable, next_condition=condition, field=field, **data)

        answer = self._query(self._objects, {'object_id': identifier, 'limit': 1}, 'the object map',
                             'wait_object_map', 'target.object')
        try:
            if answer['schema'] != OBJECT_MAP_SCHEMA:
                raise refused('precondition_failed', f'the object map answers {answer["schema"]}, not '
                              f'{OBJECT_MAP_SCHEMA}', 'fix_deployment')
            entry = next((item for item in answer['objects'] if item['object_id'] == identifier), None)
            if entry is None:
                raise refused('not_found', 'the object map has no entry with this object_id', 'query_objects',
                              field='target.object.object_id')
            state, position, sigma = entry['state'], entry['position'], entry['sigma_m']
            if state == 'absent_at_last_place':
                raise refused('precondition_failed', 'the object was not detected in views that should have shown '
                              'it at its last place', 'run_search', retryable=True, object_state=state)
            if position is None and 'position_unavailable_reason' in entry:
                code = entry['position_unavailable_reason']
                raise refused(code if code in ('site_mismatch', 'map_unaligned', 'revision_unknown')
                              else 'precondition_failed', f'the object has no place in the current map: {code}',
                              'refresh_reference', spatial_code=code)
            if position is None:
                raise refused('precondition_failed', f'the object is known only as a direction ({entry["method"]}), '
                              'not as a point', 'run_search', retryable=True, method=entry['method'])
            if not answer['localized']:
                raise refused('precondition_failed', 'the robot is not localized in a known map', 'needs_localization',
                              retryable=True)
            if entry['map_ref'] != answer['map']['map_ref']:
                raise refused('map_unaligned', 'the object is in a map that has no alignment with the one the '
                              'robot is in', 'refresh_reference')
            point = (position['x'], position['y'])
            if any(type(value) not in (float, int) or not math.isfinite(value) for value in (*point, sigma)):
                raise TypeError('position and sigma_m must be finite numbers')
            # The pose stands standoff_m from the estimate and faces it. With the object one sigma_m away
            # from the estimate, the tracked 0.5 keeps it at least half the standoff from the base and
            # within 30 degrees of its heading; beyond that the robot may stop on the object or beside it.
            limit = self._sigma_ratio * standoff
            if sigma > limit:
                wider = sigma <= self._sigma_ratio * MAX_STANDOFF_M
                raise refused('precondition_failed', f'the object estimate is uncertain by {sigma} m, more than '
                              f'{limit:.3f} m ({self._sigma_ratio} of the standoff)',
                              'change_standoff' if wider else 'run_search', retryable=not wider,
                              field='target.object.standoff_m' if wider else 'target.object',
                              sigma_m=sigma, max_sigma_m=limit)
            resolved = {name: entry[name] for name in ('object_id', 'label', 'state', 'method', 'sigma_m', 'qualified',
                                                       'last_seen_unix_sec', 'map_ref', 'revision')}
            resolved['verification'] = answer['verification']
        except (KeyError, TypeError) as error:
            raise refused('precondition_failed', f'the object map answer is malformed: {error!r}',
                          'fix_deployment') from error
        x, y, yaw, approach = self._approach(point, standoff, 'target.object', 'target.object.standoff_m')
        return x, y, yaw, {**approach, **resolved}

    def _approach(self, point, standoff, field, standoff_field):
        """Near a map point and facing it: a clear, known standoff pose, the one nearest the robot."""
        if self._footprint is None:
            raise Refused('capability_unavailable', 'point targets need the Nav2 footprint of this deployment',
                          next_condition='fix_deployment', field=field)
        # Take what queued up since the last request first: the newest map and transforms.
        self._resolve(lambda: None, wait_sec=0.1)
        robot = self._resolve(self._robot_position)
        if robot is None:
            raise Refused('precondition_failed', 'the robot has no fresh pose in the map', retryable=True,
                          next_condition='needs_localization', field=field)
        try:
            grid = map_geometry(self._resolve(lambda: self._map), self._standoff['max_map_cells'])
        except (AttributeError, ValueError) as error:
            raise Refused('precondition_failed', f'no usable occupancy map: {error}', retryable=True,
                          next_condition='needs_map', field=field) from error
        try:
            x, y, yaw = approach_pose(*grid, point, standoff, robot, clearance=self._standoff['clearance_m'],
                                      footprint=self._footprint, free_threshold=self._standoff['free_threshold'])
        except NoStandoff as error:
            raise Refused('precondition_failed', f'no standoff pose {standoff} m from the point: {error}',
                          retryable=error.unmapped, next_condition='needs_map' if error.unmapped else 'change_standoff',
                          field=field if error.unmapped else standoff_field) from error
        return x, y, yaw, {'point': list(point), 'standoff_m': standoff, 'robot': list(robot)}

    def destroy_node(self):
        for client in set(self._action_clients.values()):
            client.destroy()
        self._resolver_executor.shutdown()
        self._resolver.destroy_node()
        self._journal.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaskGateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
