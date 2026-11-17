"""Conservative planar support of the physical CAD throughout the stow tolerance.

This is an offline configuration generator, never a runtime obstacle filter.
Every physical visual mesh participates; virtual MoveIt keepouts do not.
"""

import hashlib
import itertools
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np


MODEL_ALLOWANCE_M = 0.005
NORMAL_COUNT = 16
STL = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


def origin(element):
    matrix = np.eye(4)
    if element is not None:
        roll, pitch, yaw = map(float, element.get('rpy', '0 0 0').split())
        matrix[:3, :3] = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
        matrix[:3, 3] = list(map(float, element.get('xyz', '0 0 0').split()))
    return matrix


def placements(robot, positions):
    parents = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    world, axes = {'base_footprint': np.eye(4)}, {}

    def place(name):
        if name not in world:
            joint = parents[name]
            matrix = place(joint.find('parent').get('link')) @ origin(joint.find('origin'))
            if joint.get('type') in ('revolute', 'continuous'):
                axis_node = joint.find('axis')
                axis = np.fromstring(axis_node.get('xyz', '1 0 0') if axis_node is not None else '1 0 0', sep=' ')
                axes[joint.get('name')] = (matrix[:3, 3].copy(), matrix[:3, :3] @ axis)
                motion = np.eye(4)
                motion[:3, :3] = rotation(axis, positions[joint.get('name')])
                matrix = matrix @ motion
            elif joint.get('type') != 'fixed':
                raise ValueError(f'unsupported physical joint: {joint.get("name")}')
            world[name] = matrix
        return world[name]

    for link in robot.findall('link'):
        place(link.get('name'))
    return world, axes, parents


def physical_geometry(root, robot):
    cad = ET.parse(root / 'urdf/lekiwi_cad.urdf').getroot()
    physical = {link.get('name') for link in cad.findall('link')} | {'astra_camera_link'}
    geometry, sources = [], {}
    for link in robot.findall('link'):
        if link.get('name') not in physical:
            continue
        for visual in link.findall('visual'):
            shape = visual.find('geometry')
            mesh, box = shape.find('mesh'), shape.find('box')
            if mesh is not None:
                filename = mesh.get('filename')
                if not filename.startswith('package://lekiwi_rmf/'):
                    raise ValueError(f'physical mesh outside repository: {filename}')
                path = root / filename.removeprefix('package://lekiwi_rmf/')
                data = path.read_bytes()
                if len(data) < 84 or len(data) != 84 + int.from_bytes(data[80:84], 'little') * STL.itemsize:
                    raise ValueError(f'expected binary STL: {path}')
                points = np.frombuffer(data, offset=84, dtype=STL)['vertices'].reshape(-1, 3).astype(float)
                points *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                sources[str(path.relative_to(root))] = hashlib.sha256(data).hexdigest()
            elif box is not None:
                points = np.array(list(itertools.product((-0.5, 0.5), repeat=3)))
                points *= np.fromstring(box.get('size'), sep=' ')
            else:
                raise ValueError(f'unsupported physical geometry on {link.get("name")}')
            matrix = origin(visual.find('origin'))
            geometry.append((link.get('name'), points @ matrix[:3, :3].T + matrix[:3, 3]))
    return geometry, sources


def cad_footprint(root, positions, tolerance):
    root = Path(root)
    description = subprocess.check_output(['xacro', str(root / 'urdf/lekiwi.urdf.xacro'), 'sim:=false'], text=True)
    robot = ET.fromstring(description)
    geometry, sources = physical_geometry(root, robot)
    world, axes, parents = placements(robot, positions)
    normals = np.column_stack((np.cos(np.arange(NORMAL_COUNT) * 2 * math.pi / NORMAL_COUNT),
                               np.sin(np.arange(NORMAL_COUNT) * 2 * math.pi / NORMAL_COUNT)))
    supports = np.full(NORMAL_COUNT, -np.inf)
    allowances = {}
    for name, local in geometry:
        matrix = world[name]
        points = local @ matrix[:3, :3].T + matrix[:3, 3]
        displacement = 0.0
        frame = name
        while frame != 'base_footprint':
            joint = parents[frame]
            if joint.get('name') in axes:
                center, axis = axes[joint.get('name')]
                radius = np.linalg.norm(np.cross(points - center, axis / np.linalg.norm(axis)), axis=1).max()
                # Telescoping the serial rotations bounds every intermediate
                # angle, not just sampled corners of the joint-tolerance box.
                displacement += 2 * math.sin(tolerance / 2) * radius
            frame = joint.find('parent').get('link')
        allowances[name] = max(allowances.get(name, 0.0), float(displacement))
        supports = np.maximum(supports, (points[:, :2] @ normals.T).max(axis=0) + displacement + MODEL_ALLOWANCE_M)
    # Round supporting lines outwards; never approximate by an inscribed hull.
    supports = np.ceil(supports * 1000) / 1000
    vertices = []
    for i, j in itertools.combinations(range(NORMAL_COUNT), 2):
        matrix = normals[[i, j]]
        if abs(np.linalg.det(matrix)) < 1e-8:
            continue
        point = np.linalg.solve(matrix, supports[[i, j]])
        if np.all(normals @ point <= supports + 1e-9):
            vertices.append(point)
    vertices = np.unique(np.round(vertices, 12), axis=0)
    vertices = vertices[np.argsort(np.arctan2(vertices[:, 1], vertices[:, 0]))]
    for filename in ('urdf/lekiwi_cad.urdf', 'urdf/lekiwi.urdf.xacro', 'config/safety_production.yaml'):
        sources[filename] = hashlib.sha256((root / filename).read_bytes()).hexdigest()
    return {'schema': 1, 'frame': 'base_footprint', 'joint_positions_rad': positions,
            'joint_tolerance_rad': tolerance, 'model_allowance_m': MODEL_ALLOWANCE_M,
            'normals': normals.tolist(), 'support_distances_m': supports.tolist(),
            'footprint': vertices.tolist(), 'inscribed_radius_m': float(supports.min()),
            'maximum_radius_m': float(np.linalg.norm(vertices, axis=1).max()),
            'link_motion_bounds_m': allowances, 'source_sha256': dict(sorted(sources.items()))}
