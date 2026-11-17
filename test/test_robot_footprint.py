"""Bind production collision geometry to every physical CAD part and stow range."""

import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from lekiwi_rmf.geometry import point_in_polygon
from lekiwi_rmf.robot_footprint import cad_footprint, physical_geometry, placements
from lekiwi_rmf.safety_acceptance import validate_tracked_acceptance


ROOT = Path(__file__).parents[1]


@pytest.fixture(scope='module')
def envelope():
    return json.loads((ROOT / 'docs/evidence/navigation-footprint-20261117.json').read_text())


def test_production_envelope_is_reproducible_from_unchanged_cad_and_stow(envelope):
    config = yaml.safe_load((ROOT / 'config/safety_production.yaml').read_text())['safety_supervisor']['ros__parameters']
    assert config['stow_tolerance'] == .02
    assert envelope['model_allowance_m'] == .005
    pose = dict(zip(config['stow_joint_names'], config['stow_joint_positions']))
    assert cad_footprint(ROOT, pose, config['stow_tolerance']) == envelope
    valid, reason = validate_tracked_acceptance(ROOT / 'config')
    assert valid, reason
    params = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    for costmap in ('local_costmap', 'global_costmap'):
        config = params[costmap][costmap]['ros__parameters']
        assert yaml.safe_load(config['footprint']) == envelope['footprint']
        assert config['footprint_padding'] == 0
    monitor = params['collision_monitor']['ros__parameters']
    assert monitor['polygons'] == ['StopZone', 'FootprintApproach']
    for name in monitor['polygons']:
        assert yaml.safe_load(monitor[name]['points']) == envelope['footprint']
        assert monitor[name]['enabled'] and monitor[name]['min_points'] == 1


def test_every_physical_part_fits_at_interior_stow_angles_too(envelope):
    robot = ET.fromstring(subprocess.check_output(['xacro', str(ROOT / 'urdf/lekiwi.urdf.xacro')], text=True))
    geometry, _ = physical_geometry(ROOT, robot)
    normals, supports = np.array(envelope['normals']), np.array(envelope['support_distances_m'])
    rng = np.random.default_rng(17)
    for _ in range(3):
        pose = {name: value + rng.uniform(-.02, .02) for name, value in envelope['joint_positions_rad'].items()}
        world, _, _ = placements(robot, pose)
        for name, local in geometry:
            matrix = world[name]
            points = local @ matrix[:3, :3].T + matrix[:3, 3]
            assert np.all(points[:, :2] @ normals.T <= supports - .005 + 1e-9), name


def test_rectangle_corner_is_external_space_and_does_not_become_free_sensor_data(envelope):
    from sensor_msgs.msg import LaserScan
    from lekiwi_rmf.scan_self_filter import blank_body_sectors, parse_sectors

    params = yaml.safe_load((ROOT / 'config/lidar_self_mask.yaml').read_text())['scan_self_filter']['ros__parameters']
    sectors = parse_sectors(*(params[name] for name in ('body_start_deg', 'body_end_deg', 'body_max_range_m')))
    for angle, distance in [(294.97798, .399), (298.14979, .356), (300.52864, .422)]:
        scan = LaserScan()
        scan.angle_min = float(np.radians(angle))
        scan.angle_increment, scan.range_min, scan.range_max = .01, .03, 12.
        scan.ranges = [distance]
        result = blank_body_sectors(scan, sectors)
        assert result.ranges[0] == pytest.approx(distance)
        radians = np.radians(angle)
        point = (-.135 - distance * np.sin(radians), .005 + distance * np.cos(radians))
        assert not point_in_polygon(point, envelope['footprint'])
    assert point_in_polygon((.10, .0), envelope['footprint'])
