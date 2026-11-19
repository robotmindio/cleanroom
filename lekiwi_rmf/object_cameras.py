"""Tracked camera bindings shared by launch, inference and the observation collector."""

from pathlib import Path
import re
import json
import hashlib
import math

import yaml

QUALITY_LIMITS = ('dark_luma', 'bright_luma', 'blur_variance')


def config_directory():
    source = Path(__file__).resolve().parents[1] / 'config'
    if source.is_dir():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('lekiwi_rmf')) / 'config'


def quality_limits(value):
    """A camera's input-quality limits: mean luma below `dark_luma` or above `bright_luma` (0..255) and
    Laplacian variance below `blur_variance` flag a frame. `calibration` names the record that fitted them."""
    if (not isinstance(value, dict) or set(value) - {'calibration'} != set(QUALITY_LIMITS)
            or any(type(value[name]) not in (int, float) or not math.isfinite(value[name]) for name in QUALITY_LIMITS)
            or not 0 <= value['dark_luma'] < value['bright_luma'] <= 255 or value['blur_variance'] < 0
            or not isinstance(value.get('calibration', ''), str)):
        raise ValueError('camera input_quality needs 0 <= dark_luma < bright_luma <= 255 and blur_variance >= 0')
    return {name: float(value[name]) for name in QUALITY_LIMITS}


def camera_bindings(names, path=None):
    if (not isinstance(names, (list, tuple)) or not names or len(names) > 3
            or any(not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', name) for name in names)
            or len(set(names)) != len(names)):
        raise ValueError('select 1..3 distinct camera identities')
    path = Path(path) if path else config_directory() / 'object_cameras.yaml'
    if path.stat().st_size > 16384:
        raise ValueError('camera bindings exceed 16 KiB')
    configured = yaml.safe_load(path.read_text())
    required = {'image_topic', 'optical_frame', 'camera_info_topic', 'depth_topic', 'calibration_file', 'mount'}
    result = {}
    for name in names:
        value = configured.get(name) if isinstance(configured, dict) else None
        if not isinstance(value, dict) or set(value) != required | {'input_quality'}:
            raise ValueError(f'invalid tracked camera binding: {name}')
        # The limits tune abstention reasons, not the physical source, so camera_sha256 does not cover them.
        limits = quality_limits(value['input_quality'])
        value = {key: value[key] for key in required}
        if any(not isinstance(item, str) for item in value.values()):
            raise ValueError('camera binding values must be strings')
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,127}', value['optical_frame']):
            raise ValueError('invalid camera optical frame')
        for field in ['image_topic', 'camera_info_topic', 'depth_topic']:
            if not value[field] and field == 'depth_topic':
                continue
            if not re.fullmatch(r'/[A-Za-z0-9_/]{1,255}', value[field]):
                raise ValueError(f'invalid camera {field}')
        if value['mount'] not in ('base', 'arm'):
            raise ValueError('camera mount must be base or arm')
        calibration = value['calibration_file']
        if calibration:
            relative = Path(calibration)
            if relative.is_absolute() or '..' in relative.parts or not value['depth_topic']:
                raise ValueError('metric calibration requires registered depth and a tracked relative file')
            calibration = str(path.parent / relative)
        calibration_hash = hashlib.sha256(Path(calibration).read_bytes()).hexdigest() if calibration else ''
        identity = {**value, 'camera_id': name, 'calibration_file': calibration_hash}
        result[name] = {**value, 'calibration_file': calibration, 'calibration_sha256': calibration_hash,
                        'sha256': hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest(),
                        'prefix': '/robot/perception/' + name, 'input_quality': limits}
    if len({v['image_topic'] for v in result.values()}) != len(result):
        raise ValueError('camera identities cannot alias the same image source')
    if len({v['optical_frame'] for v in result.values()}) != len(result):
        raise ValueError('camera identities require distinct optical frames')
    return result
