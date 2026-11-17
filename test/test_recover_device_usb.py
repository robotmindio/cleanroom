"""Shared USB recovery cannot drop torque or leave USB power disabled on failure."""

from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    'recover_device_usb', Path(__file__).parents[1] / 'scripts/recover-device-usb.py')
usb = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(usb)


def proof():
    positions = dict(zip(usb.ARM, (.034, -1.715, 1.625, 1.189, -.014, .216)))
    return {
        'joints': {'age': .01, 'stamp_age': .01, 'baseline_stamp_age': .31,
                   'value': positions, 'baseline': positions.copy(), 'stable_seconds': .3,
                   'spread': dict.fromkeys(usb.ARM, .002)},
        'diagnostics': {'age': .02, 'stamp_age': .02, 'value': {
            name: {'level': 0, 'torque_enabled': 'False'}
            for name in ('motor_bus', *(f'servo/{n}' for n in usb.SERVOS))}},
        'driver': {'age': .01, 'value': 'DISARMED'},
        'arm_permitted': {'age': .01, 'value': False},
        'base_permitted': {'age': .01, 'value': False},
        'resources': {'age': .01, 'value': dict.fromkeys(usb.RESOURCES)},
    }


class Remote:
    def __init__(self, fail=None):
        self.calls, self.fail = [], fail

    def schedule_restore(self, hubs):
        self.calls.append(('timer', tuple(hubs)))

    def power(self, hub, action):
        self.calls.append((hub, action))
        if (hub, action) == self.fail:
            raise RuntimeError('connection failed')


HUBS = ['2', '3', '5', '4']


def run(probe, remote):
    usb.cycle_usb(probe, remote, HUBS, lambda event: None, pause=lambda seconds: None)


def test_supported_unpowered_pose_needs_no_nominal_stow_angle():
    # This is the supported operator-confirmed pose which originally deadlocked
    # recovery. It remains outside the separate powered-arm torque-cut tolerance.
    state = proof()
    assert abs(state['joints']['value']['arm_shoulder_lift'] - -1.7967) > .02
    remote = Remote()
    run(lambda: deepcopy(state), remote)
    assert remote.calls == [('timer', tuple(HUBS)),
                            *((h, 'off') for h in HUBS), *((h, 'on') for h in HUBS)]


@pytest.mark.parametrize('name', ('motor_bus', *(f'servo/{n}' for n in usb.SERVOS)))
@pytest.mark.parametrize('fault', ('powered', 'unknown', 'unhealthy', 'missing'))
def test_every_motor_requires_healthy_explicit_off_before_any_power_command(name, fault):
    state = proof()
    values = state['diagnostics']['value']
    if fault == 'missing':
        del values[name]
    elif fault == 'powered':
        values[name]['torque_enabled'] = 'True'
    elif fault == 'unknown':
        values[name]['torque_enabled'] = ''
    else:
        values[name]['level'] = 2
    remote = Remote()
    with pytest.raises(RuntimeError):
        run(lambda: state, remote)
    assert remote.calls == []


@pytest.mark.parametrize('key', ('joints', 'diagnostics', 'driver', 'arm_permitted',
                                 'base_permitted', 'resources'))
@pytest.mark.parametrize('age', (.501, -1., float('nan')))
def test_stale_or_invalid_receipt_never_authorizes_usb(key, age):
    state = proof()
    state[key]['age'] = age
    with pytest.raises(RuntimeError):
        usb.require_off(state)


@pytest.mark.parametrize('key', ('joints', 'diagnostics'))
@pytest.mark.parametrize('age', (.501, -.051, float('inf')))
def test_source_stamp_must_be_fresh(key, age):
    state = proof()
    state[key]['stamp_age'] = age
    with pytest.raises(RuntimeError):
        usb.require_off(state)


@pytest.mark.parametrize('fault', ('armed', 'arm_lease', 'base_lease', 'held', 'missing_resource',
                                  'moving', 'short_window', 'old_baseline', 'missing_joint',
                                  'nonfinite_joint', 'missing_spread'))
def test_ambiguous_or_active_robot_is_refused(fault):
    state = proof()
    if fault == 'armed':
        state['driver']['value'] = 'ARMED'
    elif fault.endswith('_lease'):
        state[fault.removesuffix('_lease') + '_permitted']['value'] = True
    elif fault == 'held':
        state['resources']['value']['base'] = {'state': 'unknown'}
    elif fault == 'missing_resource':
        del state['resources']['value']['arm']
    elif fault == 'moving':
        state['joints']['spread'][usb.ARM[0]] = .011
    elif fault == 'short_window':
        state['joints']['stable_seconds'] = .19
    elif fault == 'old_baseline':
        state['joints']['baseline_stamp_age'] = .6
    elif fault == 'missing_joint':
        del state['joints']['value'][usb.ARM[0]]
    elif fault == 'nonfinite_joint':
        state['joints']['baseline'][usb.ARM[0]] = float('nan')
    else:
        state['joints']['spread'] = {}
    with pytest.raises(RuntimeError):
        usb.require_off(state)


def test_changed_gate_between_off_commands_aborts_and_restores_every_hub():
    states = [proof(), proof(), proof()]
    states[-1]['diagnostics']['value']['motor_bus']['torque_enabled'] = 'True'
    remote = Remote()
    with pytest.raises(RuntimeError):
        run(lambda: states.pop(0), remote)
    assert remote.calls == [('timer', tuple(HUBS)), ('2', 'off'), *((h, 'on') for h in HUBS)]


def test_failed_off_still_restores_every_hub():
    remote = Remote(fail=('3', 'off'))
    with pytest.raises(RuntimeError):
        run(proof, remote)
    assert remote.calls[-4:] == [(h, 'on') for h in HUBS]
    assert ('5', 'off') not in remote.calls


def test_failed_on_does_not_skip_remaining_hubs_or_cancel_fallback_timer():
    remote = Remote(fail=('2', 'on'))
    with pytest.raises(RuntimeError, match='Pi-local timer remains'):
        run(proof, remote)
    assert remote.calls[-4:] == [(h, 'on') for h in HUBS]


def test_fallback_must_be_scheduled_before_any_off():
    remote = Remote()

    def fail(hubs):
        raise RuntimeError('timer unavailable')

    remote.schedule_restore = fail
    with pytest.raises(RuntimeError):
        run(proof, remote)
    assert remote.calls == []


def test_slow_guard_cannot_issue_off_after_restoration_window():
    remote = Remote()
    times = iter((0., 1., 11.))
    with pytest.raises(RuntimeError, match='OFF window expired'):
        usb.cycle_usb(proof, remote, HUBS, lambda event: None,
                      pause=lambda seconds: None, clock=lambda: next(times))
    assert remote.calls == [('timer', tuple(HUBS)), ('2', 'off'), *((h, 'on') for h in HUBS)]


def topology_files(tmp_path):
    root = tmp_path / 'devices'
    root.mkdir()
    model = tmp_path / 'model'
    model.write_text('Raspberry Pi 5 Model B Rev 1.1\0')
    for bus in range(2, 6):
        hub = tmp_path / 'xhci-hcd' / f'usb{bus}'
        hub.mkdir(parents=True)
        (hub / 'idVendor').write_text('1d6b\n')
        (hub / 'busnum').write_text(f'{bus}\n')
        (root / f'usb{bus}').symlink_to(hub)
    motor = root / '4-1'
    motor.mkdir()
    (motor / 'idVendor').write_text('1a86\n')
    (motor / 'idProduct').write_text('55d3\n')
    serial = root / '4-1:1.1'
    serial.mkdir()
    (serial / 'bInterfaceClass').write_text('0a\n')
    return root, model


def test_topology_keeps_motor_hub_last_and_accepts_its_cdc_serial_interface(tmp_path):
    root, model = topology_files(tmp_path)
    assert usb.usb_topology(root, model)['hubs'] == HUBS


@pytest.mark.parametrize('fault', ('storage', 'network', 'wrong_model', 'missing_motor', 'missing_hub'))
def test_topology_rejects_a_supply_cycle_that_would_drop_storage_or_network(tmp_path, fault):
    root, model = topology_files(tmp_path)
    if fault == 'storage':
        (root / '4-1:1.1' / 'bInterfaceClass').write_text('08\n')
    elif fault == 'network':
        (root / '4-1:1.1' / 'net').mkdir()
    elif fault == 'wrong_model':
        model.write_text('Raspberry Pi 4 Model B')
    elif fault == 'missing_motor':
        (root / '4-1' / 'idProduct').write_text('0000\n')
    else:
        (root / 'usb3').unlink()
    with pytest.raises(RuntimeError):
        usb.usb_topology(root, model)
