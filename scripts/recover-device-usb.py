#!/usr/bin/env python3
"""Recover missing USB devices on a Pi 5 whose motors are already torque-off.

Run from the compute host after sourcing the selected real-robot setup. Default
is a read-only check. --cycle explicitly switches the Pi's shared USB VBUS group;
this disconnects the motor adapter too. It never disarms or changes safety gates.
Powered arms must still reach travel_stow before any operation that cuts torque.
"""

import argparse
from collections import deque
import inspect
import json
import math
import os
from pathlib import PurePosixPath
import re
import shlex
import subprocess
import time
import uuid


ARM = ('arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
       'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper')
SERVOS = (*ARM, 'base_back_wheel', 'base_left_wheel', 'base_right_wheel')
RESOURCES = {'arm', 'base', 'perception_query'}
MAX_AGE = 0.5
FUTURE_GRACE = 0.05


def require_off(proof):
    """No pose tolerance: this permits no transition from powered to unpowered."""
    for key in ('joints', 'diagnostics', 'driver', 'arm_permitted', 'base_permitted', 'resources'):
        age = proof[key]['age']
        if not math.isfinite(age) or not 0 <= age <= MAX_AGE:
            raise RuntimeError(f'{key} is stale or missing')
    if proof['driver']['value'] != 'DISARMED':
        raise RuntimeError('driver must already be DISARMED')
    for key in ('arm_permitted', 'base_permitted'):
        if proof[key]['value'] is not False:
            raise RuntimeError(f'{key} must be false')
    resources = proof['resources']['value']
    if set(resources) != RESOURCES or any(value is not None for value in resources.values()):
        raise RuntimeError('robot resources must all be free')
    diag = proof['diagnostics']
    for name in ('motor_bus', *(f'servo/{name}' for name in SERVOS)):
        status = diag['value'].get(name, {})
        if status.get('level') != 0 or status.get('torque_enabled', '').lower() != 'false':
            raise RuntimeError(f'{name}: healthy measured torque-off required')
    for key in ('diagnostics', 'joints'):
        age = proof[key]['stamp_age']
        if not math.isfinite(age) or not -FUTURE_GRACE <= age <= MAX_AGE:
            raise RuntimeError(f'{key} source stamp is invalid')
    joints = proof['joints']
    if not 0.2 <= joints['stable_seconds'] <= MAX_AGE:
        raise RuntimeError('need fresh joint samples spanning at least 0.2 seconds')
    if not -FUTURE_GRACE <= joints['baseline_stamp_age'] <= MAX_AGE:
        raise RuntimeError('baseline joint source stamp is invalid')
    for values in (joints['value'], joints['baseline']):
        if set(values) != set(ARM) or not all(math.isfinite(v) for v in values.values()):
            raise RuntimeError('incomplete or nonfinite arm joint state')
    if any(not math.isfinite(v) or v > 0.01 or v < 0 for v in joints['spread'].values()):
        raise RuntimeError('unpowered arm is moving; USB recovery refused')
    if set(joints['spread']) != set(ARM):
        raise RuntimeError('missing joint stability evidence')


def cycle_usb(probe, remote, hubs, audit, pause=time.sleep, clock=time.monotonic):
    """Recheck before every OFF; independently schedule restoration before any OFF."""
    proof = probe()
    audit({'event': 'initial_gate', 'proof': proof})
    require_off(proof)
    remote.schedule_restore(hubs)
    deadline = clock() + 10
    try:
        for hub in hubs:
            proof = probe()
            audit({'event': 'before_off', 'hub': hub, 'proof': proof})
            require_off(proof)
            if clock() >= deadline:
                raise RuntimeError('USB OFF window expired; restoring power')
            remote.power(hub, 'off')
        remote.require_all_off(hubs)
        pause(3)
    finally:
        # Attempt every hub even if restoring an earlier hub fails. The Pi-local
        # timer is left scheduled, including when SSH or this process disappears.
        failures = []
        for hub in hubs:
            try:
                remote.power(hub, 'on')
            except Exception as error:
                failures.append(f'{hub}: {error}')
        audit({'event': 'restore', 'failures': failures})
        if failures:
            raise RuntimeError('immediate USB restoration failed; Pi-local timer remains: '
                               + '; '.join(failures))


# Only onboard Pi 5 xHCI root hubs. Refuse attached USB storage/network before
# touching the shared supply; keep the motor adapter's root last in OFF order.
def usb_topology(root, model_path, pci_root=None):
    model = model_path.read_text().rstrip('\0')
    if not model.startswith('Raspberry Pi 5 '):
        raise RuntimeError('USB recovery requires Raspberry Pi 5: ' + model)
    for device in pci_root.iterdir() if pci_root is not None else ():
        vendor, product = device / 'vendor', device / 'device'
        if (vendor.exists() and vendor.read_text().strip() == '0x1106'
                and product.read_text().strip() == '0x3483'
                and not (device / 'driver').exists()):
            raise RuntimeError('VL805 PCIe USB controller is unbound: ' + device.name
                               + '; inspect boot log for DMA failure before cycling onboard USB')
    hubs, motor_hubs = [], set()
    for hub in root.glob('usb*'):
        if 'xhci' not in str(hub.resolve()):
            continue
        if (hub / 'idVendor').read_text().strip() != '1d6b':
            raise RuntimeError('unexpected USB root vendor')
        hubs.append((hub / 'busnum').read_text().strip())
    if len(hubs) != 4:
        raise RuntimeError('expected four onboard xHCI root hubs')
    for device in root.iterdir():
        bus = device.name.split('-')[0].split(':')[0]
        if bus not in hubs:
            continue
        cls = device / 'bInterfaceClass'
        # CDC serial has class 02/0a too, including the motor adapter. Detect
        # actual network interfaces, rather than misclassifying serial as net.
        if (device / 'net').exists() or (cls.exists() and cls.read_text().strip() == '08'):
            raise RuntimeError('USB storage/network interface attached: ' + device.name)
        vendor, product = device / 'idVendor', device / 'idProduct'
        if (vendor.exists() and vendor.read_text().strip() == '1a86'
                and product.read_text().strip() == '55d3'):
            motor_hubs.add(bus)
    if len(motor_hubs) != 1:
        raise RuntimeError('expected exactly one motor USB adapter')
    return {'model': model, 'hubs': sorted(hubs, key=lambda h: (h in motor_hubs, int(h)))}



class Remote:
    def __init__(self, host, identity, tool, audit):
        self.ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3']
        if identity:
            self.ssh += ['-o', 'IdentitiesOnly=yes', '-i', identity]
        self.ssh += [host]
        self.tool, self.audit = tool, audit

    def run(self, argv, timeout=8):
        result = subprocess.run([*self.ssh, shlex.join(argv)], capture_output=True,
                                text=True, timeout=timeout)
        if result.returncode:
            raise RuntimeError(f'Pi command failed ({result.returncode}): {result.stderr.strip()}')
        return result.stdout

    def topology(self):
        source = ('import json\nfrom pathlib import Path\n' + inspect.getsource(usb_topology)
                  + "\nprint(json.dumps(usb_topology(Path('/sys/bus/usb/devices'), "
                  + "Path('/proc/device-tree/model'), Path('/sys/bus/pci/devices'))))")
        return json.loads(self.run(['python3', '-c', source]))

    def power(self, hub, action):
        output = self.run(['sudo', '-n', self.tool, '-e', '-l', hub, '-a', action])
        self.audit({'event': 'power', 'hub': hub, 'action': action, 'output': output})
        if action == 'off' and not ports_off(output.split('New status for hub')[-1]):
            raise RuntimeError(f'hub {hub} did not confirm USB power off')

    def require_all_off(self, hubs):
        for hub in hubs:
            output = self.run(['sudo', '-n', self.tool, '-e', '-l', hub])
            self.audit({'event': 'off_readback', 'hub': hub, 'output': output})
            if not ports_off(output):
                raise RuntimeError(f'hub {hub} is powered; shared USB supply never turned off')

    def schedule_restore(self, hubs):
        active = self.run(['systemctl', '--user', 'list-units', '--plain', '--no-legend',
                           '--state=active,activating', 'lekiwi-usb-restore-*'])
        if active.strip():
            raise RuntimeError('previous USB restoration still active; wait for it to finish')
        # No &&: restoration of subsequent hubs must survive individual failures.
        restore = '\n'.join(shlex.join(['timeout', '3s', 'sudo', '-n', self.tool, '-e', '-l', h, '-a', 'on'])
                            for h in hubs)
        # Repeat for a minute so delayed in-flight SSH commands cannot outlive
        # a single restoration attempt. ON is idempotent and does not reset USB.
        restore = 'for attempt in {1..30}; do\n' + restore + '\nsleep 2\ndone'
        unit = 'lekiwi-usb-restore-' + uuid.uuid4().hex
        self.run(['systemd-run', '--user', '--on-active=15s', '--timer-property=AccuracySec=1s',
                  '--unit=' + unit, '/bin/bash', '-c', restore])
        # Verify both timer activation and the executable before the first OFF.
        self.run(['systemctl', '--user', 'is-active', unit + '.timer'])
        self.run(['test', '-x', self.tool])
        self.audit({'event': 'restore_timer', 'unit': unit})


def ports_off(output):
    ports = re.findall(r'^\s*Port \d+:.*$', output, flags=re.MULTILINE)
    return bool(ports) and all(re.search(r'\boff\b', line) for line in ports)


class LiveProbe:
    def __init__(self):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger
        self.ros, self.trigger = rclpy, Trigger
        rclpy.init()
        if rclpy.get_default_context().get_domain_id() != 0:
            rclpy.try_shutdown()
            raise RuntimeError('USB recovery requires real-robot ROS domain 0')
        self.node = rclpy.create_node('verified_usb_recovery')
        self.values, self.joints = {}, deque(maxlen=100)
        n = self.node
        n.create_subscription(JointState, '/joint_states', self.joint, qos_profile_sensor_data)
        n.create_subscription(DiagnosticArray, '/hardware/diagnostics', self.diag, 10)
        for key, topic, typ in (
                ('driver', '/safety/driver_state', String),
                ('arm_permitted', '/safety/arm_motion_permitted', Bool),
                ('base_permitted', '/safety/base_motion_permitted', Bool)):
            n.create_subscription(typ, topic, lambda m, k=key: self.remember(k, m.data),
                                  qos_profile_sensor_data)
        self.client = n.create_client(Trigger, '/robot/capabilities')

    def remember(self, key, value, stamp=None):
        self.values[key] = {'value': value, 'received': time.monotonic(), 'stamp': stamp}

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def joint(self, message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            self.joints.clear()
            self.values.pop('joints', None)
            return
        values = {name: pos for name, pos in zip(message.name, message.position) if name in ARM}
        self.remember('joints', values, self.stamp(message))
        self.joints.append(self.values['joints'])

    def diag(self, message):
        statuses = {}
        for status in message.status:
            if status.name in statuses:
                self.values.pop('diagnostics', None)
                return
            level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
            statuses[status.name] = {'level': level, **{v.key: v.value for v in status.values}}
        self.remember('diagnostics', statuses, self.stamp(message))

    def __call__(self):
        started = time.monotonic()
        if not self.client.wait_for_service(timeout_sec=2):
            raise RuntimeError('task gateway unavailable')
        future = self.client.call_async(self.trigger.Request())
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.ros.spin_once(self.node, timeout_sec=0.02)
            if future.done() and 'resources' not in self.values:
                result = future.result()
                if not result.success:
                    raise RuntimeError('task gateway rejected state query')
                self.remember('resources', json.loads(result.message)['task_api']['resources'])
            if (time.monotonic() - started >= 0.3 and len(self.joints) >= 2
                    and all(k in self.values for k in
                            ('joints', 'diagnostics', 'driver', 'arm_permitted', 'base_permitted', 'resources'))):
                now, ros_now = time.monotonic(), self.node.get_clock().now().nanoseconds * 1e-9
                proof = {k: {'value': v['value'], 'age': now - v['received'],
                             **({'stamp_age': ros_now - v['stamp']} if v['stamp'] is not None else {})}
                         for k, v in self.values.items()}
                baseline = joint_baseline(self.joints, self.values['joints']['received'], ros_now)
                if baseline is None:
                    continue
                window = [v['value'] for v in self.joints if baseline and v['received'] >= baseline['received']]
                proof['joints'].update(baseline=baseline['value'] if baseline else {},
                                       baseline_stamp_age=ros_now - baseline['stamp'] if baseline else float('inf'),
                                       spread={name: max(v[name] for v in window) - min(v[name] for v in window)
                                               for name in ARM if window and all(name in v for v in window)},
                                       stable_seconds=(self.values['joints']['received'] - baseline['received']
                                                       if baseline else 0))
                self.values.pop('resources', None)
                return proof
        self.values.pop('resources', None)
        raise RuntimeError('no complete fresh robot state within 3 seconds')

    def close(self):
        self.node.destroy_node()
        self.ros.try_shutdown()


def joint_baseline(samples, latest_receipt, ros_now):
    """Use the most recent adequate baseline, never an older stale source sample."""
    return next((v for v in reversed(samples)
                 if 0.2 <= latest_receipt - v['received'] <= MAX_AGE
                 and -FUTURE_GRACE <= ros_now - v['stamp'] <= MAX_AGE), None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ssh-host', required=True)
    parser.add_argument('--identity')
    parser.add_argument('--uhubctl', required=True, help='absolute Pi path to uhubctl')
    parser.add_argument('--cycle', action='store_true', help='explicitly cycle shared USB power')
    args = parser.parse_args()
    if (args.ssh_host.startswith('-') or not PurePosixPath(args.uhubctl).is_absolute()
            or os.environ.get('ROS_DOMAIN_ID', '0') != '0'):
        parser.error('require a real host, absolute uhubctl path and ROS domain 0')

    def audit(event):
        print(json.dumps({'time': time.time(), **event}, allow_nan=False), flush=True)

    remote = Remote(args.ssh_host, args.identity, args.uhubctl, audit)
    topology = remote.topology()
    audit({'event': 'topology', **topology})
    probe = LiveProbe()
    try:
        if args.cycle:
            cycle_usb(probe, remote, topology['hubs'], audit)
        else:
            proof = probe()
            require_off(proof)
            audit({'event': 'check_passed', 'proof': proof})
    finally:
        probe.close()


if __name__ == '__main__':
    main()
