"""The zenoh sensor bridge accepts only mutually authenticated TLS peers."""

import os
import re
import signal
import shutil
import socket
import ssl
import subprocess
import sys
import time
from pathlib import Path

import pytest

from launch_snapshot import resolve_bringup

ROOT = Path(__file__).resolve().parents[1]
TLS_DIR = "/etc/lekiwi/zenoh-tls"


def generate(ca_dir, *extra):
    return subprocess.run(
        [str(ROOT / "scripts" / "setup-zenoh-tls.sh"), "--ca-dir", str(ca_dir), "--generate-only", *extra],
        check=True, capture_output=True, text=True,
    )


def openssl(*args):
    return subprocess.run(["openssl", *args], check=True, capture_output=True, text=True).stdout


def test_setup_generates_role_limited_identities_and_is_idempotent(tmp_path):
    ca = tmp_path / "ca"
    generate(ca)
    for name in ("ca", "device", "compute", "rmf"):
        assert (ca / f"{name}.crt").is_file()
    for name in ("ca", "device", "compute", "rmf"):
        assert (ca / f"{name}.key").stat().st_mode & 0o777 == 0o600
        # The certificate carries the key that was written beside it.
        assert openssl("x509", "-in", str(ca / f"{name}.crt"), "-noout", "-pubkey") == \
            openssl("pkey", "-in", str(ca / f"{name}.key"), "-pubout")
    assert "TLS Web Server Authentication" in openssl("x509", "-in", str(ca / "device.crt"), "-noout", "-text")
    assert "TLS Web Server Authentication" in openssl("x509", "-in", str(ca / "rmf.crt"), "-noout", "-text")
    compute = openssl("x509", "-in", str(ca / "compute.crt"), "-noout", "-text")
    assert "TLS Web Client Authentication" in compute and "Server Authentication" not in compute
    ssl.PEM_cert_to_DER_cert((ca / "device.crt").read_text())  # valid PEM

    before = (ca / "ca.crt").read_text(), (ca / "device.crt").read_text()
    generate(ca)
    assert before == ((ca / "ca.crt").read_text(), (ca / "device.crt").read_text())
    generate(ca, "--renew")
    assert (ca / "ca.crt").read_text() != before[0]


def test_bridge_configs_require_the_same_private_ca_and_never_use_plaintext():
    device = (ROOT / "config" / "zenoh_device.json5").read_text()
    computes = [(ROOT / "config" / f"zenoh_compute_{role}.json5").read_text()
                for role in ("scan", "depth", "images")]
    compute = "\n".join(computes)

    assert 'endpoints: ["tls/0.0.0.0:7447"]' in device and "tcp/" not in re.sub(r"//.*", "", device)
    assert "enable_mtls: true" in device and "enable_mtls: true" in compute
    for config in (device, *computes):
        assert f'root_ca_certificate: "{TLS_DIR}/ca.crt"' in config
    assert f'"{TLS_DIR}/device.key"' in device and f'"{TLS_DIR}/device.crt"' in device
    assert f'"{TLS_DIR}/compute.key"' in compute and f'"{TLS_DIR}/compute.crt"' in compute
    for arguments in ({"profile": "split"}, {"profile": "wired", "camera_source": "remote",
                                             "laser_source": "camera", "remote_ip": "192.0.2.4"}):
        bridges = [record["process"] for record in resolve_bringup(**arguments)
                   if "process" in record and record["process"][0] == "zenoh-bridge-ros2dds"]
        assert len(bridges) == 3
        endpoints = [bridge[bridge.index("-e") + 1] for bridge in bridges]
        assert len(endpoints) == 3
        assert sorted(endpoint.split("?prio=")[-1] for endpoint in endpoints) == ["0-1", "2-5", "6-7"]
        assert all(endpoint.startswith("tls/") and ":7447?prio=" in endpoint for endpoint in endpoints)
    for config in computes:
        assert 'mode: "client"' in config and "listen: { endpoints: [] }" in config
        assert "timeout_ms: -1" in config and "exit_on_failure: false" in config
        assert "period_max_ms: 1000" in config
    for role, config in zip(("scan", "depth", "images"), computes, strict=True):
        assert f'nodename: "zenoh_{role}_bridge"' in config


def test_sensor_bridge_caps_previews_and_drops_stale_sensor_samples_on_congestion():
    device = (ROOT / "config" / "zenoh_device.json5").read_text()
    computes = [(ROOT / "config" / f"zenoh_compute_{role}.json5").read_text()
                for role in ("scan", "depth", "images")]
    compute = "\n".join(computes)
    assert "reliable_routes_blocking: false" in device
    assert '"^/pi/lidar/scan$=1:express"' in device
    frequencies = re.search(r'pub_max_frequencies:\s*\[(.*?)\]', device, re.S).group(1)
    assert '^/pi/lidar/scan$=' not in frequencies
    assert '"^/camera/depth/points$=3"' not in device
    assert '"^/pi/camera/front/image_raw/compressed$=5"' in device
    assert '"^/pi/camera/wrist/image_raw/compressed$=5"' in device
    assert '"^/camera/astra/depth/image_raw/compressed$"' in device
    assert '"^/camera/astra/depth/image_raw/compressed$=6"' in device
    assert '"^/camera/astra/(color|depth)/image_raw/compressed$=2"' not in device
    assert '"^/camera/astra/color/image_raw/compressed$"' in device
    assert '"^/camera/astra/color/image_raw$"' not in device
    assert '"^/camera/astra/(color|depth)/image_raw/compressed$"' in compute


def test_main_installers_provision_the_identities():
    compute = (ROOT / "scripts" / "install-compute-services.sh").read_text()
    assert 'setup-zenoh-tls.sh" --user "$LEKIWI_SERVICE_USER" "${REMOTE:-local}"' in compute
    assert '$STACK_ARGS == *start_rmf:=true*' in compute
    # test_service_installation checks that a change to setup-zenoh-tls.sh changes the
    # compute service fingerprint, so deploy-split refreshes the compute service.


def test_fleet_actions_use_explicit_local_mutual_tls_without_discovery():
    bridge = (ROOT / 'config/zenoh_bridge.json5').read_text()
    client = (ROOT / 'config/zenoh_fleet_client.json5').read_text()
    for config in (bridge, client):
        assert '"tls/127.0.0.1:7448"' in config
        assert 'enable_mtls: true' in config and 'tcp/' not in config
        assert 'scouting: { multicast: { enabled: false }, gossip: { enabled: false } }' in config
    assert 'listen_private_key: "/etc/lekiwi/zenoh-tls/rmf.key"' in bridge
    assert 'listen: { endpoints: [] }' in client
    records = resolve_bringup(profile='split', localization='amcl', start_rmf='true', map_bundle_approved=True)
    adapter = next(record for gate in records for record in gate.get('start', [])
                   if record.get('node') == 'free_fleet_adapter/fleet_adapter.py')
    assert '--zenoh-config' in adapter['arguments']


def test_missing_options_and_unknown_flags_are_rejected():
    for args in (["--user"], ["--bogus"], ["one", "two"]):
        result = subprocess.run([str(ROOT / "scripts" / "setup-zenoh-tls.sh"), *args],
                                capture_output=True, text=True)
        assert result.returncode != 0 and result.stderr, args


def native_bridge():
    binary = shutil.which('zenoh-bridge-ros2dds')
    installed = Path.home() / '.local/bin/zenoh-bridge-ros2dds'
    if binary is None and installed.is_file():
        binary = str(installed)
    if binary is None:
        pytest.skip('native transport qualification requires the pinned Zenoh bridge')
    return binary


def test_native_priority_links_isolate_camera_delay_and_recover(tmp_path):
    binary = native_bridge()
    generate(tmp_path / 'pki')
    # Match the native bridge's middleware/interface without changing the pytest process.
    env = {**os.environ, 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
           'CYCLONEDDS_URI': f'file://{ROOT}/config/cyclonedds.xml'}
    env.pop('ROS_LOCALHOST_ONLY', None)
    command = [sys.executable, str(ROOT / 'test/zenoh_transport_faults.py'),
               '--root', str(ROOT), '--output', str(tmp_path), '--binary', binary]
    with subprocess.Popen(command, env=env, start_new_session=True) as process:
        try:
            assert process.wait(timeout=45) == 0, 'native transport probe failed; see its output directory'
        finally:
            # A timeout must not leave any of this isolated fixture's bridges alive.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_native_fleet_listener_requires_a_trusted_client_identity(tmp_path):
    binary = native_bridge()
    ca = tmp_path / 'pki'
    generate(ca)
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    config = (ROOT / 'config/zenoh_bridge.json5').read_text().replace(TLS_DIR, str(ca))
    path = tmp_path / 'fleet.json5'
    path.write_text(config.replace('127.0.0.1:7448', f'127.0.0.1:{port}'))
    env = {**os.environ, 'CYCLONEDDS_URI': f'file://{ROOT}/config/cyclonedds.xml'}
    with (tmp_path / 'fleet.log').open('w') as log:
        process = subprocess.Popen([binary, '-c', str(path), '-d', '226'], env=env, stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 5
            while True:
                assert process.poll() is None, (tmp_path / 'fleet.log').read_text()
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=.2):
                        break
                except ConnectionRefusedError:
                    assert time.monotonic() < deadline, 'fleet listener did not start'
                    time.sleep(.02)
            for identity in ('compute', None, 'device'):
                context = ssl.create_default_context(cafile=str(ca / 'ca.crt'))
                context.minimum_version = ssl.TLSVersion.TLSv1_3
                if identity:
                    context.load_cert_chain(str(ca / f'{identity}.crt'), str(ca / f'{identity}.key'))
                with socket.create_connection(('127.0.0.1', port), timeout=2) as connection:
                    if identity == 'compute':
                        with context.wrap_socket(connection, server_hostname='lekiwi-rmf') as secured:
                            assert secured.getpeercert()
                    else:
                        with pytest.raises(ssl.SSLError):
                            with context.wrap_socket(connection, server_hostname='lekiwi-rmf') as secured:
                                secured.recv(1)
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
