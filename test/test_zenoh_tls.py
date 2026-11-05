"""The zenoh sensor bridge accepts only mutually authenticated TLS peers."""

import re
import shutil
import ssl
import subprocess
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
    for name in ("ca", "device", "compute"):
        assert (ca / f"{name}.crt").is_file()
    for name in ("ca", "device", "compute"):
        assert (ca / f"{name}.key").stat().st_mode & 0o777 == 0o600
        # The certificate carries the key that was written beside it.
        assert openssl("x509", "-in", str(ca / f"{name}.crt"), "-noout", "-pubkey") == \
            openssl("pkey", "-in", str(ca / f"{name}.key"), "-pubout")
    assert "TLS Web Server Authentication" in openssl("x509", "-in", str(ca / "device.crt"), "-noout", "-text")
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
    assert '"^/pi/camera/wrist/image_raw/compressed$=2"' in device
    assert '"^/camera/astra/depth/image_raw/compressed$"' in device
    assert '"^/camera/astra/depth/image_raw/compressed$=6"' in device
    assert '"^/camera/astra/(color|depth)/image_raw/compressed$=2"' not in device
    assert '"^/camera/astra/color/image_raw/compressed$"' in device
    assert '"^/camera/astra/color/image_raw$"' not in device
    assert '"^/camera/astra/(color|depth)/image_raw/compressed$"' in compute


def test_main_installers_provision_the_identities():
    compute = (ROOT / "scripts" / "install-compute-services.sh").read_text()
    assert 'setup-zenoh-tls.sh" --user "$LEKIWI_SERVICE_USER" "${REMOTE:-local}"' in compute
    # test_service_installation checks that a change to setup-zenoh-tls.sh changes the
    # compute service fingerprint, so deploy-split refreshes the compute service.


def test_missing_options_and_unknown_flags_are_rejected():
    for args in (["--user"], ["--bogus"], ["one", "two"]):
        result = subprocess.run([str(ROOT / "scripts" / "setup-zenoh-tls.sh"), *args],
                                capture_output=True, text=True)
        assert result.returncode != 0 and result.stderr, args


def test_native_priority_links_isolate_camera_delay_and_recover(tmp_path):
    binary = shutil.which('zenoh-bridge-ros2dds')
    if binary is None:
        pytest.skip('native transport qualification requires the pinned Zenoh bridge')
    from zenoh_transport_faults import probe
    generate(tmp_path / 'pki')
    probe(ROOT, tmp_path, binary)
