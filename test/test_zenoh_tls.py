"""The zenoh sensor bridge accepts only mutually authenticated TLS peers."""

import re
import ssl
import subprocess
from pathlib import Path

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
    compute = (ROOT / "config" / "zenoh_compute.json5").read_text()
    launch = (ROOT / "launch" / "bringup.launch.py").read_text()

    assert 'endpoints: ["tls/0.0.0.0:7447"]' in device and "tcp/" not in re.sub(r"//.*", "", device)
    assert "enable_mtls: true" in device and "enable_mtls: true" in compute
    for config in (device, compute):
        assert f'root_ca_certificate: "{TLS_DIR}/ca.crt"' in config
    assert f'"{TLS_DIR}/device.key"' in device and f'"{TLS_DIR}/device.crt"' in device
    assert f'"{TLS_DIR}/compute.key"' in compute and f'"{TLS_DIR}/compute.crt"' in compute
    assert '["tls/", remote_ip, ":7447"]' in launch and '["tcp/"' not in launch
