"""CURVE authentication helpers for the LeKiwi ZeroMQ transport.

The module deliberately imports pyzmq lazily so configuration validation and
unit tests do not require the motor-host virtual environment.
"""

from __future__ import annotations

import ipaddress
import os
import stat
from dataclasses import dataclass
from pathlib import Path


class CurveConfigurationError(ValueError):
    """CURVE key material is absent, incomplete, or unsafe to use."""


def is_loopback_address(address: str) -> bool:
    if isinstance(address, str) and address.strip().lower() in {"localhost", "ip6-localhost"}:
        return True
    try:
        return ipaddress.ip_address(address.strip()).is_loopback
    except ValueError:
        return False


def _existing_file(path: str, description: str, *, secret: bool = False) -> Path:
    resolved = Path(path).expanduser()
    if not resolved.is_file():
        raise CurveConfigurationError(f"{description} does not exist: {resolved}")
    if secret and os.name == "posix":
        mode = stat.S_IMODE(resolved.stat().st_mode)
        if mode & (stat.S_IRWXG | stat.S_IRWXO):
            raise CurveConfigurationError(
                f"{description} must not be accessible by group or other users: {resolved}"
            )
    return resolved


@dataclass(frozen=True)
class CurveClientCredentials:
    """Client secret certificate and the pinned server public certificate."""

    client_secret_key_file: str = ""
    server_public_key_file: str = ""

    @property
    def enabled(self) -> bool:
        return bool(self.client_secret_key_file and self.server_public_key_file)

    def validate(self) -> "CurveClientCredentials":
        configured = bool(self.client_secret_key_file), bool(self.server_public_key_file)
        if any(configured) and not all(configured):
            raise CurveConfigurationError(
                "both client secret and server public CURVE certificates are required"
            )
        if self.enabled:
            _existing_file(self.client_secret_key_file, "client secret certificate", secret=True)
            _existing_file(self.server_public_key_file, "server public certificate")
        return self

    def configure_socket(self, socket) -> None:
        if not self.enabled:
            return
        from zmq.auth import load_certificate

        client_public, client_secret = load_certificate(
            str(Path(self.client_secret_key_file).expanduser())
        )
        server_public, _unused = load_certificate(
            str(Path(self.server_public_key_file).expanduser())
        )
        if client_secret is None:
            raise CurveConfigurationError("client certificate does not contain a secret key")
        socket.curve_publickey = client_public
        socket.curve_secretkey = client_secret
        socket.curve_serverkey = server_public


class CurveServerSecurity:
    """One ZAP authenticator and server identity shared by all host sockets."""

    def __init__(
        self,
        context,
        bind_address: str,
        server_secret_key_file: str = "",
        authorized_clients_dir: str = "",
        *,
        allow_insecure_test_bind: bool = False,
    ):
        configured = bool(server_secret_key_file), bool(authorized_clients_dir)
        if any(configured) and not all(configured):
            raise CurveConfigurationError(
                "both server secret certificate and authorized-client directory are required"
            )
        self.enabled = all(configured)
        if not self.enabled:
            self._authenticator = None
            self._server_public = None
            self._server_secret = None
            return

        secret_path = _existing_file(
            server_secret_key_file, "server secret certificate", secret=True
        )
        clients_path = Path(authorized_clients_dir).expanduser()
        if not clients_path.is_dir():
            raise CurveConfigurationError(
                f"authorized-client directory does not exist: {clients_path}"
            )
        if not any(path.is_file() for path in clients_path.glob("*.key")):
            raise CurveConfigurationError(
                f"authorized-client directory contains no public .key certificates: {clients_path}"
            )

        from zmq.auth import load_certificate
        from zmq.auth.thread import ThreadAuthenticator

        public, secret = load_certificate(str(secret_path))
        if secret is None:
            raise CurveConfigurationError("server certificate does not contain a secret key")
        self._server_public = public
        self._server_secret = secret
        self._authenticator = ThreadAuthenticator(context)
        self._authenticator.start()
        try:
            self._authenticator.configure_curve(domain="*", location=str(clients_path))
        except Exception:
            self._authenticator.stop()
            raise

    def configure_socket(self, socket) -> None:
        if not self.enabled:
            return
        socket.curve_publickey = self._server_public
        socket.curve_secretkey = self._server_secret
        socket.curve_server = True

    def close(self) -> None:
        if self._authenticator is not None:
            self._authenticator.stop()
            self._authenticator = None
