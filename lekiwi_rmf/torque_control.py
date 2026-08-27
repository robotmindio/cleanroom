"""Small request/reply client for the motor-host torque safety channel."""

from __future__ import annotations

import ipaddress
import json
import math

from lekiwi_rmf.zmq_security import (
    CurveClientCredentials, CurveConfigurationError, is_loopback_address,
)


class TorqueControlError(RuntimeError):
    """The host did not confirm the requested physical torque state."""


def validate_action_payload(message: str, expected_keys) -> dict[str, float]:
    """Decode one complete finite action without JSON duplicate-key ambiguity."""
    def object_from_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate action key {key!r}")
            result[key] = value
        return result

    try:
        decoded = json.loads(
            message,
            object_pairs_hook=object_from_pairs,
            parse_constant=lambda value: (_ for _ in ()).throw(
                ValueError(f"invalid JSON number {value}")
            ),
        )
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError("action must be valid strict JSON") from error
    if not isinstance(decoded, dict) or set(decoded) != set(expected_keys):
        raise ValueError("action must contain exactly every configured motor command")
    if any(isinstance(value, bool) for value in decoded.values()):
        raise ValueError("action values must be finite numbers")
    try:
        action = {key: float(value) for key, value in decoded.items()}
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("action values must be finite numbers") from error
    if not all(math.isfinite(value) for value in action.values()):
        raise ValueError("action values must be finite numbers")
    return action


def run_all_safety_steps(steps):
    """Run every named safety step and return failures without short-circuiting."""
    failures = []
    for name, operation in steps:
        try:
            operation()
        except Exception as error:  # The caller decides which failures are fatal.
            failures.append((name, error))
    return failures


def enable_with_rollback(enable_steps, rollback_steps):
    """Run an enable transaction, executing every rollback step on any failure."""
    try:
        for _name, operation in enable_steps:
            operation()
    except Exception as enable_error:
        failures = run_all_safety_steps(rollback_steps)
        detail = "; ".join(f"{name}: {error}" for name, error in failures)
        suffix = f"; rollback failures: {detail}" if detail else ""
        raise RuntimeError(f"enable transaction failed: {enable_error}{suffix}") from enable_error


def validated_bind_address(value: str) -> str:
    """Require one explicit IPv4 control interface; never accept a wildcard."""
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as error:
        raise ValueError("bind address must be an explicit IPv4 address") from error
    if address.is_unspecified or address.is_multicast:
        raise ValueError("bind address may not be wildcard or multicast")
    return str(address)


def torque_readback_matches(states, enabled: bool, expected_motors) -> bool:
    """Return true only when every expected servo confirms the requested state."""
    if not isinstance(states, dict):
        return False
    expected = set(expected_motors)
    return set(states) == expected and all(
        value == (1 if enabled else 0) and not isinstance(value, bool)
        for value in states.values()
    )


class TorqueControlClient:
    """Use a one-shot ZMQ request so service callbacks cannot share socket state."""

    def __init__(
        self, host: str, port: int = 5557, timeout_ms: int = 1000, zmq_module=None,
        client_secret_key_file: str = "", server_public_key_file: str = "",
        *, allow_insecure_test_connection: bool = False,
    ):
        if not isinstance(host, str) or not host.strip():
            raise ValueError("torque-control host must be a non-empty string")
        if not isinstance(port, int) or isinstance(port, bool) or not 1 <= port <= 65535:
            raise ValueError("torque-control port must be between 1 and 65535")
        if not isinstance(timeout_ms, int) or isinstance(timeout_ms, bool) or timeout_ms <= 0:
            raise ValueError("torque-control timeout must be a positive integer")
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self._zmq_module = zmq_module
        self.curve = CurveClientCredentials(
            client_secret_key_file, server_public_key_file
        ).validate()
        if (
            not is_loopback_address(host)
            and not self.curve.enabled
            and not allow_insecure_test_connection
        ):
            raise CurveConfigurationError(
                "refusing unauthenticated torque connection to a non-loopback host"
            )

    def _zmq(self):
        if self._zmq_module is None:
            import zmq

            return zmq
        return self._zmq_module

    def set_enabled(self, enabled: bool) -> None:
        """Require the host to confirm that all servo torque is on or off."""
        if not isinstance(enabled, bool):
            raise ValueError("torque state must be boolean")
        zmq = self._zmq()
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        try:
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self.curve.configure_socket(socket)
            socket.connect(f"tcp://{self.host}:{self.port}")
            socket.send_json({"command": "enable" if enabled else "disable"})
            response = socket.recv_json()
        except Exception as error:
            raise TorqueControlError(f"torque host {self.host}:{self.port} did not reply: {error}") from error
        finally:
            socket.close()
            context.term()

        if not isinstance(response, dict) or response.get("ok") is not True:
            detail = response.get("error", "invalid response") if isinstance(response, dict) else "invalid response"
            raise TorqueControlError(f"torque host rejected the request: {detail}")
        if response.get("torque_enabled") is not enabled:
            raise TorqueControlError("torque host confirmed an unexpected torque state")
