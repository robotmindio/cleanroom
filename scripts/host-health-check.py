#!/usr/bin/env python3
"""Check that the managed motor host is serving both required protocols.

This is intentionally read-only: TCP :5555 must accept connections and the
repository-owned torque endpoint must reply to a ``state`` request.  Merely
checking listening sockets can accept an unrelated or half-started process.
"""

from __future__ import annotations

import argparse
import socket
import sys

import zmq

from lekiwi_rmf.zmq_security import CurveClientCredentials


def check(
    host: str, timeout_s: float, client_secret_key_file: str = "",
    server_public_key_file: str = "",
) -> None:
    with socket.create_connection((host, 5555), timeout=timeout_s):
        pass
    context = zmq.Context()
    client = context.socket(zmq.REQ)
    try:
        timeout_ms = max(1, int(timeout_s * 1000))
        client.setsockopt(zmq.LINGER, 0)
        client.setsockopt(zmq.SNDTIMEO, timeout_ms)
        client.setsockopt(zmq.RCVTIMEO, timeout_ms)
        CurveClientCredentials(
            client_secret_key_file, server_public_key_file
        ).validate().configure_socket(client)
        client.connect(f"tcp://{host}:5557")
        client.send_json({"command": "state"})
        response = client.recv_json()
    finally:
        client.close()
        context.term()
    if not isinstance(response, dict) or response.get("ok") is not True:
        raise RuntimeError(f"torque endpoint rejected state request: {response!r}")
    if not isinstance(response.get("torque_enabled"), bool):
        raise RuntimeError(f"torque endpoint returned invalid state: {response!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--client-secret-key-file", default="")
    parser.add_argument("--server-public-key-file", default="")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    try:
        check(
            args.host, args.timeout, args.client_secret_key_file,
            args.server_public_key_file,
        )
    except (OSError, ValueError, zmq.ZMQError, RuntimeError) as error:
        print(f"host health check failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
