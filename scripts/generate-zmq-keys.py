#!/usr/bin/env python3
"""Generate a LeKiwi CURVE server identity and authorized client keys."""

import argparse
import os
import shutil
from pathlib import Path

from zmq.auth import create_certificates


def private(path: Path) -> None:
    os.chmod(path, 0o600)


def public(path: Path) -> None:
    os.chmod(path, 0o644)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate CURVE certificates without overwriting existing identities."
    )
    parser.add_argument("output_dir", type=Path)
    parser.add_argument(
        "--client", action="append", default=[], metavar="NAME",
        help="authorized client identity to generate (repeatable; default: driver and health)",
    )
    args = parser.parse_args()
    clients = args.client or ["driver", "health"]
    if any(not name or name in {".", ".."} or "/" in name for name in clients):
        parser.error("client names must be non-empty path components")
    if len(set(clients)) != len(clients):
        parser.error("client names must be unique")

    output = args.output_dir.expanduser().resolve()
    expected = [output / "server.key", output / "server.key_secret"]
    for name in clients:
        expected.extend((output / "clients" / f"{name}.key", output / "clients" / f"{name}.key_secret"))
    existing = [path for path in expected if path.exists()]
    if existing:
        parser.error(f"refusing to overwrite existing certificate: {existing[0]}")

    clients_dir = output / "clients"
    authorized_dir = output / "authorized_clients"
    clients_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    authorized_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    server_public, server_secret = map(Path, create_certificates(str(output), "server"))
    private(server_secret)
    public(server_public)
    for name in clients:
        client_public, client_secret = map(
            Path, create_certificates(str(clients_dir), name)
        )
        private(client_secret)
        public(client_public)
        authorized = authorized_dir / client_public.name
        shutil.copy2(client_public, authorized)
        public(authorized)

    print(f"server secret: {server_secret}")
    print(f"server public: {server_public}")
    print(f"authorized clients: {authorized_dir}")
    for name in clients:
        print(f"client {name} secret: {clients_dir / f'{name}.key_secret'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
