#!/usr/bin/env python3
"""Refuse to start a duplicate free_fleet adapter visible on this DDS graph.

This is deliberately a preflight, not a process manager.  It identifies the
two stable node names created by free_fleet_adapter's ``fleet_adapter.py`` for
the configured fleet and exits before this launch starts its own adapter.
It neither enumerates operating-system processes nor kills, adopts, or changes
any participant it sees.

ROS graph discovery only covers participants discoverable in this process's
current ROS domain and DDS network configuration.  It cannot make claims about
other domains, blocked peers, or hosts outside that discovered graph.  Like any
preflight, it also cannot prevent a second adapter from starting after this
process has completed its check; deployments needing that stronger guarantee
need a single external supervisor/lease authority.
"""

from __future__ import annotations

import math
from pathlib import Path
import time
from typing import Iterable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml


CLEAR_EXIT = 0
CONFLICT_EXIT = 20
ERROR_EXIT = 21


def expected_free_fleet_nodes(fleet_name: str) -> frozenset[str]:
    """Return free_fleet_adapter's fully qualified ownership node names."""
    name = str(fleet_name).strip()
    if not name:
        raise ValueError("fleet_name must be non-empty")
    if "/" in name:
        raise ValueError("fleet_name must not contain '/'")
    # free_fleet_adapter/fleet_adapter.py creates these exact node names:
    # rclpy.node.Node(f'{fleet_name}_command_handle') and
    # rmf_adapter.Adapter.make(f'{fleet_name}_fleet_adapter').
    return frozenset({
        f"/{name}_command_handle",
        f"/{name}_fleet_adapter",
    })


def qualified_node_name(name: str, namespace: str) -> str:
    """Normalize rclpy graph tuples to absolute ROS node names."""
    clean_name = str(name).strip("/")
    clean_namespace = str(namespace).strip("/")
    if not clean_name:
        raise ValueError("ROS node name must be non-empty")
    return f"/{clean_namespace}/{clean_name}" if clean_namespace else f"/{clean_name}"


def conflicting_free_fleet_nodes(
    graph_nodes: Iterable[tuple[str, str]], fleet_name: str
) -> frozenset[str]:
    """Return adapter ownership nodes visible in an rclpy graph snapshot.

    An exact match is intentional.  It avoids treating unrelated nodes whose
    names merely contain ``fleet_adapter`` as an ownership conflict.
    """
    expected = expected_free_fleet_nodes(fleet_name)
    return frozenset(
        qualified_node_name(name, namespace)
        for name, namespace in graph_nodes
        if qualified_node_name(name, namespace) in expected
    )


def fleet_name_from_config(config_path: str) -> str:
    """Read and validate the fleet identity from a tracked fleet YAML file."""
    path = Path(str(config_path)).expanduser()
    if not str(config_path).strip():
        raise ValueError("fleet_config must be supplied")
    try:
        with path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
    except OSError as error:
        raise ValueError(f"unable to read fleet_config {path}: {error}") from error
    except yaml.YAMLError as error:
        raise ValueError(f"invalid fleet_config YAML {path}: {error}") from error
    try:
        fleet_name = config["rmf_fleet"]["name"]
    except (KeyError, TypeError) as error:
        raise ValueError("fleet_config must contain rmf_fleet.name") from error
    if not isinstance(fleet_name, str) or not fleet_name.strip():
        raise ValueError("fleet_config rmf_fleet.name must be a non-empty string")
    return fleet_name.strip()


def resolve_fleet_name(fleet_name: str, fleet_config: str) -> str:
    """Resolve one identity and reject conflicting launch/config declarations."""
    configured_name = fleet_name_from_config(fleet_config) if str(fleet_config).strip() else ""
    requested_name = str(fleet_name).strip()
    if configured_name and requested_name and configured_name != requested_name:
        raise ValueError(
            f"fleet_name {requested_name!r} does not match fleet_config name {configured_name!r}"
        )
    resolved = configured_name or requested_name
    # Reuse the public-name validation so an invalid configuration cannot turn
    # into an accidentally permissive graph query.
    expected_free_fleet_nodes(resolved)
    return resolved


class RMFOwnerGuard(Node):
    """Observe graph discovery for a bounded interval before permitting startup."""

    def __init__(self) -> None:
        super().__init__("rmf_owner_guard")
        self.declare_parameter("fleet_name", "")
        self.declare_parameter("fleet_config", "")
        self.declare_parameter("settle_seconds", 1.0)
        self.declare_parameter("poll_period_seconds", 0.1)
        self.fleet_name = resolve_fleet_name(
            str(self.get_parameter("fleet_name").value),
            str(self.get_parameter("fleet_config").value),
        )
        self.settle_seconds = self._positive_finite_parameter("settle_seconds")
        self.poll_period_seconds = self._positive_finite_parameter("poll_period_seconds")

    def _positive_finite_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def conflicts(self) -> frozenset[str]:
        return conflicting_free_fleet_nodes(
            self.get_node_names_and_namespaces(), self.fleet_name
        )

    def check(self) -> frozenset[str]:
        """Scan through discovery settling; return immediately on a conflict."""
        deadline = time.monotonic() + self.settle_seconds
        while True:
            conflicts = self.conflicts()
            if conflicts:
                return conflicts
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return frozenset()
            rclpy.spin_once(self, timeout_sec=min(self.poll_period_seconds, remaining))


def main(args: Optional[list[str]] = None) -> None:
    node: Optional[RMFOwnerGuard] = None
    exit_code = ERROR_EXIT
    try:
        rclpy.init(args=args)
        node = RMFOwnerGuard()
        conflicts = node.check()
        if conflicts:
            node.get_logger().error(
                "refusing RMF startup: discovered free_fleet ownership node(s) "
                f"for fleet {node.fleet_name!r}: {', '.join(sorted(conflicts))}"
            )
            exit_code = CONFLICT_EXIT
        else:
            node.get_logger().info(
                f"no free_fleet ownership nodes for fleet {node.fleet_name!r} "
                "were discovered on this ROS graph"
            )
            exit_code = CLEAR_EXIT
    except (KeyboardInterrupt, ExternalShutdownException):
        # Interrupted preflight must not be treated as permission to launch.
        exit_code = ERROR_EXIT
    except Exception as error:
        if node is not None:
            node.get_logger().error(f"RMF ownership preflight failed: {error}")
        else:
            print(f"RMF ownership preflight failed: {error}")
        exit_code = ERROR_EXIT
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
