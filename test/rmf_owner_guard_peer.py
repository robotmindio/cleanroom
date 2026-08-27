#!/usr/bin/env python3
"""Launch-test peer that has the exact public name of a free_fleet adapter."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("lekiwi_fleet_adapter")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
