import time

import rclpy


def spin_until(node, predicate, timeout: float = 5.0, period: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=period)
        if predicate():
            return True
    return False
