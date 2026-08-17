#!/usr/bin/env python3
"""Drive the LeKiwi from the keyboard.

    ros2 run lekiwi_rmf teleop.py

    w/s   forward, back        a/d   strafe left, right
    q/e   turn left, right     space  stop
    -/+   slower, faster       Ctrl-C quit

Publishes Twist on /cmd_vel, the same topic Nav2 drives, so send a goal or teleoperate
but not both at once. Releasing a key does not stop the robot -- the base runs until the
next command, and LeRobot's host stops it by itself after half a second of silence -- so
this repeats the current command at 10 Hz and zeroes it on exit.

There is nothing here that ros-jazzy-teleop-twist-keyboard would not do; it is only that
apt needs a password and this does not.
"""
import sys
import select
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# x, y, yaw per key, scaled by the current speed setting
KEYS = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
    " ": (0.0, 0.0, 0.0),
}
# The base is geared for walking pace; these are gentle enough for an indoor first drive.
LINEAR = 0.15  # m/s
ANGULAR = 0.8  # rad/s


def read_key(timeout):
    """One keypress, or None if nothing arrives within timeout seconds."""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def main():
    rclpy.init()
    node = Node("lekiwi_teleop")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    settings = termios.tcgetattr(sys.stdin)
    twist = Twist()
    scale = 1.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        print("\n".join(__doc__.split("\n\n")[1:3]))
        while rclpy.ok():
            key = read_key(0.1)
            if key == "\x03":  # cbreak leaves Ctrl-C to us
                break
            if key in ("-", "_"):
                scale = max(0.1, scale - 0.1)
                print(f"speed {scale:.1f}\r")
            elif key in ("+", "="):
                scale = min(2.0, scale + 0.1)
                print(f"speed {scale:.1f}\r")
            elif key in KEYS:
                x, y, yaw = KEYS[key]
                twist.linear.x = x * LINEAR * scale
                twist.linear.y = y * LINEAR * scale
                twist.angular.z = yaw * ANGULAR * scale
            pub.publish(twist)  # repeat: the host stops the base without a fresh command
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        pub.publish(Twist())  # never leave the robot driving
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
