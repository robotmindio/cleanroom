#!/usr/bin/env python3
"""Drive the LeKiwi from the keyboard.

    ros2 run lekiwi_rmf teleop.py

    up/down     forward, back        left/right  strafe left, right
    1/2         turn left, right     space       stop
    9/0         slower, faster       Ctrl-C      quit

Arrows, digits and space are the only keys here because they sit on the same physical
key and send the same character on every layout -- QWERTY, Dvorak, Latin American --
so there is no layout to detect and nothing to configure.

Publishes Twist on /cmd_vel_manual, which must pass through the repository's command
mux and collision monitor before reaching the driver. Releasing a key does not stop the
robot -- the base runs until the next command, and LeRobot's host stops it by itself
after half a second of silence -- so this repeats the current command at 10 Hz and
zeroes it on exit.

There is nothing here that ros-jazzy-teleop-twist-keyboard would not do; it is only that
apt needs a password and this does not.
"""
import os
import sys
import select
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# x, y, yaw per key, scaled by the current speed setting. Arrows arrive as escape
# sequences; the terminal sends these three bytes for them in normal cursor mode.
KEYS = {
    "\x1b[A": (1.0, 0.0, 0.0),   # up
    "\x1b[B": (-1.0, 0.0, 0.0),  # down
    "\x1b[D": (0.0, 1.0, 0.0),   # left
    "\x1b[C": (0.0, -1.0, 0.0),  # right
    "1": (0.0, 0.0, 1.0),
    "2": (0.0, 0.0, -1.0),
    " ": (0.0, 0.0, 0.0),
}
SLOWER, FASTER = "9", "0"
# The base is geared for walking pace; these are gentle enough for an indoor first drive.
LINEAR = 0.15  # m/s
ANGULAR = 0.8  # rad/s


def read_key(timeout):
    """One keypress, or None if nothing arrives within timeout seconds.

    Reads the file descriptor rather than sys.stdin, because Python's buffer would
    swallow the tail of an arrow key's escape sequence where select cannot see it.
    """
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    key = os.read(sys.stdin.fileno(), 3).decode(errors="ignore")
    if key.startswith("\x1bO"):  # some terminals put the cursor keys in application mode
        key = "\x1b[" + key[2:]
    return key


def main():
    rclpy.init()
    node = Node("lekiwi_teleop")
    pub = node.create_publisher(Twist, "/cmd_vel_manual", 10)

    settings = termios.tcgetattr(sys.stdin)
    twist = Twist()
    scale = 1.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(__doc__.split("\n\n")[2])  # the key table, which is also the help above
        while rclpy.ok():
            key = read_key(0.1)
            if key == "\x03":  # cbreak leaves Ctrl-C to us
                break
            if key == SLOWER:
                scale = max(0.1, scale - 0.1)
                print(f"speed {scale:.1f}\r")
            elif key == FASTER:
                scale = min(2.0, scale + 0.1)
                print(f"speed {scale:.1f}\r")
            elif key in KEYS:
                x, y, yaw = KEYS[key]
                twist.linear.x = x * LINEAR * scale
                twist.linear.y = y * LINEAR * scale
                twist.angular.z = yaw * ANGULAR * scale
            elif key is not None:
                # Do not keep driving after a typo or an unexpected terminal sequence.
                twist = Twist()
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
