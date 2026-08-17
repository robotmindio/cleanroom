#!/usr/bin/env python3
"""Drive the LeKiwi from the keyboard.

    ros2 run lekiwi_rmf teleop.py

The keys are the WASD block by position, not by letter, so a Dvorak layout gets the
same physical keys under the same fingers. The program prints the ones it picked;
set LEKIWI_LAYOUT=qwerty or =dvorak to override what it detects.

Publishes Twist on /cmd_vel, the same topic Nav2 drives, so send a goal or teleoperate
but not both at once. Releasing a key does not stop the robot -- the base runs until the
next command, and LeRobot's host stops it by itself after half a second of silence -- so
this repeats the current command at 10 Hz and zeroes it on exit.

There is nothing here that ros-jazzy-teleop-twist-keyboard would not do; it is only that
apt needs a password and this does not.
"""
import os
import subprocess
import sys
import select
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# x, y, yaw per QWERTY key, scaled by the current speed setting
KEYS = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
    " ": (0.0, 0.0, 0.0),
}
# What those same physical keys type on Dvorak, unshifted and shifted.
DVORAK = str.maketrans("wasdqe-_+=", ",aoe'.[{}]")
# The base is geared for walking pace; these are gentle enough for an indoor first drive.
LINEAR = 0.15  # m/s
ANGULAR = 0.8  # rad/s


def is_dvorak():
    """True if this session types Dvorak. LEKIWI_LAYOUT wins if set."""
    override = os.environ.get("LEKIWI_LAYOUT", "")
    if override:
        return override.lower().startswith("dv")
    # ponytail: any Dvorak in the layout list counts, so a second QWERTY group we are
    # not currently on still reads as Dvorak -- LEKIWI_LAYOUT=qwerty is the way out.
    for cmd in (["setxkbmap", "-query"], ["localectl", "status"]):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=2).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        if "dvorak" in out.lower():
            return True
    return False


def layout():
    """(key -> twist, slower keys, faster keys, help text) for this keyboard."""
    dvorak = is_dvorak()
    table = DVORAK if dvorak else {}
    keys = {k.translate(table): v for k, v in KEYS.items()}
    slower, faster = ("[{", "]}") if dvorak else ("-_", "+=")
    k = {name: key.translate(table) for name, key in
         (("fwd", "w"), ("back", "s"), ("left", "a"), ("right", "d"),
          ("ccw", "q"), ("cw", "e"))}
    text = (
        f"\n{'Dvorak' if dvorak else 'QWERTY'} keys:\n"
        f"    {k['fwd']}/{k['back']}   forward, back        "
        f"{k['left']}/{k['right']}   strafe left, right\n"
        f"    {k['ccw']}/{k['cw']}   turn left, right     space  stop\n"
        f"    {slower[0]}/{faster[0]}   slower, faster       Ctrl-C quit\n"
    )
    return keys, slower, faster, text


def read_key(timeout):
    """One keypress, or None if nothing arrives within timeout seconds."""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def main():
    rclpy.init()
    node = Node("lekiwi_teleop")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    keys, slower, faster, help_text = layout()
    settings = termios.tcgetattr(sys.stdin)
    twist = Twist()
    scale = 1.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(help_text)
        while rclpy.ok():
            key = read_key(0.1)
            if key == "\x03":  # cbreak leaves Ctrl-C to us
                break
            if key is None:  # `None in "-_"` is a TypeError, so say so first
                pass
            elif key in slower:
                scale = max(0.1, scale - 0.1)
                print(f"speed {scale:.1f}\r")
            elif key in faster:
                scale = min(2.0, scale + 0.1)
                print(f"speed {scale:.1f}\r")
            elif key in keys:
                x, y, yaw = keys[key]
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
