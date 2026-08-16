#!/usr/bin/env python3
"""Measure the LeKiwi's odometry scale against a checkerboard of known size.

The wheel odometry LeRobot reports is only as good as its wheel radius and geometry
constants, and those are never exactly right. This drives a short, fixed motion and
compares the distance odometry claims against the distance the camera actually sees,
using a stationary checkerboard as the reference. The camera is rigidly attached to
the base, so its displacement is the base's displacement, and its rotation is the
base's rotation regardless of where on the robot it sits.

Run it once per axis, then put the printed values in the launch:

    ros2 run lekiwi_rmf odom_scale.py --axis linear
    ros2 run lekiwi_rmf odom_scale.py --axis angular

The robot moves. Keep the area clear and stay near the power switch.
"""
import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

BOARD = (8, 6)
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


class OdomScale(Node):
    def __init__(self, square):
        super().__init__("odom_scale")
        self.object_points = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
        self.object_points[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * square
        self.k = None
        self.d = None
        self.frame = None
        self.odom = None
        self.create_subscription(CameraInfo, "/camera/front/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/front/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_info(self, msg):
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.d = np.array(msg.d, dtype=np.float64)

    def on_image(self, msg):
        self.frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    def on_odom(self, msg):
        pose = msg.pose.pose
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        self.odom = (pose.position.x, pose.position.y, yaw)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def camera_pose(self, attempts=40):
        """Camera position and orientation in the board's frame, averaged over frames."""
        positions, yaws = [], []
        for _ in range(attempts):
            self.spin(0.1)
            if self.frame is None or self.k is None:
                continue
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH)
            if not found:
                continue
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
            ok, rvec, tvec = cv2.solvePnP(self.object_points, corners, self.k, self.d)
            if not ok:
                continue
            rotation, _ = cv2.Rodrigues(rvec)
            # solvePnP gives the board in camera coordinates; invert for camera in board
            # coordinates, which is the frame the robot actually moves through.
            positions.append((-rotation.T @ tvec).ravel())
            yaws.append(math.atan2(-rotation.T[1, 0], rotation.T[0, 0]))
            if len(positions) >= 10:
                break
        if len(positions) < 5:
            raise RuntimeError("no pude ver el tablero -- ponlo completo y quieto frente a la camara")
        return np.median(positions, axis=0), float(np.median(yaws))

    def drive(self, twist, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())
        self.spin(1.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--axis", choices=["linear", "angular"], default="linear")
    parser.add_argument("--square", type=float, default=0.025, help="checkerboard square, metres")
    parser.add_argument("--speed", type=float, help="m/s or rad/s (default 0.08 / 0.4)")
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--current-scale", type=float, default=1.0, help="scale the driver runs with now")
    args = parser.parse_args()
    speed = args.speed if args.speed else (0.08 if args.axis == "linear" else 0.4)

    rclpy.init()
    node = OdomScale(args.square)
    try:
        node.spin(2.0)
        start_position, start_yaw = node.camera_pose()
        start_odom = node.odom
        if start_odom is None:
            raise RuntimeError("sin /odom -- el driver no esta publicando")

        twist = Twist()
        if args.axis == "linear":
            twist.linear.x = speed
        else:
            twist.angular.z = speed
        print(f"moviendo {args.seconds:.1f}s a {speed}...", flush=True)
        node.drive(twist, args.seconds)

        end_position, end_yaw = node.camera_pose()
        end_odom = node.odom

        if args.axis == "linear":
            measured = float(np.linalg.norm(end_position - start_position))
            reported = math.hypot(end_odom[0] - start_odom[0], end_odom[1] - start_odom[1])
            name = "xy_velocity_scale"
        else:
            measured = abs(math.atan2(math.sin(end_yaw - start_yaw), math.cos(end_yaw - start_yaw)))
            reported = abs(math.atan2(math.sin(end_odom[2] - start_odom[2]), math.cos(end_odom[2] - start_odom[2])))
            name = "yaw_velocity_scale"

        unit = "m" if args.axis == "linear" else "rad"
        print(f"camara (real): {measured:.4f} {unit}")
        print(f"odometria:     {reported:.4f} {unit}")
        if reported < 1e-3:
            print("la odometria no reporto movimiento -- sube --speed o --seconds")
        elif measured < 1e-3:
            print("la camara no vio movimiento -- el robot no se movio?")
        else:
            print(f"error: {100 * (reported / measured - 1):+.1f}%")
            print(f"{name}: {args.current_scale * measured / reported:.4f}")
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
