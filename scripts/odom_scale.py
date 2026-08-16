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


def find_board(gray):
    """Locate the grid, preferring the detector that survives real lighting.

    findChessboardCornersSB reads a board that the classic detector refuses -- ceiling
    light glaring off the paper, or a board small in frame -- and returns subpixel
    corners without a separate refinement pass. The classic one stays as a fallback for
    OpenCV builds without it.
    """
    found, corners = cv2.findChessboardCornersSB(
        gray, BOARD, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY
    )
    if found:
        return True, corners
    found, corners = cv2.findChessboardCorners(
        gray, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if found:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
    return found, corners


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

    def sample(self):
        """One (camera pose in board frame, odometry pose) pair, or None."""
        if self.frame is None or self.k is None or self.odom is None:
            return None
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        found, corners = find_board(gray)
        if not found:
            return None
        ok, rvec, tvec = cv2.solvePnP(self.object_points, corners, self.k, self.d)
        if not ok:
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        # solvePnP gives the board in camera coordinates; invert for camera in board
        # coordinates, which is the frame the robot actually moves through.
        position = (-rotation.T @ tvec).ravel()
        return position, rotation.T, self.odom

    def drive_sampling(self, twist, seconds, samples):
        """Drive while collecting samples.

        Sampling throughout instead of only at the endpoints is what makes this usable
        on a real floor: a ceiling light glaring off the board kills detection at some
        poses, and demanding a detection at exactly the start and end pose fails the
        whole run. Any two well-separated samples measure the motion between them.
        """
        deadline = time.monotonic() + seconds
        next_sample = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() >= next_sample:
                next_sample = time.monotonic() + 0.2
                found = self.sample()
                if found:
                    samples.append(found)
        self.cmd_pub.publish(Twist())
        for _ in range(15):
            self.spin(0.1)
            found = self.sample()
            if found:
                samples.append(found)


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
        if node.odom is None:
            raise RuntimeError("sin /odom -- el driver no esta publicando")

        samples = []
        for _ in range(15):
            node.spin(0.1)
            found = node.sample()
            if found:
                samples.append(found)

        twist = Twist()
        if args.axis == "linear":
            twist.linear.x = speed
        else:
            twist.angular.z = speed
        print(f"moviendo {args.seconds:.1f}s a {speed}...", flush=True)
        node.drive_sampling(twist, args.seconds, samples)

        if len(samples) < 2:
            raise RuntimeError(
                f"solo {len(samples)} deteccion(es) del tablero -- ponlo completo en el cuadro "
                "y sin reflejo encima"
            )
        (start_position, start_rotation, start_odom) = samples[0]
        (end_position, end_rotation, end_odom) = samples[-1]
        print(f"{len(samples)} detecciones utiles", flush=True)

        if args.axis == "linear":
            measured = float(np.linalg.norm(end_position - start_position))
            reported = math.hypot(end_odom[0] - start_odom[0], end_odom[1] - start_odom[1])
            name = "xy_velocity_scale"
        else:
            # Magnitude of the relative rotation, straight off Rodrigues. Pulling a yaw
            # angle out of the board frame would need the board's axes to line up with
            # the robot's, and a board leaning a few degrees breaks that; the rotation
            # angle itself needs no such assumption.
            relative, _ = cv2.Rodrigues(start_rotation.T @ end_rotation)
            measured = float(np.linalg.norm(relative))
            reported = abs(math.atan2(math.sin(end_odom[2] - start_odom[2]), math.cos(end_odom[2] - start_odom[2])))
            name = "yaw_velocity_scale"

        if args.axis == "angular" and measured < 0.3:
            print(
                f"AVISO: solo {math.degrees(measured):.0f} grados medidos. La orientacion que "
                "solvePnP saca de un tablero plano visto casi de frente se equivoca varios "
                "grados, asi que por debajo de ~20 grados este numero no sirve. Para el giro "
                "mide la separacion entre ruedas: yaw_velocity_scale = 0.125 / (sep / 1.732)."
            )

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
