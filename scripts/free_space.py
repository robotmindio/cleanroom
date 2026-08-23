#!/usr/bin/env python3
"""Turn the front camera into a LaserScan by finding where the floor stops.

    ros2 run lekiwi_rmf free_space.py --ros-args -p camera_height:=0.20 -p camera_pitch:=0.35

One camera cannot measure depth, but it does not have to: everything this robot drives on
is a flat floor, so a pixel of floor has exactly one possible position in the world. That
turns the image into a range sensor -- walk out along each column of the ground plane,
and the first pixel that does not look like floor is an obstacle at a known distance.

Nav2's obstacle layer already subscribes to /scan, so the output drops straight into
navigation with nothing else to configure.

What it assumes, and how it fails:

  - the floor is flat and roughly uniform. Patterned tiles, strong shadows and puddles of
    reflection all read as obstacles. Better a false obstacle than a missed one, but a
    floor with a busy pattern will fence the robot in.
  - anything that looks like the floor is invisible to it -- a grey wall over a grey floor
    especially.
  - it measures where an object *touches the floor*. An overhanging table top reads as the
    distance to its legs.
  - it sees nothing above the top of the image or outside the 41 degree horizontal field
    of view. This is not a substitute for a laser; it is a way to stop before a wall.

Calibrate first -- the geometry is what makes the ranges mean anything:

    ros2 run lekiwi_rmf free_space.py --ros-args -p calibrate:=true

with the printed 8x6 checkerboard lying flat on the floor in view. It prints the camera
height and pitch to pass back in.
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan

BOARD = (8, 6)  # inner corners of scripts/checkerboard.py
SQUARE = 0.025  # m
FAIL_SAFE_HALF_FOV = math.radians(30)


class FreeSpace(Node):
    def __init__(self):
        super().__init__("free_space")
        self.declare_parameter("camera_height", 0.20)
        self.declare_parameter("camera_pitch", 0.30)  # rad, positive tips the lens down
        self.declare_parameter("camera_offset_x", 0.03)  # lens ahead of base_footprint
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("range_min", 0.10)
        self.declare_parameter("range_max", 3.0)
        self.declare_parameter("beams", 60)
        # How far a pixel may drift from the floor's colour before it counts as an
        # obstacle, in Lab units. Lower is twitchier.
        self.declare_parameter("floor_tolerance", 14.0)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("calibrate", False)

        self.k = None
        self.d = None
        self.create_subscription(CameraInfo, "camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "image", self.on_image, qos_profile_sensor_data)
        # Nav2's obstacle layer requests reliable scan delivery. A reliable publisher is
        # compatible with both that subscriber and best-effort visualizers; publishing
        # sensor-data (best effort) here silently left Nav2 with no safety scan at all.
        self.scan_pub = self.create_publisher(
            LaserScan, "scan", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.calibrated = False
        self.last_scan = 0
        self.last_valid_scan = 0
        self.create_timer(0.2, self.watchdog)

    # -- geometry ---------------------------------------------------------------

    def on_info(self, msg):
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64)
        if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            self.k = self.d = None
            self.get_logger().warn("ignoring invalid camera intrinsics")
            return
        self.k, self.d = k, d

    def ground_points(self, rows, cols):
        """Where each pixel of a grid lands on the floor, in metres from the camera.

        A pixel is a ray; the floor is a plane; the intersection is one point. Returns
        (forward, left) arrays and a mask of the pixels whose rays point below the
        horizon at all -- the rest look at the sky and can never hit the floor.
        """
        h = self.get_parameter("camera_height").value
        pitch = self.get_parameter("camera_pitch").value
        fx, fy = self.k[0, 0], self.k[1, 1]
        cx, cy = self.k[0, 2], self.k[1, 2]

        u, v = np.meshgrid(cols.astype(np.float64), rows.astype(np.float64))
        # ray in camera frame: x right, y down, z forward
        x = (u - cx) / fx
        y = (v - cy) / fy
        # rotate by the pitch so that y is measured against a level floor
        cos_p, sin_p = math.cos(pitch), math.sin(pitch)
        down = y * cos_p + sin_p
        forward = cos_p - y * sin_p
        with np.errstate(divide="ignore", invalid="ignore"):
            scale = h / down
            fwd = scale * forward
            left = -scale * x
        visible = (down > 1e-6) & (fwd > 0)
        return fwd, left, visible

    # -- calibration ------------------------------------------------------------

    def calibrate(self, bgr):
        """Camera height and pitch from a checkerboard lying flat on the floor."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        # A board lying flat in front of a nearly level camera is brutally foreshortened:
        # its far rows shrink to a couple of pixels and no detector finds the full grid.
        # It does not matter -- all we want is the plane the board lies in, and any patch
        # of it defines the same plane, at the same scale, because the squares are known.
        size, corners = None, None
        for candidate in [(BOARD[0] - n, BOARD[1] - n) for n in range(BOARD[1] - 2)]:
            found, found_corners = cv2.findChessboardCorners(gray, candidate, None)
            if not found:
                found, found_corners = cv2.findChessboardCornersSB(
                    gray, candidate, cv2.CALIB_CB_EXHAUSTIVE)
            if found:
                size, corners = candidate, found_corners
                break
        if size is None:
            self.get_logger().warn("no checkerboard yet -- lay it flat on the floor, in view")
            return
        if size != BOARD:
            self.get_logger().info(
                f"using a {size[0]}x{size[1]} patch of the board -- the rest is too "
                "foreshortened to resolve")
        corners = cv2.cornerSubPix(
            gray, corners.astype(np.float32), (7, 7), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        object_points = np.zeros((size[0] * size[1], 3), np.float32)
        object_points[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2) * SQUARE
        ok, rvec, tvec = cv2.solvePnP(object_points, corners, self.k, self.d)
        if not ok:
            self.get_logger().warn("checkerboard found but the pose solve failed")
            return
        rot, _ = cv2.Rodrigues(rvec)
        # The board defines the floor: its normal is the floor normal, and the camera's
        # height is its distance from that plane. Camera axes are x right, y down, z
        # forward, so the upward normal is the one with a negative y.
        normal = rot[:, 2]
        if normal[1] > 0:
            normal = -normal
        height = abs(float(normal @ tvec.ravel()))
        # level camera sees the floor normal as (0, -1, 0); pitching down by t turns it
        # into (0, -cos t, -sin t)
        pitch = math.asin(max(-1.0, min(1.0, float(-normal[2]))))
        self.get_logger().info(
            f"camera_height:={height:.3f} camera_pitch:={pitch:.3f}  "
            f"(pitch {math.degrees(pitch):.1f} degrees below level)")
        self.calibrated = True

    # -- detection --------------------------------------------------------------

    def bgr_image(self, msg):
        """Decode the supported ROS encodings without assuming tightly packed RGB."""
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"ignoring unsupported image encoding {msg.encoding!r}")
            return None
        row_bytes = msg.width * 3
        if msg.step < row_bytes or len(msg.data) < msg.step * msg.height:
            self.get_logger().warn("ignoring malformed camera image")
            return None
        rows = np.frombuffer(msg.data, dtype=np.uint8, count=msg.step * msg.height)
        image = rows.reshape(msg.height, msg.step)[:, :row_bytes].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else image

    def on_image(self, msg):
        if self.k is None:
            return
        bgr = self.bgr_image(msg)
        if bgr is None:
            return
        if self.get_parameter("calibrate").value:
            if not self.calibrated:
                self.calibrate(bgr)
            return
        # The camera runs at 25 Hz and Nav2 is happy with far less; skipping frames leaves
        # the CPU to RTAB-Map, which needs every one of them.
        now = self.get_clock().now().nanoseconds
        if now - self.last_scan < 0.2e9:
            return
        self.last_scan = now
        scan = self.scan(bgr, msg.header.stamp)
        if scan is not None:
            self.scan_pub.publish(scan)
            self.last_valid_scan = now

    def watchdog(self):
        now = self.get_clock().now()
        if now.nanoseconds - self.last_valid_scan > self.get_parameter("scan_timeout").value * 1e9:
            self.scan_pub.publish(self.blocked_scan(now.to_msg()))

    def blocked_scan(self, stamp):
        beams = max(2, int(self.get_parameter("beams").value))
        rmin = self.get_parameter("range_min").value
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.get_parameter("frame_id").value
        scan.angle_min = -FAIL_SAFE_HALF_FOV
        scan.angle_max = FAIL_SAFE_HALF_FOV
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (beams - 1)
        scan.range_min = rmin
        scan.range_max = self.get_parameter("range_max").value
        scan.ranges = [rmin + 0.01] * beams
        return scan

    def scan(self, bgr, stamp):
        beams = max(2, int(self.get_parameter("beams").value))
        rmin = self.get_parameter("range_min").value
        rmax = self.get_parameter("range_max").value
        tol = self.get_parameter("floor_tolerance").value
        offset = self.get_parameter("camera_offset_x").value

        # Half resolution: the floor boundary is a coarse thing to find and this turns a
        # per-beam pass over the image from tens of milliseconds into a few.
        bgr = cv2.pyrDown(bgr)
        height, width = bgr.shape[:2]
        rows = np.arange(height) * 2.0
        cols = np.arange(width) * 2.0
        fwd, left, visible = self.ground_points(rows, cols)

        if not visible.any():
            return None

        lab = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2Lab).astype(np.float32)
        # The strip of floor directly under the camera is the reference: if the robot is
        # about to drive into something there, no colour model would have saved it anyway.
        seed = (fwd > rmin) & (fwd < rmin + 0.25) & (np.abs(left) < 0.15) & visible
        if seed.sum() < 50:  # nothing that close in view: fall back to the nearest floor
            seed = visible & (fwd < np.percentile(fwd[visible], 5))
        if seed.sum() < 50:
            return None  # no floor in sight at all: better silence than invented ranges
        reference = np.median(lab[seed], axis=0)
        floor = np.linalg.norm(lab - reference, axis=2) < tol

        # Transform the floor intersections from the camera frame into the scan frame.
        # A LaserScan's bearing and range are both relative to header.frame_id; adding
        # the camera offset to an already computed range is only correct on centreline.
        scan_fwd = fwd + offset
        scan_left = left
        scan_bearing = np.arctan2(scan_left, scan_fwd)
        scan_distance = np.hypot(scan_fwd, scan_left)

        # Cover the field of view the lens actually has rather than a guessed arc.
        half = float(np.nanmax(np.abs(scan_bearing[visible])))
        angles = np.linspace(-half, half, beams)
        ranges = [float("inf")] * beams
        step = angles[1] - angles[0]
        for i, angle in enumerate(angles):
            sector = visible & (np.abs(scan_bearing - angle) < step / 2) & (scan_distance < rmax)
            if not sector.any():
                continue
            blocked = sector & ~floor
            if blocked.any():
                measured = float(scan_distance[blocked].min())
                if rmin <= measured <= rmax:
                    ranges[i] = measured

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.get_parameter("frame_id").value
        scan.angle_min = float(angles[0])
        scan.angle_max = float(angles[-1])
        scan.angle_increment = float(step)
        scan.range_min = float(rmin)
        scan.range_max = float(rmax)
        scan.ranges = ranges
        return scan


def main():
    rclpy.init()
    node = FreeSpace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
