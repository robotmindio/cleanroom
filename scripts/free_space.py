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
import os
import signal

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

BOARD = (8, 6)  # inner corners of scripts/checkerboard.py
SQUARE = 0.025  # m
FAIL_SAFE_HALF_FOV = math.radians(30)


def save_launch_calibration(**values):
    """Atomically retain measured launch parameters without trusting executable config."""
    path = os.environ.get("LEKIWI_LAUNCH_CALIBRATION", os.path.expanduser("~/.ros/lekiwi_launch_calibration.conf"))
    saved = {}
    try:
        with open(path) as source:
            for line in source:
                key, separator, value = line.strip().partition("=")
                if separator and key in {"camera_height", "camera_pitch", "xy_velocity_scale", "yaw_velocity_scale"}:
                    saved[key] = value
    except FileNotFoundError:
        pass
    saved.update({key: f"{value:.6f}" for key, value in values.items()})
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    temporary = f"{path}.tmp"
    with open(temporary, "w") as output:
        for key in ("camera_height", "camera_pitch", "xy_velocity_scale", "yaw_velocity_scale"):
            if key in saved:
                output.write(f"{key}={saved[key]}\n")
    os.replace(temporary, path)
    print(f"Saved launch calibration to {path}", flush=True)


class FreeSpace(Node):
    def __init__(self):
        super().__init__("free_space")
        self.declare_parameter("camera_height", 0.20)
        self.declare_parameter("camera_pitch", 0.30)  # rad, positive tips the lens down
        self.declare_parameter("camera_offset_x", 0.03)  # lens ahead of base_footprint
        self.declare_parameter("camera_offset_y", 0.0)  # lens left of base_footprint
        self.declare_parameter("camera_yaw", 0.0)  # rad, positive turns optical axis left
        self.declare_parameter("camera_roll", 0.0)  # rad, positive rolls image clockwise
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("range_min", 0.10)
        self.declare_parameter("range_max", 3.0)
        self.declare_parameter("beams", 60)
        # How far a pixel may drift from the floor's colour before it counts as an
        # obstacle, in Lab units. Lower is twitchier.
        self.declare_parameter("floor_tolerance", 14.0)
        # A full camera-floor inference can take nearly half a second on the robot
        # while RTAB-Map is also using the CPU.  0.5 s made the watchdog publish a
        # false 11 cm obstacle between otherwise valid scans, permanently blocking
        # Nav2.  This still fails safe promptly when camera input actually stops.
        self.declare_parameter("scan_timeout", 1.5)
        self.declare_parameter("calibrate", False)

        self.k = None
        self.d = None
        self.last_intrinsics_warn = 0
        # The local v4l2 driver publishes these canonical raw topics reliably.  A
        # best-effort subscriber can be starved under image load even while other
        # reliable subscribers receive every frame, leaving the safety watchdog to
        # fence the robot in.  camera_relay publishes the same canonical contract.
        # ``scan()`` is deliberately conservative and can cost hundreds of
        # milliseconds on the Pi. Retaining a deep reliable history then turns
        # a live camera into an old scan, which collision_monitor rightly
        # rejects. Process the newest frame only; a missed frame is safer than
        # planning from its stale predecessor.
        camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CameraInfo, "camera_info", self.on_info, camera_qos)
        self.create_subscription(Image, "image", self.on_image, camera_qos)
        # Nav2's obstacle layer requests reliable scan delivery. A reliable publisher is
        # compatible with both that subscriber and best-effort visualizers; publishing
        # sensor-data (best effort) here silently left Nav2 with no safety scan at all.
        self.scan_pub = self.create_publisher(
            LaserScan, "scan", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        # A small reliable heartbeat lets service supervision distinguish an
        # alive-but-blocked safety scan from a dead Python process without
        # subscribing to the high-bandwidth camera topic.
        self.health_pub = self.create_publisher(
            String, "free_space/health", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.calibrated = False
        self.last_scan = 0
        self.last_valid_scan = 0
        self.last_image = 0
        self.last_scan_failure = "no image received"
        self.last_blocked_warn = 0
        self.create_timer(0.2, self.watchdog)
        self.create_timer(1.0, self.publish_health)

    # -- geometry ---------------------------------------------------------------

    def on_info(self, msg):
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64)
        if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            self.k = self.d = None
            # The driver spams CameraInfo at frame rate; without this gate a
            # missing calibration floods the log at 25 Hz (19k warnings in one
            # 19-hour run) and hides the actual fault.
            now = self.get_clock().now().nanoseconds
            if now - self.last_intrinsics_warn > 10e9:
                self.last_intrinsics_warn = now
                self.get_logger().warn(
                    "ignoring invalid camera intrinsics -- the camera driver is "
                    "publishing an uncalibrated CameraInfo; usually the "
                    "camera_info_url's camera_name does not match the driver's")
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
        u, v = np.meshgrid(cols.astype(np.float64), rows.astype(np.float64))
        # Use the calibrated distortion model before treating pixels as rays.
        # Projecting raw distorted pixels with K alone underestimates/overestimates
        # ranges toward lens edges and can place an obstacle in the wrong beam.
        pixels = np.column_stack((u.ravel(), v.ravel())).reshape(-1, 1, 2)
        normalized = cv2.undistortPoints(pixels, self.k, self.d).reshape(u.shape + (2,))
        # ray in camera frame: x right, y down, z forward
        x = normalized[..., 0]
        y = normalized[..., 1]
        roll = self.get_parameter("camera_roll").value
        x, y = x * math.cos(roll) - y * math.sin(roll), x * math.sin(roll) + y * math.cos(roll)
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
        save_launch_calibration(camera_height=height, camera_pitch=pitch)
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
        self.last_image = self.get_clock().now().nanoseconds
        if self.k is None:
            self.last_scan_failure = "waiting for valid camera intrinsics"
            return
        bgr = self.bgr_image(msg)
        if bgr is None:
            self.last_scan_failure = "unsupported or malformed camera image"
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
            self.last_scan_failure = ""
        else:
            self.last_scan_failure = "floor projection found no safe scan"

    def watchdog(self):
        now = self.get_clock().now()
        if now.nanoseconds - self.last_valid_scan > self.get_parameter("scan_timeout").value * 1e9:
            if now.nanoseconds - self.last_blocked_warn > 5e9:
                self.last_blocked_warn = now.nanoseconds
                image_age = (now.nanoseconds - self.last_image) / 1e9 if self.last_image else float("inf")
                self.get_logger().warn(
                    f"publishing blocked scan: {self.last_scan_failure}; "
                    f"last image {image_age:.2f}s ago")
            self.scan_pub.publish(self.blocked_scan(now.to_msg()))

    def publish_health(self):
        """Publish a bounded-rate liveness/status signal for watchdog consumers."""
        now = self.get_clock().now().nanoseconds
        timeout_ns = int(self.get_parameter("scan_timeout").value * 1e9)
        image_age = (now - self.last_image) / 1e9 if self.last_image else float("inf")
        scan_age = (now - self.last_valid_scan) / 1e9 if self.last_valid_scan else float("inf")
        if self.last_valid_scan and now - self.last_valid_scan <= timeout_ns:
            state = "OK"
        elif self.last_image:
            state = "DEGRADED"
        else:
            state = "NO_CAMERA"
        msg = String()
        msg.data = f"state={state};image_age_s={image_age:.2f};scan_age_s={scan_age:.2f};reason={self.last_scan_failure}"
        self.health_pub.publish(msg)

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
        scan_fwd = fwd
        scan_left = left
        yaw = self.get_parameter("camera_yaw").value
        scan_fwd, scan_left = (
            scan_fwd * math.cos(yaw) - scan_left * math.sin(yaw),
            scan_fwd * math.sin(yaw) + scan_left * math.cos(yaw),
        )
        scan_fwd += offset
        scan_left += self.get_parameter("camera_offset_y").value
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

    def on_sigterm(_signum, _frame):
        # launch escalates to SIGTERM when a node ignores SIGINT for 5s; the
        # default disposition dies uncleanly and leaves DDS participants
        # lingering. Treat it as Ctrl-C so the finally-block runs.
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_sigterm)
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
