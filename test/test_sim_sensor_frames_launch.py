"""The production Gazebo bridges must publish canonical ROS sensor frames."""

import time
import uuid
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2


@pytest.mark.launch_test
def generate_test_description():
    lidar = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        parameters=[{"override_frame_id": "laser"}], output="screen",
    )
    camera = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=[
            "/camera/front@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        parameters=[{"override_frame_id": "front_camera_optical_frame"}],
        remappings=[
            ("/camera/front", "/camera/front/image_raw"),
            ("/camera/depth/points", "/camera/depth/points_raw"),
        ], output="screen",
    )
    publish = ExecuteProcess(cmd=["bash", "-c", """
        sleep 2
        for i in {1..8}; do
          gz topic -t /scan -m gz.msgs.LaserScan -p 'header { stamp { sec: 1 } data { key: "frame_id" value: "wrong" } } frame: "wrong" angle_min: -1 angle_max: 1 angle_step: 2 range_min: 0.1 range_max: 10 count: 1 vertical_count: 1 ranges: 1.0 intensities: 1.0'
          gz topic -t /camera/front -m gz.msgs.Image -p 'header { stamp { sec: 1 } data { key: "frame_id" value: "wrong" } } width: 1 height: 1 step: 3 pixel_format_type: RGB_INT8 data: "rgb"'
          gz topic -t /camera/depth/points -m gz.msgs.PointCloudPacked -p 'header { stamp { sec: 1 } data { key: "frame_id" value: "wrong" } } height: 1 width: 0 point_step: 12 row_step: 0 is_dense: true'
        done
    """], output="screen")
    return launch.LaunchDescription([
        SetEnvironmentVariable("GZ_PARTITION", f"lekiwi_sensor_frames_{uuid.uuid4().hex}"),
        lidar, camera, publish, launch_testing.actions.ReadyToTest(),
    ]), {"publish": publish}


class TestSensorFrames(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_bridge_overrides_every_sensor_frame(self):
        node = rclpy.create_node("sim_sensor_frame_test")
        frames = {}
        node.create_subscription(LaserScan, "/scan", lambda msg: frames.setdefault("scan", msg.header.frame_id), 10)
        node.create_subscription(Image, "/camera/front/image_raw", lambda msg: frames.setdefault("rgb", msg.header.frame_id), 10)
        node.create_subscription(PointCloud2, "/camera/depth/points_raw", lambda msg: frames.setdefault("depth", msg.header.frame_id), 10)
        deadline = time.monotonic() + 20
        while len(frames) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        self.assertEqual(frames, {
            "scan": "laser",
            "rgb": "front_camera_optical_frame",
            "depth": "front_camera_optical_frame",
        })


@launch_testing.post_shutdown_test()
class TestPublisherExit(unittest.TestCase):
    def test_publisher_exits_cleanly(self, proc_info, publish):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=publish, allowable_exit_codes=[0, -2, -15]
        )
