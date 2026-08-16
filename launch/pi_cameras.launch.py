"""Publish the robot's cameras from the Pi they are plugged into.

The Pi owns the USB cameras, so it owns the ROS nodes that read them. Relaying frames
through the LeRobot host instead would put the cameras back on the critical path of
motor control: the host aborts the whole robot when a frame arrives late, and a USB
webcam does exactly that. Here a stalled camera costs frames and nothing else.

Images go out on a Pi-local topic and only their compressed form crosses the network --
raw 640x480 at 30 Hz is 27 MB/s, which no robot wifi will carry. The workstation turns
compressed back into the canonical /camera/front/image_raw. camera_info is small enough
to publish straight onto its canonical topic.

    ros2 launch pi_cameras.launch.py front_device:=/dev/v4l/by-id/usb-...-video-index0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    front_device = LaunchConfiguration("front_device")
    wrist_device = LaunchConfiguration("wrist_device")
    camera_info_url = LaunchConfiguration("camera_info_url")
    no_wrist = PythonExpression(["'", wrist_device, "' == 'none'"])

    return LaunchDescription(
        [
            # Prefer /dev/v4l/by-id/... paths: /dev/videoN is reassigned on every USB
            # re-enumeration, which on this hardware happens often enough to matter.
            DeclareLaunchArgument("front_device", default_value="/dev/video0"),
            DeclareLaunchArgument("wrist_device", default_value="none"),
            DeclareLaunchArgument(
                "camera_info_url", default_value="file://${ROS_HOME}/camera_info/lekiwi_front.yaml"
            ),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="front_camera",
                parameters=[{
                    "video_device": front_device,
                    "camera_info_url": camera_info_url,
                    "camera_frame_id": "front_camera_optical_frame",
                    "camera_name": "lekiwi_front",
                    "pixel_format": "YUYV",
                    "output_encoding": "rgb8",
                    "image_size": [640, 480],
                }],
                remappings=[
                    ("image_raw", "/pi/camera/front/image_raw"),
                    ("camera_info", "/camera/front/camera_info"),
                    ("set_camera_info", "/camera/front/set_camera_info"),
                ],
                output="screen",
            ),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="wrist_camera",
                parameters=[{
                    "video_device": wrist_device,
                    "camera_frame_id": "wrist_camera_optical_frame",
                    "camera_name": "lekiwi_wrist",
                    "pixel_format": "YUYV",
                    "output_encoding": "rgb8",
                    "image_size": [640, 480],
                }],
                remappings=[
                    ("image_raw", "/pi/camera/wrist/image_raw"),
                    ("camera_info", "/camera/wrist/camera_info"),
                    ("set_camera_info", "/camera/wrist/set_camera_info"),
                ],
                condition=UnlessCondition(no_wrist),
                output="screen",
            ),
        ]
    )
