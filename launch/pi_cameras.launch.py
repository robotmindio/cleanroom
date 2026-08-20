"""Publish the robot's cameras from the Pi they are plugged into.

The Pi owns the USB cameras, so it owns the ROS nodes that read them. Relaying frames
through the LeRobot host instead would put the cameras back on the critical path of
motor control: the host aborts the whole robot when a frame arrives late, and a USB
webcam does exactly that. Here a stalled camera costs frames and nothing else.

Every topic lives under /pi/, so nothing here can be mistaken for the canonical topics
the rest of the graph uses. Only the compressed image crosses the network -- raw 640x480
at 30 Hz is 27 MB/s, which no robot wifi will carry -- and the workstation expands it
back into /camera/front/image_raw. camera_info is a few hundred bytes and is read from
/pi/camera/front/camera_info directly.

    ros2 launch pi_cameras.launch.py front_device:=/dev/v4l/by-id/usb-...-video-index0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    front_device = LaunchConfiguration("front_device")
    wrist_device = LaunchConfiguration("wrist_device")
    camera_info_url = LaunchConfiguration("camera_info_url")
    jpeg_quality = ParameterValue(LaunchConfiguration("jpeg_quality"), value_type=int)
    no_wrist = PythonExpression(["'", wrist_device, "' == 'none'"])

    return LaunchDescription(
        [
            # Prefer /dev/v4l/by-id/... paths: /dev/videoN is reassigned on every USB
            # re-enumeration, which on this hardware happens often enough to matter.
            DeclareLaunchArgument("front_device", default_value="/dev/video0"),
            DeclareLaunchArgument("wrist_device", default_value="none"),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value=["file://", EnvironmentVariable("HOME"), "/.ros/camera_info/lekiwi_front.yaml"],
            ),
            DeclareLaunchArgument("jpeg_quality", default_value="50"),
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
                    # Default quality 95 costs ~90 KB a frame, 18 Mbit/s at 25 Hz. RTAB-Map
                    # matches features, not pixels, and does not need the last 45 KB.
                    # The leading dot is image_transport's own naming, taken from the
                    # resolved topic; without it the parameter is silently ignored.
                    ".image_raw.compressed.jpeg_quality": jpeg_quality,
                }],
                # A namespace rather than remappings: image_transport builds its extra
                # transport topics from the unremapped base name, so remapping image_raw
                # leaves compressed advertised at /image_raw/compressed.
                namespace="/pi/camera/front",
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
                namespace="/pi/camera/wrist",
                condition=UnlessCondition(no_wrist),
                output="screen",
            ),
        ]
    )
