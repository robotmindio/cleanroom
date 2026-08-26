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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare("lekiwi_rmf")
    camera_supervisor = PathJoinSubstitution([package, "scripts", "camera-supervisor.sh"])
    front_device = LaunchConfiguration("front_device")
    wrist_device = LaunchConfiguration("wrist_device")
    camera_info_url = LaunchConfiguration("camera_info_url")
    wrist_camera_info_url = LaunchConfiguration("wrist_camera_info_url")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
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
            DeclareLaunchArgument(
                "wrist_camera_info_url",
                default_value=["file://", EnvironmentVariable("HOME"), "/.ros/camera_info/lekiwi_wrist.yaml"],
            ),
            DeclareLaunchArgument("jpeg_quality", default_value="50"),
            ExecuteProcess(
                cmd=[
                    camera_supervisor,
                    "--device", front_device, "--name", "front_camera",
                    "--namespace", "/pi/camera/front", "--camera-name", "lekiwi_front",
                    "--frame", "front_camera_optical_frame", "--size", "[640, 480]",
                    "--camera-info-url", camera_info_url,
                    # Default quality 50 keeps a 25 Hz stream inside robot Wi-Fi bandwidth.
                    # matches features, not pixels, and does not need the last 45 KB.
                    "--jpeg-quality", jpeg_quality,
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    camera_supervisor,
                    "--device", wrist_device, "--name", "wrist_camera",
                    "--namespace", "/pi/camera/wrist", "--camera-name", "lekiwi_wrist",
                    "--frame", "wrist_camera_optical_frame", "--size", "[352, 288]",
                    "--camera-info-url", wrist_camera_info_url,
                ],
                condition=UnlessCondition(no_wrist),
                output="screen",
            ),
        ]
    )
