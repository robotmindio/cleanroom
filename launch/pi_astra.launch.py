"""Publish the Astra Pro from the device that physically owns its USB bus."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from lekiwi_rmf.launch_validation import astra_serial_from_hardware_config


def generate_launch_description():
    package = Path(get_package_share_directory("lekiwi_rmf"))
    serial = astra_serial_from_hardware_config(
        package / "config" / "hardware.yaml", required=True
    )
    return LaunchDescription([
        Node(
            package="astra_camera", executable="astra_camera_node", name="astra_pro",
            parameters=[str(package / "config" / "astra_pro.yaml"), {"serial_number": serial}],
            remappings=[
                ("/color/image_raw", "/camera/astra/color/image_raw"),
                ("/color/camera_info", "/camera/astra/color/camera_info"),
                ("/depth/image_raw", "/camera/astra/depth/image_raw"),
                ("/depth/camera_info", "/camera/astra/depth/camera_info"),
                ("/depth/points", "/camera/depth/points"),
            ],
            output="screen",
        ),
    ])
