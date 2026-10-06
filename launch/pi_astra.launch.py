"""Publish the Astra Pro from the device that physically owns its USB bus."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lekiwi_rmf.launch_validation import astra_serial_from_hardware_config


def _astra_nodes(context):
    package = Path(get_package_share_directory("lekiwi_rmf"))
    serial = astra_serial_from_hardware_config(
        LaunchConfiguration("hardware_config").perform(context), required=True
    )
    return [
        Node(
            package="astra_camera", executable="astra_camera_node", name="astra_pro",
            parameters=[str(package / "config" / "astra_pro.yaml"), {"serial_number": serial}],
            remappings=[
                ("/color/image_raw", "/camera/astra/color/image_raw"),
                ("/color/camera_info", "/camera/astra/color/camera_info"),
                ("/depth/image_raw", "/camera/astra/depth/image_raw"),
                ("/depth/camera_info", "/camera/astra/depth/camera_info"),
                # The full raster cloud is several MiB per frame. Keep it on
                # the USB-owning machine and publish the compact result below.
                ("/depth/points", "/camera/depth/points_raw"),
            ],
            output="screen", respawn=LaunchConfiguration("respawn"), respawn_delay=5.0,
        ),
        Node(
            package="lekiwi_rmf", executable="astra_cloud_filter", name="astra_cloud_filter",
            parameters=[str(package / "config" / "astra_cloud_filter.yaml")],
            output="screen", respawn=LaunchConfiguration("respawn"), respawn_delay=5.0,
        ),
    ]


def generate_launch_description():
    package = Path(get_package_share_directory("lekiwi_rmf"))
    return LaunchDescription([
        DeclareLaunchArgument("hardware_config", default_value=str(package / "config" / "hardware.yaml")),
        DeclareLaunchArgument("respawn", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=_astra_nodes),
    ])
