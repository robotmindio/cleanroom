"""Stationary image observations, using a standard ROS detection boundary."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('lekiwi_rmf')) / 'config/object_perception.yaml')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('image_topic', default_value='/camera/astra/color/image_raw'),
        Node(package='lekiwi_rmf', executable='object_detector', name='object_detector',
             parameters=[config, clock], remappings=[('image', LaunchConfiguration('image_topic')),
                ('source_image', '/robot/perception/source_image'), ('detections', '/robot/perception/detections')], output='screen'),
        Node(package='lekiwi_rmf', executable='object_observations', name='object_observations',
             parameters=[config, clock], remappings=[('image', '/robot/perception/source_image'),
                ('detections', '/robot/perception/detections')], output='screen'),
    ])
