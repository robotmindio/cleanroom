"""One recognizer and one evidence service with explicitly bound camera sources."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from lekiwi_rmf.object_cameras import camera_bindings, config_directory
from lekiwi_rmf.object_recognition import model_runtime


def nodes(context):
    cameras = LaunchConfiguration('cameras').perform(context).split(',')
    camera_bindings(cameras)
    config = config_directory() / 'object_perception.yaml'
    settings = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    environment, prefix = {}, ''
    if settings['model_name'] != 'yolox':
        prefix = str(model_runtime(settings['model_name']))
        environment = {'PYTHONNOUSERSITE': '1'}
    shared = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
              'camera_ids': cameras}
    return [
        Node(package='lekiwi_rmf', executable='object_detector', name='object_detector',
             parameters=[str(config), shared], additional_env=environment, prefix=prefix, output='screen'),
        Node(package='lekiwi_rmf', executable='object_observations', name='object_observations',
             parameters=[str(config), shared], additional_env=environment, prefix=prefix, output='screen'),
    ]


def generate_launch_description():
    deployment = yaml.safe_load((config_directory() / 'object_perception_deployment.yaml').read_text())
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cameras', default_value=','.join(deployment['cameras'])),
        OpaqueFunction(function=nodes),
    ])
