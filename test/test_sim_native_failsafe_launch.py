"""Native Gazebo watchdog must stop actuators without either ROS adapter."""

import uuid

import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import pytest
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from lekiwi_rmf.sim_topics import SIM_ACTUATOR_BRIDGE_ARGUMENTS


@pytest.mark.launch_test
def generate_test_description():
    package = FindPackageShare("lekiwi_rmf")
    gz = Node(
        package="ros_gz_sim",
        executable="gzserver",
        parameters=[{"world_sdf_file": PathJoinSubstitution([
            package, "worlds", "physics_smoke.sdf"
        ])}],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-world", "physics_smoke", "-name", "lekiwi_1",
                   "-string", Command(["python3 -m lekiwi_rmf.sim_sdf"])],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/sim/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            *SIM_ACTUATOR_BRIDGE_ARGUMENTS,
        ],
        remappings=[("/sim/joint_states", "/joint_states")], output="screen",
    )
    smoke = ExecuteProcess(
        cmd=["python3", "-m", "lekiwi_rmf.sim_native_failsafe_smoke",
             "--ros-args", "-p", "use_sim_time:=true"], output="screen",
    )
    return launch.LaunchDescription([
        # Gazebo transport is independent of DDS domains. A fresh partition
        # keeps concurrent runs and a stale server from a failed prior run
        # from receiving this test's commands.
        SetEnvironmentVariable(
            "GZ_PARTITION", f"lekiwi_native_failsafe_{uuid.uuid4().hex}"
        ),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([package, ".."])),
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            PathJoinSubstitution([package, "..", "..", "lib", "lekiwi_rmf"]),
        ),
        gz, spawn, bridge, smoke, launch_testing.actions.ReadyToTest(),
    ]), {"smoke": smoke}


class TestNativeFailsafe(unittest.TestCase):
    def test_client_reports_pass(self, proc_output, smoke):
        proc_output.assertWaitFor(
            "simulation native failsafe smoke passed", process=smoke,
            stream="stdout", timeout=30,
        )


@launch_testing.post_shutdown_test()
class TestNativeFailsafeExit(unittest.TestCase):
    def test_client_passes(self, proc_info, smoke):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=smoke, allowable_exit_codes=[0, -2]
        )
