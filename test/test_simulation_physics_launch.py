"""Renderer-free end-to-end Gazebo base/odometry/arm acceptance."""

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
    gz_resources = PathJoinSubstitution([package, ".."])
    gz_plugins = PathJoinSubstitution([package, "..", "..", "lib", "lekiwi_rmf"])
    gz = Node(
        package="ros_gz_sim",
        executable="gzserver",
        parameters=[{"world_sdf_file": PathJoinSubstitution([
            package, "worlds", "physics_smoke.sdf"
        ])}],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "physics_smoke", "-name", "lekiwi_1",
            "-string", Command(["python3 -m lekiwi_rmf.sim_sdf"]),
        ],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/sim/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            *SIM_ACTUATOR_BRIDGE_ARGUMENTS,
        ],
        remappings=[("/sim/joint_states", "/joint_states")],
        output="screen",
    )
    base = ExecuteProcess(
        cmd=["python3", "-m", "lekiwi_rmf.sim_omni_controller", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )
    arm = ExecuteProcess(
        cmd=["python3", "-m", "lekiwi_rmf.sim_arm_controller", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )
    smoke = ExecuteProcess(
        cmd=["python3", "-m", "lekiwi_rmf.sim_physics_smoke", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )
    description = launch.LaunchDescription([
        # Gazebo transport is independent of DDS domains. A fresh partition
        # keeps concurrent runs and a stale server from a failed prior run
        # from receiving this test's commands.
        SetEnvironmentVariable(
            "GZ_PARTITION", f"lekiwi_physics_{uuid.uuid4().hex}"
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            gz_resources,
        ),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", gz_plugins),
        gz, spawn, bridge, base, arm, smoke, launch_testing.actions.ReadyToTest(),
    ])
    return description, {"smoke": smoke}


class TestPhysicsSmoke(unittest.TestCase):
    def test_client_reports_pass(self, proc_output, smoke):
        proc_output.assertWaitFor(
            "simulation physics smoke passed",
            process=smoke,
            stream="stdout",
            timeout=45,
        )


@launch_testing.post_shutdown_test()
class TestPhysicsSmokeExit(unittest.TestCase):
    def test_client_passes(self, proc_info, smoke):
        # The active test observes the client's success sentinel first; launch
        # teardown may then deliver SIGINT in the few microseconds before the
        # Python process returns normally. Both outcomes are successful here,
        # while a pre-sentinel failure still fails the active test.
        launch_testing.asserts.assertExitCodes(
            proc_info, process=smoke, allowable_exit_codes=[0, -2]
        )
