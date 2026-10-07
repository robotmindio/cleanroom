"""RTAB-Map visual SLAM and its sensor preprocessing; included by bringup for visual_slam.

RTAB-Map starts only after a real sensor sample: SLAM waits for the sensor it
maps with, the merged lidar/Astra cloud whenever there is a laser, so one
sensor dropping off USB never holds the map (and Nav2) back.
"""

from contextlib import closing
from pathlib import Path
import sqlite3

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from lekiwi_rmf.launch_gates import gated
from lekiwi_rmf.launch_validation import launch_topology

FRONT_RGB = "/camera/front/image_raw"
FRONT_CAMERA_INFO = "/camera/front/camera_info"
FRONT_DEPTH = "/slam/front_depth/image_raw"


def rtabmap_waits_for_loop(context) -> bool:
    """An empty or one-node RGB map cannot satisfy visual hypothesis testing.

    Before starting the mapper, allow that seed to gain another observation;
    ordinary maps still require a verified closure before appending a session.
    Never erase the database or invent a map-to-odometry transform.
    """
    if LaunchConfiguration("slam_mode").perform(context) != "mapping":
        return True
    database = Path(LaunchConfiguration("rtabmap_database").perform(context)).expanduser().resolve()
    count = 0
    if database.is_file():
        try:
            with closing(sqlite3.connect(database.as_uri() + "?mode=ro", uri=True, timeout=1)) as connection:
                count = connection.execute("SELECT count(*) FROM Node").fetchone()[0]
        except sqlite3.Error as error:
            raise RuntimeError(f"cannot inspect RTAB-Map database before startup: {database}: {error}") from error
    return count > 1


def _slam(context):
    topology = launch_topology(context)
    package = FindPackageShare("lekiwi_rmf")
    database = LaunchConfiguration("rtabmap_database")
    camera_on, lidar_on, dual_rgbd = topology.camera_on, topology.lidar_on, topology.dual_rgbd
    wait_for_loop = rtabmap_waits_for_loop(context)
    actions = [] if wait_for_loop else [LogInfo(
        msg="RTAB-Map empty/one-node seed: allow observations in the same database; "
            "relocalization still requires verified registration")]
    if lidar_on:
        actions.append(Node(
            package="lekiwi_rmf", executable="slam_cloud", name="slam_cloud",
            parameters=[{"use_sim_time": topology.sim, "require_arm_stowed": topology.real and camera_on}],
            additional_env={"OPENBLAS_NUM_THREADS": "1"},
            output="screen",
        ))
    rtabmap = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap",
        ros_arguments=["--log-level", "warn"],
        parameters=[PathJoinSubstitution([package, "config", "rtabmap.yaml"]), {
            "use_sim_time": topology.sim,
            # Keep appearance-based retrieval enabled alongside LiDAR ICP.
            # A restarted odometry session cannot use proximity ICP until it
            # has globally relocalized, so the camera must find that first link.
            "database_path": database,
            "subscribe_rgb": camera_on and not dual_rgbd,
            "Reg/Strategy": "2" if camera_on and lidar_on else "0" if camera_on else "1",
            # OpenGV is absent in the packaged core. Native 3D-to-3D visual
            # registration supports both cameras using their measured depth.
            "Vis/EstimationType": "0" if dual_rgbd else "1",
            "subscribe_depth": camera_on and not dual_rgbd,
            "subscribe_rgbd": dual_rgbd,
            "rgbd_cameras": 0 if dual_rgbd else 1,
            "subscribe_scan_cloud": lidar_on,
            "Rtabmap/MemoryThr": ParameterValue(LaunchConfiguration("rtabmap_wm_nodes"), value_type=str),
            "Mem/IncrementalMemory": str(topology.slam_mapping),
            # ICP proximity closure only searches working memory. Reload the
            # saved scans for lidar mapping so a restart can relocalize against
            # the existing graph.
            "Mem/InitWMWithAllNodes": str(not topology.slam_mapping or lidar_on),
            # A one-node seed needs another observation before a visual closure
            # is possible. Populated maps retain the normal relocalization gate.
            "Rtabmap/StartNewMapOnLoopClosure": str(wait_for_loop).lower(),
        }],
        remappings=[
            ("rgb/image", FRONT_RGB), ("rgb/camera_info", FRONT_CAMERA_INFO),
            ("depth/image", FRONT_DEPTH),
            ("rgbd_images", "/slam/rgbd_images"),
            # Whoever owns /map owns what Nav2 plans against: the map server when
            # a floor plan is supplied, RTAB-Map's own grid when the robot draws one.
            ("odom", "/odom"), ("scan_cloud", "/slam/cloud"),
            ("map", "/rtabmap/map" if topology.static_map else "/map"),
        ],
        output="screen",
        respawn=True, respawn_delay=2.0,
    )
    robot_explorer = Node(
        package="lekiwi_rmf", executable="robot_explorer", name="robot_explorer",
        parameters=[PathJoinSubstitution([package, "config", "exploration.yaml"]), {
            "use_sim_time": topology.sim,
            "database_path": database,
            "navigation_params_file": LaunchConfiguration(
                "navigation_params_file", default=PathJoinSubstitution([package, "config", "nav2_params.yaml"])),
            "mapping_max_bytes": ParameterValue(LaunchConfiguration("rtabmap_mapping_max_bytes"), value_type=int),
            "mapping_max_seconds": ParameterValue(LaunchConfiguration("rtabmap_mapping_max_seconds"), value_type=float),
            "allow_exploration": not (topology.static_map or topology.start_rmf),
        }],
        # Keep watching mapping transitions even when startup selected localization.
        additional_env={"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"},
        output="screen",
    )
    sensor_gate = Node(
        package="lekiwi_rmf", executable="readiness_gate", name="wait_for_slam_sensor",
        parameters=[{
            "kind": "topic",
            "topic": FRONT_DEPTH if camera_on else "/slam/cloud",
            "topic_type": "image" if camera_on else "cloud",
        }],
        output="screen",
    )
    actions.extend(gated(sensor_gate, "SLAM sensor", [rtabmap, robot_explorer]))
    if camera_on:
        # The front camera uses visible range returns. Astra uses its own
        # dense registered depth, sent losslessly from the device at 2 Hz.
        actions.append(Node(
            package="rtabmap_util", executable="pointcloud_to_depthimage", name="slam_front_depth",
            parameters=[{
                "use_sim_time": topology.sim,
                "fixed_frame_id": "odom", "approx": True,
                "decimation": 2, "fill_holes_size": 2, "fill_iterations": 1,
                "fill_holes_error": 0.05, "wait_for_transform": 0.1,
                "qos": 1, "qos_camera_info": 1,
                # 25 Hz calibration must survive the slower range/image
                # interval; five metadata samples cover only 0.2 seconds.
                "topic_queue_size": 5, "sync_queue_size": 30,
            }],
            remappings=[("cloud", "/slam/cloud"), ("camera_info", FRONT_CAMERA_INFO),
                        ("image_raw", FRONT_DEPTH)],
            output="screen",
        ))
    if dual_rgbd:
        # RTAB-Map concatenates equal-sized camera rasters. Astra VGA
        # becomes QVGA, matching the front RGB and its 160x120 depth.
        for camera, source, info_qos, decimation, depth in [
            ("front", "front", 1, 1, FRONT_DEPTH),
            ("astra", "astra/color", 2, 2, "/camera/astra/depth/image_raw"),
        ]:
            actions.append(Node(
                package="rtabmap_sync", executable="rgbd_sync", name=f"slam_{camera}_rgbd",
                parameters=[{
                    "use_sim_time": topology.sim,
                    "approx_sync": True, "approx_sync_max_interval": 0.35,
                    "qos": 1, "qos_camera_info": info_qos,
                    "decimation": decimation, "topic_queue_size": 5, "sync_queue_size": 30,
                }],
                remappings=[("rgb/image", f"/camera/{source}/image_raw"),
                            ("rgb/camera_info", f"/camera/{source}/camera_info"),
                            ("depth/image", depth),
                            ("rgbd_image", f"/slam/{camera}/rgbd_image")],
                output="screen",
            ))
        # The packaged mapper supports RGBDImages arrays, but was built
        # without direct multi-topic RGB-D synchronization. Use its native
        # array synchronizer rather than rebuilding RTAB-Map.
        actions.append(Node(
            package="rtabmap_sync", executable="rgbdx_sync", name="slam_rgbd_views",
            parameters=[{
                "use_sim_time": topology.sim,
                "rgbd_cameras": 2, "qos": 1, "approx_sync": True,
                "approx_sync_max_interval": 0.35,
                "topic_queue_size": 5, "sync_queue_size": 5,
            }],
            remappings=[("rgbd_image0", "/slam/astra/rgbd_image"),
                        ("rgbd_image1", "/slam/front/rgbd_image"),
                        ("rgbd_images", "/slam/rgbd_images")],
            output="screen",
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_slam)])
