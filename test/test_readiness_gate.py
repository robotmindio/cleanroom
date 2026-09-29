"""Keep launch dependency gates tied to concrete ROS message types."""

import importlib.util
import pathlib
import types

import pytest

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from lifecycle_msgs.msg import State
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, JointState, LaserScan, PointCloud2

from lekiwi_rmf.readiness_gate import ReadinessGate, TOPIC_TYPES, topic_qos


ROOT = pathlib.Path(__file__).parents[1]


def test_readiness_gate_supports_the_bringup_dependencies():
    assert set(TOPIC_TYPES) == {"image", "odom", "map", "scan", "cloud"}
    assert '<exec_depend>rtabmap_msgs</exec_depend>' in (ROOT / "package.xml").read_text()


def test_topic_gate_requires_semantically_usable_messages():
    gate = ReadinessGate.__new__(ReadinessGate)
    image = Image()
    gate._on_message(image)
    assert not gate._ready
    image.width, image.height, image.step, image.encoding, image.data = 1, 1, 3, "rgb8", [0, 0, 0]
    gate._on_message(image)
    assert gate._ready

    grid = OccupancyGrid()
    gate._on_message(grid)
    assert not gate._ready
    grid.info.width = grid.info.height = 1
    grid.info.resolution = 0.05
    grid.data = [0]
    gate._on_message(grid)
    assert gate._ready

    odom = Odometry()
    gate._on_message(odom)
    assert not gate._ready
    odom.child_frame_id = "base_footprint"
    odom.pose.pose.orientation.w = 1.0
    gate._on_message(odom)
    assert gate._ready

    scan = LaserScan()
    scan.range_min, scan.range_max = 0.02, 12.0
    scan.ranges = [float("inf"), 0.0]
    gate._on_message(scan)
    assert not gate._ready
    scan.ranges = [float("inf"), 1.5]
    gate._on_message(scan)
    assert gate._ready

    cloud = PointCloud2()
    gate._on_message(cloud)
    assert not gate._ready
    cloud.width, cloud.height, cloud.data = 1, 1, b"\0" * 12
    gate._on_message(cloud)
    assert gate._ready


def test_joint_state_gate_waits_for_fresh_complete_samples():
    gate = ReadinessGate.__new__(ReadinessGate)
    gate._required_joint_names = ("arm_shoulder_lift", "arm_gripper")
    gate._minimum_joint_samples = 2
    gate._joint_samples = 0
    gate._last_joint_stamp = 0
    gate._ready = False
    message = JointState()
    message.name = list(gate._required_joint_names)
    message.position = [0.2, 0.0]

    message.header.stamp.sec = 1
    gate._on_joint_states(message)
    assert not gate._ready
    message.header.stamp.nanosec = 1
    gate._on_joint_states(message)
    assert gate._ready

    message.name = ["arm_shoulder_lift"]
    message.position = [0.2]
    message.header.stamp.nanosec = 2
    gate._on_joint_states(message)
    assert not gate._ready
    assert gate._joint_samples == 0


def test_scan_readiness_matches_best_effort_laser_drivers():
    assert topic_qos("scan").reliability == ReliabilityPolicy.BEST_EFFORT


def test_map_readiness_receives_rtabmaps_latched_grid():
    assert topic_qos("map").durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert topic_qos("image").durability == DurabilityPolicy.VOLATILE


def test_map_gate_requests_the_saved_rtabmap_grid_once():
    class Future:
        def add_done_callback(self, callback):
            self.callback = callback

    class Client:
        def __init__(self):
            self.requests = []

        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.requests.append(request)
            return Future()

    gate = ReadinessGate.__new__(ReadinessGate)
    gate._ready = False
    gate._map_publish_requested = False
    gate._map_publish_client = Client()

    gate._request_map_publication()
    gate._request_map_publication()

    assert len(gate._map_publish_client.requests) == 1
    request = gate._map_publish_client.requests[0]
    assert (request.global_map, request.optimized, request.graph_only) == (True, True, False)


def test_rtabmap_restarts_after_a_runtime_exit():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    node = source.split("rtabmap_node = Node(", 1)[1].split("mapping_guard = ExecuteProcess(", 1)[0]
    assert "respawn=True, respawn_delay=2.0" in node


def test_map_gate_retries_when_the_publish_service_call_fails():
    warnings = []
    gate = ReadinessGate.__new__(ReadinessGate)
    gate._map_publish_requested = True
    gate.get_logger = lambda: types.SimpleNamespace(warning=warnings.append)

    def fail():
        raise RuntimeError("service unavailable")

    gate._map_publication_finished(types.SimpleNamespace(result=fail))

    assert not gate._map_publish_requested
    assert "service unavailable" in warnings[0]


def test_nav_action_is_not_ready_until_lifecycle_node_is_active():
    class Future:
        def __init__(self, state):
            self.state = state

        def done(self):
            return True

        def result(self):
            return types.SimpleNamespace(
                current_state=types.SimpleNamespace(id=self.state)
            )

    gate = ReadinessGate.__new__(ReadinessGate)
    gate._ready = False
    gate._action_client = types.SimpleNamespace(wait_for_server=lambda **_: True)
    gate._lifecycle_client = types.SimpleNamespace(
        wait_for_service=lambda **_: True,
        call_async=lambda _request: Future(State.PRIMARY_STATE_INACTIVE),
    )
    gate._lifecycle_future = None
    gate._check_action()
    gate._check_action()
    assert not gate._ready

    gate._lifecycle_client.call_async = lambda _request: Future(
        State.PRIMARY_STATE_ACTIVE
    )
    gate._check_action()
    gate._check_action()
    assert gate._ready


def test_failed_gate_does_not_start_its_dependents():
    spec = importlib.util.spec_from_file_location("bringup_under_test", ROOT / "launch" / "bringup.launch.py")
    bringup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bringup)

    after_camera = bringup._after_success("camera", ["rtabmap"])
    running_context = types.SimpleNamespace(is_shutdown=False)
    assert after_camera(types.SimpleNamespace(returncode=0), running_context) == ["rtabmap"]

    failure = after_camera(types.SimpleNamespace(returncode=1), running_context)
    assert len(failure) == 1
    assert "dependents remain stopped" in failure[0].msg[0].text


def test_safety_supervisor_is_not_blocked_by_map_relocalization():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    joint_gate = source[source.index("target_action=joint_state_ready_gate"):]
    map_gate = source[source.index("target_action=map_ready_gate"):]
    assert "safety_supervisor_node" in joint_gate.split("RegisterEventHandler", 1)[0]
    assert "safety_supervisor_node" not in map_gate.split("RegisterEventHandler", 1)[0]


def test_shutdown_gate_does_not_start_dependents():
    spec = importlib.util.spec_from_file_location("bringup_under_test", ROOT / "launch" / "bringup.launch.py")
    bringup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bringup)

    after_camera = bringup._after_success("camera", ["rtabmap"])
    assert after_camera(
        types.SimpleNamespace(returncode=0), types.SimpleNamespace(is_shutdown=True)
    ) == []
    assert after_camera(
        types.SimpleNamespace(returncode=130), types.SimpleNamespace(is_shutdown=False)
    ) == []


def test_simulation_base_controller_consumes_only_the_guarded_velocity_topic():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    controller = (ROOT / "lekiwi_rmf" / "sim_omni_controller.py").read_text()
    assert '"/cmd_vel_safe"' in controller
    assert '"/cmd_vel"' not in controller
    assert "/sim/sim_base_left_wheel/cmd_vel" in source
    # Real LD06 and simulated Gazebo scans both pass through body masking.
    assert source.count('executable="scan_self_filter"') == 2
    assert "'/pi/lidar/scan' if " in source
    assert '"topic_name": "/lidar/scan_raw"' in source
    assert '"topic_name": "/scan"' not in source


def test_simulation_uses_a_database_separate_from_the_real_robot():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert "lekiwi_rtabmap_sim.db" in source


def test_moveit_receives_a_lowercase_sim_argument():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert "\"'true' if '\", mode, \"' == 'sim' else 'false'\"" in source


def test_simulation_exports_a_resource_path_for_vendored_cad_meshes():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    assert "SetEnvironmentVariable" in source
    assert "GZ_SIM_RESOURCE_PATH" in source


def test_simulation_selects_ogre2_without_an_unsupported_host_override():
    source = (ROOT / "worlds" / "cleanroom.sdf").read_text()
    assert "<render_engine>ogre2</render_engine>" in source
    assert "render_engine_api_backend" not in source


def test_rviz_replaces_only_the_recorded_instance():
    source = (ROOT / "scripts" / "rviz.sh").read_text()
    assert "rviz_pid_file" in source
    assert "pgrep -f" not in source
    assert 'printf \'%s\\n\' "$$" > "$rviz_pid_file"' in source


def test_camera_supervisor_uses_a_valid_compressed_parameter_name():
    source = (ROOT / "scripts" / "camera-supervisor.sh").read_text()
    assert '"image_raw.compressed.jpeg_quality:=$jpeg_quality"' in source
    assert '".image_raw.compressed.jpeg_quality:=$jpeg_quality"' not in source


def test_camera_safety_path_keeps_only_the_latest_frame():
    relay = (ROOT / "lekiwi_rmf" / "camera_relay.py").read_text()
    free_space = (ROOT / "scripts" / "free_space.py").read_text()
    assert "self.raw_qos = QoSProfile(depth=1" in relay
    assert "camera_qos = QoSProfile(depth=1" in free_space


def test_pi_camera_service_uses_the_low_latency_calibrated_front_size():
    source = (ROOT / "launch" / "pi_cameras.launch.py").read_text()
    assert '"--frame", "front_camera_optical_frame", "--size", "[320, 240]"' in source


def test_long_lived_nodes_have_idempotent_ros_shutdown():
    for relative_path in (
        "lekiwi_rmf/cmd_vel_mux.py",
        "lekiwi_rmf/driver.py",
        "lekiwi_rmf/readiness_gate.py",
    ):
        source = (ROOT / relative_path).read_text()
        assert "rclpy.try_shutdown()" in source
    assert "A timer already in flight" in (ROOT / "lekiwi_rmf/cmd_vel_mux.py").read_text()


def test_interrupted_readiness_gate_is_not_a_successful_dependency():
    source = (ROOT / "lekiwi_rmf/readiness_gate.py").read_text()
    assert "raise SystemExit(130)" in source


def test_real_driver_restart_is_rate_limited():
    source = (ROOT / "launch" / "bringup.launch.py").read_text()
    start = source.index('executable="lekiwi_driver"')
    driver = source[start:source.index("arm_ready_gate", start)]
    assert "respawn_delay=60.0" in driver


def test_readiness_gate_exits_nonzero_when_ros_shuts_down_before_ready(monkeypatch):
    import lekiwi_rmf.readiness_gate as gate

    node = types.SimpleNamespace(_ready=False, destroy_node=lambda: None)
    monkeypatch.setattr(gate, "ReadinessGate", lambda: node)
    monkeypatch.setattr(gate, "rclpy", types.SimpleNamespace(
        init=lambda args=None: None,
        ok=lambda: False,  # the context is already shut down
        spin_once=lambda *_args, **_kwargs: None,
        try_shutdown=lambda: None,
    ))

    with pytest.raises(SystemExit) as exited:
        gate.main()
    assert exited.value.code == 1

    node._ready = True
    gate.main()  # a ready dependency is the only successful exit
