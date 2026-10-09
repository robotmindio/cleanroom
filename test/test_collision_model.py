"""Regression checks for the conservative CAD-backed MoveIt collision model."""

import math
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from lekiwi_rmf.arm_trajectory import JOINT_LIMITS, JOINT_VELOCITY_LIMITS


ROOT = pathlib.Path(__file__).parents[1]


def _real_robot() -> ET.Element:
    description = subprocess.check_output(
        ["xacro", str(ROOT / "urdf" / "lekiwi.urdf.xacro"), "sim:=false"], text=True
    )
    return ET.fromstring(description)


def _sim_robot() -> ET.Element:
    description = subprocess.check_output(
        ["xacro", str(ROOT / "urdf" / "lekiwi.urdf.xacro"), "sim:=true"], text=True
    )
    return ET.fromstring(description)


def test_arm_has_complete_link_and_servo_collision_envelopes():
    robot = _real_robot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    expected = {
        "arm_pedestal_collision_proxy",
        "front_camera_collision_proxy",
        "shoulder_collision_proxy",
        "upper_arm_collision_proxy",
        "forearm_collision_proxy",
        "wrist_collision_proxy",
        "roll_collision_proxy",
        "gripper_collision_proxy",
        "shoulder_motor_collision_proxy", "shoulder_holder_collision_proxy",
        "forearm_link_collision_proxy", "forearm_holder_collision_proxy", "roll_holder_collision_proxy",
    }

    assert expected <= links.keys()
    assert all(links[name].find("collision") is not None for name in expected)


def test_distal_arm_checks_physical_chassis_and_mounted_hardware():
    robot = _real_robot()
    base = robot.find("link[@name='base_link']")
    guard = base.find("collision/geometry/mesh")
    visible_guard = base.find("visual/geometry/mesh")
    mount = robot.find("joint[@name='base_footprint_to_base_link']/origin")
    srdf = ET.parse(ROOT / "config" / "lekiwi.srdf").getroot()
    exemptions = {
        frozenset((item.get("link1"), item.get("link2")))
        for item in srdf.findall("disable_collisions")
    }

    assert robot.find("link[@name='arm_workspace_keepout_proxy']") is None
    assert guard.get("filename") == "package://lekiwi_rmf/urdf/chassis_guard.stl"
    assert visible_guard.get("filename") == guard.get("filename")
    assert base.find("collision/origin").get("xyz") == base.find("visual/origin").get("xyz")
    assert np.fromstring(base.find("collision/origin").get("xyz"), sep=" ") == pytest.approx([0, 0, 0.082])
    assert float(base.find("visual/material/color").get("rgba").split()[-1]) > 0
    mesh = np.frombuffer((ROOT / "urdf/chassis_guard.stl").read_bytes()[84:],
                         dtype=np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")]))["vertices"].reshape(-1, 3)
    assert mesh[:, 0].min() == pytest.approx(-0.145)
    assert mesh[:, 0].max() == pytest.approx(-0.003)
    assert np.ptp(mesh[:, 2]) == pytest.approx(0.050)
    assert np.fromstring(mount.get("xyz"), sep=" ") == pytest.approx([0.0, 0.0, 0.0406])
    obstacles = (
        "base_link", "front_camera_collision_proxy", "lidar_collision_proxy",
        "rpi5_stack_collision_proxy", "astra_camera_link",
    )
    distal_links = (
        "forearm_collision_proxy", "wrist_collision_proxy", "roll_collision_proxy",
        "gripper_collision_proxy", "forearm_link_collision_proxy",
        "forearm_holder_collision_proxy", "roll_holder_collision_proxy",
    )
    for obstacle in obstacles:
        for arm_link in distal_links:
            assert frozenset((obstacle, arm_link)) not in exemptions


def test_real_arm_collision_model_includes_ground_keepout():
    real = _real_robot()
    sim = _sim_robot()
    floor = real.find("link[@name='arm_ground_keepout_proxy']/collision")
    box = floor.find("geometry/box")
    box_origin = floor.find("origin")
    mount = real.find("joint[@name='arm_ground_keepout_mount']")
    exemptions = {
        frozenset((item.get("link1"), item.get("link2")))
        for item in ET.parse(ROOT / "config" / "lekiwi.srdf").getroot().findall("disable_collisions")
    }

    assert np.fromstring(box.get("size"), sep=" ") == pytest.approx([2.0, 2.0, 2.0])
    assert box_origin is not None
    box_centre = np.fromstring(box_origin.get("xyz"), sep=" ")
    box_size = np.fromstring(box.get("size"), sep=" ")
    assert box_centre == pytest.approx([0.0, 0.0, -1.0])
    assert box_centre[2] + box_size[2] / 2 == pytest.approx(0.0)
    assert mount.find("parent").get("link") == "base_footprint"
    assert mount.find("child").get("link") == "arm_ground_keepout_proxy"
    assert sim.find("link[@name='arm_ground_keepout_proxy']/collision") is None
    for link in (
        "shoulder_collision_proxy", "upper_arm_collision_proxy", "forearm_collision_proxy",
        "wrist_collision_proxy", "roll_collision_proxy", "gripper_collision_proxy",
    ):
        assert frozenset(("arm_ground_keepout_proxy", link)) not in exemptions


def test_pedestal_upper_arm_and_camera_proxies_match_their_vendored_cad_meshes():
    robot = _real_robot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    for source, proxy in (
        ("so101_base_link", "arm_pedestal_collision_proxy"),
        ("so101_upper_arm_link", "upper_arm_collision_proxy"),
        ("Camera-Model-v3", "front_camera_collision_proxy"),
    ):
        visuals = links[source].findall("visual")
        collisions = links[proxy].findall("collision")
        assert len(collisions) == len(visuals), proxy
        for visual, collision in zip(visuals, collisions):
            assert visual.find("geometry/mesh").attrib == collision.find("geometry/mesh").attrib
            assert visual.find("origin").attrib == collision.find("origin").attrib
        mount = robot.find(f"joint[@name='{proxy}_mount']")
        assert mount.find("parent").get("link") == source
        assert np.fromstring(mount.find("origin").get("xyz"), sep=" ") == pytest.approx([0, 0, 0])


def test_rpi5_stack_box_encloses_plate_carrier_and_table_with_clearance():
    robot = _real_robot()
    joints = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}
    box = robot.find("link[@name='rpi5_stack_collision_proxy']/collision/geometry/box")
    assert joints["rpi5_stack_collision_proxy"].find("parent").get("link") == "rpi5_through_plate"
    half_size = np.fromstring(box.get("size"), sep=" ") / 2
    centre = np.fromstring(joints["rpi5_stack_collision_proxy"].find("origin").get("xyz"), sep=" ")
    for name in ("rpi5_through_plate", "rpi5_usb_carrier", "rpi5_table"):
        offset = np.zeros(3)
        if name != "rpi5_through_plate":
            origin = joints[name].find("origin")
            assert joints[name].find("parent").get("link") == "rpi5_through_plate"
            assert not np.any(np.fromstring(origin.get("rpy"), sep=" "))
            offset = np.fromstring(origin.get("xyz"), sep=" ")
        vertices = _visual_vertices(robot.find(f"link[@name='{name}']/visual")) + offset
        assert np.all(np.abs(vertices - centre) + 0.004 <= half_size + 1e-6), name


def test_folded_link_hulls_enclose_every_cad_part_without_filling_the_whole_link():
    from scipy.spatial import ConvexHull

    robot = _real_robot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    for source, proxy in (
        ("so101_shoulder_link", ("shoulder_motor_collision_proxy", "shoulder_holder_collision_proxy", "shoulder_collision_proxy")),
        ("so101_lower_arm_link", ("forearm_link_collision_proxy", "forearm_holder_collision_proxy", "forearm_collision_proxy")),
        ("so101_wrist_link", ("wrist_collision_proxy",)),
    ):
        visuals = links[source].findall("visual")
        collisions = [collision for name in proxy for collision in links[name].findall("collision")]
        assert len(collisions) == len(visuals)
        for visual, collision in zip(visuals, collisions):
            assert collision.find("origin").attrib == visual.find("origin").attrib
            assert "/urdf/collision/" in collision.find("geometry/mesh").get("filename")
            envelope = _visual_vertices(collision)
            vertices = np.unique(_visual_vertices(visual), axis=0)
            if visual.find("geometry/mesh").get("filename").endswith("native_wrist_flex.stl"):
                assert len(envelope) // 3 < 4000
                continue  # concavity is checked by the live MoveIt folded-pose regression
            hull = ConvexHull(envelope)
            assert len(hull.simplices) < 400
            # Chunk to keep the dense CAD check out of the runtime memory budget.
            for chunk in np.array_split(vertices, 32):
                assert np.max(chunk @ hull.equations[:, :3].T + hull.equations[:, 3]) < 1e-7
            assert np.all(vertices.min(axis=0) - envelope.min(axis=0) < 0.002)
            assert np.all(envelope.max(axis=0) - vertices.max(axis=0) < 0.002)

    for source, proxy in (
        ("so101_gripper_link", ("roll_collision_proxy", "roll_holder_collision_proxy")),
        ("so101_moving_jaw_link", ("gripper_collision_proxy",)),
    ):
        visuals = links[source].findall("visual")
        collisions = [collision for name in proxy for collision in links[name].findall("collision")]
        assert len(collisions) == len(visuals)
        for visual, collision in zip(visuals, collisions):
            assert visual.find("geometry/mesh").attrib == collision.find("geometry/mesh").attrib
            assert visual.find("origin").attrib == collision.find("origin").attrib


def test_srdf_collision_exemptions_reference_real_links_only():
    robot = _real_robot()
    links = {link.attrib["name"] for link in robot.findall("link")}
    srdf = ET.parse(ROOT / "config" / "lekiwi.srdf").getroot()

    for exemption in srdf.findall("disable_collisions"):
        assert exemption.attrib["link1"] in links
        assert exemption.attrib["link2"] in links


def _visual_vertices(visual: ET.Element) -> np.ndarray:
    """Binary-STL visual vertices in their link frame."""
    mesh = visual.find("geometry/mesh")
    data = (ROOT / mesh.get("filename").removeprefix("package://lekiwi_rmf/")).read_bytes()
    assert len(data) == 84 + int.from_bytes(data[80:84], "little") * 50
    vertices = np.frombuffer(data, offset=84, dtype=np.dtype([
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")
    ]))["vertices"].reshape(-1, 3)
    origin = visual.find("origin")
    r, p, y = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])
    vertices = vertices * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
    return vertices @ rotation.T + np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")


def test_tool_frame_adds_no_extra_collision_volume():
    robot = _real_robot()
    assert robot.find("link[@name='tool0']/collision") is None


def _rotation(r: float, p: float, y: float) -> np.ndarray:
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _transform(origin: ET.Element | None) -> np.ndarray:
    matrix = np.eye(4)
    if origin is not None:
        matrix[:3, :3] = _rotation(*np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
        matrix[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
    return matrix


def _link_surface_points(robot: ET.Element, link: str, count: int) -> np.ndarray:
    """Vertices, edge midpoints and centroids of every visual mesh, in the link frame."""
    points = []
    for visual in robot.find(f"link[@name='{link}']").findall("visual"):
        mesh = visual.find("geometry/mesh")
        data = (ROOT / mesh.get("filename").removeprefix("package://lekiwi_rmf/")).read_bytes()
        assert len(data) == 84 + int.from_bytes(data[80:84], "little") * 50
        triangles = np.frombuffer(data, offset=84, dtype=np.dtype([
            ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")
        ]))["vertices"].astype(float)
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        cloud = np.vstack([a, b, c, (a+b)/2, (b+c)/2, (c+a)/2, (a+b+c)/3])
        cloud = cloud * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
        transform = _transform(visual.find("origin"))
        points.append(cloud @ transform[:3, :3].T + transform[:3, 3])
    points = np.vstack(points)
    return points[np.random.default_rng(0).choice(len(points), count, replace=False)]


def test_wrist_and_moving_jaw_meshes_never_touch_at_any_roll_or_gripper_angle():
    """Justifies the SRDF exemption: only their padded proxies overlap when the jaw opens."""
    robot = _real_robot()
    roll = robot.find("joint[@name='arm_wrist_roll']")
    grip = robot.find("joint[@name='arm_gripper']")
    wrist = _link_surface_points(robot, "so101_wrist_link", 4000)
    jaw = _link_surface_points(robot, "so101_moving_jaw_link", 2500)
    wrist_sq = (wrist ** 2).sum(axis=1)

    def limits(joint):
        return float(joint.find("limit").get("lower")), float(joint.find("limit").get("upper"))

    def spin(angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    closest = np.inf
    for roll_angle in np.linspace(*limits(roll), 13):
        for grip_angle in np.linspace(*limits(grip), 5):
            pose = (_transform(roll.find("origin")) @ spin(roll_angle)
                    @ _transform(grip.find("origin")) @ spin(grip_angle))
            moved = jaw @ pose[:3, :3].T + pose[:3, 3]
            squared = wrist_sq[:, None] + (moved ** 2).sum(axis=1) - 2 * wrist @ moved.T
            closest = min(closest, float(np.sqrt(max(squared.min(), 0.0))))
    # Measured 9 mm on the full meshes; sampling error is about 2 mm.
    assert closest > 0.004


def test_wrist_roll_is_bounded_identically_on_hardware_and_simulation():
    real_joint = _real_robot().find("./joint[@name='arm_wrist_roll']")
    assert real_joint is not None
    assert real_joint.attrib["type"] == "revolute"
    limit = real_joint.find("limit")
    assert limit is not None
    assert float(limit.attrib["lower"]) == pytest.approx(-2.74385)
    assert float(limit.attrib["upper"]) == pytest.approx(2.84121)
    assert float(limit.attrib["velocity"]) == pytest.approx(3.0)

    sim_joint = _sim_robot().find("./joint[@name='arm_wrist_roll']")
    assert sim_joint is not None
    assert sim_joint.attrib["type"] == "revolute"
    sim_limit = sim_joint.find("limit")
    assert sim_limit is not None
    assert float(sim_limit.attrib["lower"]) == pytest.approx(-2.74385)
    assert float(sim_limit.attrib["upper"]) == pytest.approx(2.84121)


def test_generated_arm_limits_match_the_runtime_configuration():
    robot = _real_robot()
    for name, (lower, upper) in JOINT_LIMITS.items():
        limit = robot.find(f"./joint[@name='{name}']/limit")
        assert limit is not None
        assert float(limit.get("lower")) == pytest.approx(lower)
        assert float(limit.get("upper")) == pytest.approx(upper)
        assert float(limit.get("velocity")) == pytest.approx(
            JOINT_VELOCITY_LIMITS[name]
        )


def test_rmf_circle_encloses_the_tracked_nav2_polygon():
    nav2 = yaml.safe_load((ROOT / "config" / "nav2_params.yaml").read_text())
    polygon = yaml.safe_load(
        nav2["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    )
    local_parameters = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    global_parameters = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    polygon_radius = max(math.hypot(float(x), float(y)) for x, y in polygon)
    fleet = yaml.safe_load((ROOT / "config" / "fleet_config.yaml").read_text())
    rmf_radius = float(fleet["rmf_fleet"]["profile"]["footprint"])
    bundle = yaml.safe_load(
        (ROOT / "maps" / "bundles" / "cleanroom-development.yaml").read_text()
    )

    assert rmf_radius >= polygon_radius
    assert float(bundle["robot_footprint_radius"]) == rmf_radius
    # The map/RMF radius is the whole live Nav2 envelope, not the raw polygon
    # plus a hidden padding margin.
    assert float(local_parameters["footprint_padding"]) == 0.0
    assert float(global_parameters["footprint_padding"]) == 0.0


def test_moveit_and_rviz_share_tracked_scaling_and_depth_defaults():
    limits = yaml.safe_load((ROOT / "config" / "joint_limits.yaml").read_text())
    rviz = yaml.safe_load((ROOT / "config" / "lekiwi.rviz").read_text())
    planning_display = next(
        display
        for display in rviz["Visualization Manager"]["Displays"]
        if display.get("Class") == "lekiwi_rmf/MotionPlanning"
    )
    sensors = yaml.safe_load((ROOT / "config" / "moveit_sensors.yaml").read_text())

    assert planning_display["Planning Group"] == "arm"
    assert planning_display["Velocity_Scaling_Factor"] == pytest.approx(
        limits["default_velocity_scaling_factor"]
    )
    assert planning_display["Acceleration_Scaling_Factor"] == pytest.approx(
        limits["default_acceleration_scaling_factor"]
    )
    assert sensors["point_cloud"]["point_cloud_topic"] == "/moveit/depth/points_ready"
    assert sensors["point_cloud"]["max_update_rate"] == 0.0
    assert sensors["point_cloud"]["sensor_plugin"] == (
        "occupancy_map_monitor/PointCloudOctomapUpdater"
    )


def test_resting_contacts_only_exempt_the_confirmed_parts():
    srdf = ET.parse(ROOT / "config/lekiwi.srdf").getroot()
    rest = {frozenset((item.get("link1"), item.get("link2")))
            for item in srdf.findall("disable_collisions") if item.get("reason") == "MechanicalRest"}
    assert rest == {
        frozenset(("shoulder_motor_collision_proxy", "forearm_link_collision_proxy")),
        frozenset(("shoulder_motor_collision_proxy", "forearm_holder_collision_proxy")),
        frozenset(("shoulder_holder_collision_proxy", "roll_holder_collision_proxy")),
        frozenset(("shoulder_collision_proxy", "roll_holder_collision_proxy")),
    }
    # In particular, neither distal servo body is exempt against the shoulder.
    exemptions = {frozenset((item.get("link1"), item.get("link2")))
                  for item in srdf.findall("disable_collisions")}
    for shoulder in ("shoulder_collision_proxy", "shoulder_motor_collision_proxy", "shoulder_holder_collision_proxy"):
        for motor in ("forearm_collision_proxy", "roll_collision_proxy"):
            assert frozenset((shoulder, motor)) not in exemptions


def test_travel_stow_fits_navigation_footprint_with_joint_tolerance():
    import itertools
    from scipy.spatial import ConvexHull

    robot = _real_robot()
    parents = {j.find("child").get("link"): j for j in robot.findall("joint")}
    config = yaml.safe_load((ROOT / "config/safety_production.yaml").read_text())["safety_supervisor"]["ros__parameters"]
    pose = dict(zip(config["stow_joint_names"], config["stow_joint_positions"]))
    geometry = {}
    for name in ("so101_shoulder_link", "so101_upper_arm_link", "so101_lower_arm_link",
                 "so101_wrist_link", "so101_gripper_link", "so101_moving_jaw_link"):
        pieces = []
        for visual in robot.find(f"link[@name='{name}']").findall("visual"):
            points = np.unique(_visual_vertices(visual), axis=0)
            pieces.append(points[ConvexHull(points).vertices])
        geometry[name] = np.concatenate(pieces)
    for signs in itertools.product((-1, 1), repeat=6):
        angles = {name: pose[name] + config["stow_tolerance"] * sign for name, sign in zip(pose, signs)}
        for name, points in geometry.items():
            frame, transform = name, np.eye(4)
            while frame != "base_footprint":
                joint = parents[frame]
                motion = np.eye(4)
                if joint.get("type") == "revolute":
                    motion[:3, :3] = _rotation(0, 0, angles[joint.get("name")])
                transform = _transform(joint.find("origin")) @ motion @ transform
                frame = joint.find("parent").get("link")
            world = points @ transform[:3, :3].T + transform[:3, 3]
            # 5 mm model allowance, in addition to the full joint tolerance.
            assert world[:, 0].min() >= -0.22 + 0.005
            assert world[:, 0].max() <= 0.24 - 0.005
            assert np.abs(world[:, 1]).max() <= 0.22 - 0.005
    nav2 = yaml.safe_load((ROOT / "config/nav2_params.yaml").read_text())
    for name in ("local_costmap", "global_costmap"):
        assert yaml.safe_load(nav2[name][name]["ros__parameters"]["footprint"]) == [[.24,.22],[.24,-.22],[-.22,-.22],[-.22,.22]]
