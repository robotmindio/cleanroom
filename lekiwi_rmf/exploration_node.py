#!/usr/bin/env python3
"""Explicit, bounded Nav2 exploration and an always-on RTAB-Map quota guard."""

import json
import math
from pathlib import Path
import signal
import threading
import time

import numpy as np
import yaml

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rtabmap_msgs.msg import Info
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from unique_identifier_msgs.msg import UUID

from lekiwi_rmf.action import Explore, Search
from lekiwi_rmf.coverage_search import Profile, SearchSession, normalize_label, select_viewpoint, viewpoint_gain
from lekiwi_rmf.exploration_owner_guard import owned_goal_id
from lekiwi_rmf.exploration import (
    database_size, footprint_is_free, load_navigation_footprint, map_geometry, select_target, task_limits,
    with_body_free, load_task_checkpoint, save_task_checkpoint, task_time,
)
from lekiwi_rmf.navigation_distance import NavigationDistance
from lekiwi_rmf.search_evidence import SearchEvidence


class Paused(RuntimeError):
    """A recoverable fault: stop navigation, wait for recovery, then resume."""


class ResponseTimeout(Paused):
    """A ROS peer did not answer before its deadline."""


class Replan(Exception):
    """New evidence or a map revision invalidated the viewpoint being driven to."""

    def __init__(self, reason, evidence=()):
        super().__init__(reason)
        self.evidence = list(evidence)


class SearchEnded(Exception):
    """A search termination condition: found, or a time/travel budget reached."""

    def __init__(self, outcome, message):
        super().__init__(message)
        self.outcome = outcome


class RobotExplorer(Node):
    def __init__(self, **kwargs):
        super().__init__("robot_explorer", **kwargs)
        config = Path(get_package_share_directory("lekiwi_rmf")) / "config"
        # The tracked exploration profile is the single source of the limits
        # and mapping quota; only machine-local paths are computed here.
        defaults = {
            "allow_exploration": True, "database_path": str(Path.home() / ".ros/lekiwi_rtabmap.db"),
            **yaml.safe_load((config / "exploration.yaml").read_text())["robot_explorer"]["ros__parameters"],
            "navigation_tree": str(config / "explore_nav_to_pose.xml"),
            "navigation_params_file": str(config / "nav2_params.yaml"),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        self._footprint, inscribed, self._region_margin = None, 0.0, 0.0
        if self.config["allow_exploration"]:
            self._footprint, inscribed, self._region_margin = load_navigation_footprint(
                Path(self.config["navigation_params_file"]).expanduser())
        for name, value in self.config.items():
            if type(value) in (float, int) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and positive")
        if (not 0 < self.config["free_threshold"] < 100
                or self.config["clearance_m"] < inscribed - 1e-6
                or self.config["max_radius_m"] <= self._region_margin
                or self.config["observation_distance_m"] <= self.config["clearance_m"]):
            raise ValueError("invalid exploration clearance, observation distance or region")
        self._task_defaults = self.create_publisher(
            String, '/robot/explore/defaults',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._task_defaults.publish(String(data=json.dumps({
            name: self.config[name] for name in ('max_duration_sec', 'max_radius_m')})))
        self.database = Path(self.config["database_path"]).expanduser()
        self._task_file = Path(f"{self.database}.exploration.json")
        self._lock = threading.Lock()
        self._busy = False
        self._nav_uncertain = False
        self._nav_request = self._nav_handle = self._nav_result = None
        self._owned_id = None
        self._guard_ack = None
        self._stage = "idle"
        self._rejection = ""
        self._map = None
        self._inputs = {}
        self._collision_clear_after = 0.0
        self._collision_stop_name = ""
        self._mapping = None
        self._mode_at = 0.0
        self._mode_query_at = 0.0
        self._mapping_started = None
        self._quota_reason = ""
        self._mode_future = self._freeze_future = None
        self._freeze_unconfirmed = False
        self._mode_requested_at = 0.0
        self._freeze_requested_at = 0.0
        self._shutdown_requested = threading.Event()
        self._group = ReentrantCallbackGroup()
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        for topic in ("base_motion_permitted", "arm_stowed"):
            self.create_subscription(Bool, f"/safety/{topic}",
                                     lambda msg, name=topic: self._record(name, msg.data), latched)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_safety_status, 10)
        self._map_revision = 0
        self.create_subscription(Info, "/info", self._on_slam, 1)
        self.create_subscription(Bool, "/robot/explore/navigation_guard_ready",
                                 lambda msg: self._record("navigation_guard", msg.data), 1)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                 lambda msg: self._record("collision_monitor", msg), 1)
        self.create_subscription(UUID, "/robot/explore/navigation_guarded", self._on_guard_ack, 10)
        self._owner_pub = self.create_publisher(UUID, "/robot/explore/navigation_owner", 10)
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(0.2, self._owner_heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._mapping_parameters = AsyncParameterClient(self, "/rtabmap", callback_group=self._group)
        self._mapping_client = self.create_client(Empty, "/rtabmap/set_mode_mapping", callback_group=self._group)
        self._localization_client = self.create_client(Empty, "/rtabmap/set_mode_localization", callback_group=self._group)
        self._navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose", callback_group=self._group)
        self._planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose", callback_group=self._group)
        self._costmap = self.create_client(GetCostmap, "/global_costmap/get_costmap", callback_group=self._group)
        self._action = ActionServer(
            self, Explore, "/robot/explore", self._execute, goal_callback=self._accept,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT, callback_group=self._group,
        )
        self._evidence = self._search_action = None
        if not 0 < self.config["search_target_coverage"] <= 1:
            raise ValueError("search_target_coverage must be within (0, 1]")
        if self.config["search_min_height_m"] >= self.config["search_max_height_m"]:
            raise ValueError("search height band must be increasing")
        if self.config["allow_exploration"] and self.config["allow_search"]:
            # Search observes by moving the base, so it shares exploration's guarded Nav2 path and limits.
            self._evidence = SearchEvidence(self, self._tf, self._group, self.config["search_capture_log"])
            self._odometry = NavigationDistance()
            self.create_subscription(Odometry, "/odom", self._odometry.update, 1)
            self.create_publisher(String, "/robot/search/defaults", QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)).publish(String(data=json.dumps({
                    "max_duration_sec": self.config["max_duration_sec"],
                    "max_travel_m": self.config["search_max_travel_m"],
                    "target_coverage": self.config["search_target_coverage"]})))
            self._search_action = ActionServer(
                self, Search, "/robot/search", self._execute_search, goal_callback=self._accept_search,
                cancel_callback=lambda _goal: CancelResponse.ACCEPT, callback_group=self._group,
            )
        # Wall time quotas remain effective even if simulation time stops.
        self._timer = self.create_timer(0.25, self._monitor, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def destroy_node(self):
        # rclpy Node.destroy_node does not destroy action waitables.
        self._action.destroy()
        if self._search_action is not None:
            self._search_action.destroy()
        self._navigation.destroy()
        self._planner.destroy()
        return super().destroy_node()

    def _record(self, name, value):
        now = time.monotonic()
        if name == "collision_monitor" and value.action_type == CollisionMonitorState.STOP:
            # A single clear scan near a person's foot must not restart movement.
            self._collision_clear_after = now + 1.0
            self._collision_stop_name = value.polygon_name
        self._inputs[name] = (value, now)

    def _on_slam(self, message):
        self._record("slam", message.header)
        if message.loop_closure_id or message.proximity_detection_id:
            # An optimized graph moves earlier map coordinates; plans built on them are revalidated.
            # Search coverage itself follows spatial memory's revisions (SearchEvidence.generation).
            self._map_revision += 1

    def _on_safety_status(self, message):
        for status in message.status:
            if status.name == "lekiwi/safety_supervisor":
                values = {item.key: item.value for item in status.values}
                reason = values.get("latched_faults") or values.get("faults") or status.message
                self._record("safety_reason", reason)

    def _on_map(self, message):
        try:
            self._map = map_geometry(message, self.config["max_map_cells"])
        except ValueError as error:
            self._map = None
            self.get_logger().error(str(error), throttle_duration_sec=5)

    def _mode_response(self, future, requested_at):
        if future.cancelled():
            return  # A slow reply proves nothing about the mode; it only lets the last reading age.
        with self._lock:
            if requested_at < self._mode_query_at:
                return  # A delayed older query cannot overwrite a verified mode change.
            self._mode_query_at = requested_at
            try:
                value = future.result().values[0]
                if value.type == 1:
                    mapping = value.bool_value
                elif value.type == 4 and value.string_value.lower() in {"true", "false"}:
                    mapping = value.string_value.lower() == "true"
                else:
                    raise ValueError("RTAB-Map did not return Mem/IncrementalMemory")
                now = time.monotonic()
                if mapping and self._mapping_started is None:
                    self._mapping_started = now
                    self._quota_reason = ""
                if not mapping:
                    self._mapping_started = None
                self._mapping, self._mode_at = mapping, now
            except Exception as error:
                self._mapping = None
                self.get_logger().error(f"cannot read mapping mode: {error}", throttle_duration_sec=5)

    def _monitor(self):
        self._monitor_mapping()
        reason = self._readiness_reason()
        status = DiagnosticStatus(name="lekiwi/exploration", hardware_id="lekiwi",
                                  level=DiagnosticStatus.WARN if reason else DiagnosticStatus.OK,
                                  message=reason or "ready")
        status.values = [KeyValue(key="stage", value=self._stage),
                         KeyValue(key="busy", value=str(self._busy).lower()),
                         KeyValue(key="prerequisite_reason", value=self._readiness_reason(require_permission=False)),
                         KeyValue(key="last_rejection", value=self._rejection)]
        message = DiagnosticArray(status=[status])
        message.header.stamp = self.get_clock().now().to_msg()
        self._diagnostics.publish(message)

    def _on_guard_ack(self, message):
        self._guard_ack = (bytes(message.uuid), time.monotonic())

    def _owner_heartbeat(self):
        owned = self._owned_id
        if owned is not None:
            self._owner_pub.publish(owned)

    def _monitor_mapping(self):
        now = time.monotonic()
        if self._mapping_parameters.services_are_ready():
            if self._mode_future is not None and not self._mode_future.done():
                if now - self._mode_requested_at > self.config["service_timeout_sec"]:
                    # Ask again; the last confirmed reading ages out through _mode_at.
                    self._mode_future.cancel()
                    self._mode_future = None
            if self._mode_future is None or self._mode_future.done():
                self._mode_requested_at = now
                self._mode_future = self._mapping_parameters.get_parameters(["Mem/IncrementalMemory"])
                self._mode_future.add_done_callback(lambda future, sent=now: self._mode_response(future, sent))
        # Mode replies cannot restart a session midway through an old freeze decision.
        with self._lock:
            if (self._freeze_future is not None and not self._freeze_future.done()
                    and now - self._freeze_requested_at > self.config["service_timeout_sec"]):
                self._freeze_future.cancel()
                self._freeze_future = None
            try:
                reason = self._quota_limit(now) if self._mapping_started is not None else ""
            except OSError as error:
                reason = f"cannot measure mapping storage: {error}"
            if reason:
                self._quota_reason = reason
            if reason or self._freeze_unconfirmed:
                if self._localization_client.service_is_ready() and (
                        self._freeze_future is None or self._freeze_future.done()):
                    self.get_logger().warning(f"freezing RTAB-Map: {self._quota_reason}", throttle_duration_sec=5)
                    self._freeze_unconfirmed = True
                    self._freeze_future = self._localization_client.call_async(Empty.Request())
                    self._freeze_requested_at = now
                    self._freeze_future.add_done_callback(self._check_freeze)

    def _check_freeze(self, future):
        if future.cancelled():
            return
        try:
            future.result()
            with self._lock:
                if future is self._freeze_future:
                    self._freeze_unconfirmed = False
        except Exception as error:
            self.get_logger().error(f"cannot freeze RTAB-Map: {error}")

    def _quota_limit(self, now):
        started = self._mapping_started
        if database_size(self.database) >= self.config["mapping_max_bytes"]:
            return "mapping database quota reached"
        if (started is not None and now - started >= self.config["mapping_max_seconds"]):
            return "mapping session duration reached"
        return ""

    def _pose(self):
        try:
            transform = self._tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except TransformException as error:
            raise Paused(f"map-to-robot transform unavailable: {error}") from error
        stamp = rclpy.time.Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        t, q = transform.transform.translation, transform.transform.rotation
        if (not -0.3 <= age <= self.config["data_timeout_sec"]
                or not all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w))):
            raise Paused("map-to-robot transform is stale or non-finite")
        pose = PoseStamped(header=transform.header)
        pose.pose.position.x, pose.pose.position.y = t.x, t.y
        pose.pose.orientation = q
        return pose

    def _healthy(self, require_permission=True):
        now = time.monotonic()
        if self._map is None:
            raise Paused("no valid occupancy map")
        for name in ("base_motion_permitted", "arm_stowed", "slam"):
            if name == "base_motion_permitted" and not require_permission:
                continue
            value, received = self._inputs.get(name, (None, 0))
            timeout = self.config["slam_timeout_sec"] if name == "slam" else self.config["data_timeout_sec"]
            if value is None:
                raise Paused(f"missing {name}")
            if now - received > timeout:
                raise Paused(f"stale {name}")
            if not value:
                reason, seen = self._inputs.get("safety_reason", ("", 0))
                detail = f": {reason}" if name == "base_motion_permitted" and reason and now - seen <= timeout else ""
                raise Paused(f"denied {name}{detail}")
            if name == "slam":
                age = (self.get_clock().now() - rclpy.time.Time.from_msg(value.stamp)).nanoseconds / 1e9
                if not -0.3 <= age <= timeout:
                    raise Paused("SLAM observations are stale")
        if self._mapping is None or now - self._mode_at > self.config["service_timeout_sec"]:
            raise Paused("mapping mode is unknown or stale")
        value, seen = self._inputs.get("navigation_guard", (False, 0.0))
        if self.config["allow_exploration"] and (not value or now - seen > self.config["data_timeout_sec"]):
            raise Paused("navigation ownership guard unavailable or stopping an orphaned goal")
        if self.config["allow_exploration"]:
            state, seen = self._inputs.get("collision_monitor", (None, 0.0))
            if state is None or now - seen > self.config["data_timeout_sec"]:
                raise Paused("collision monitor state is unavailable or stale")
            if require_permission and (state.action_type == CollisionMonitorState.STOP
                    or now < self._collision_clear_after):
                raise Paused(f"collision monitor stop: {self._collision_stop_name}")

    def _footprint_clear(self, pose, grid=None):
        q, p = pose.pose.orientation, pose.pose.position
        grid = self._map if grid is None else grid
        if (grid is None or self._footprint is None
                or not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w))
                or abs(q.x) > 1e-6 or abs(q.y) > 1e-6
                or abs(q.z * q.z + q.w * q.w - 1) > 1e-3):
            return False
        return footprint_is_free(*grid, (p.x, p.y), 2 * math.atan2(q.z, q.w),
                                 self._footprint, self.config["free_threshold"])

    def _route_clear(self, waypoints, start, grid, center, radius):
        measured_start = False
        for index, waypoint in enumerate(waypoints):
            p, q = waypoint.pose.position, waypoint.pose.orientation
            # The lattice's first heading is quantized; validate the measured starting body instead.
            if (index == 0 and math.hypot(p.x - start.pose.position.x, p.y - start.pose.position.y) <= grid[1] / 2
                    and all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w))
                    and abs(q.x) <= 1e-6 and abs(q.y) <= 1e-6
                    and abs(q.z * q.z + q.w * q.w - 1) <= 1e-3):
                waypoint = start
                p = waypoint.pose.position
                measured_start = True
            if (math.hypot(p.x - center[0], p.y - center[1]) >= radius - self._region_margin
                    or not self._footprint_clear(waypoint, grid)):
                return False
            if index == 1 and measured_start:
                yaw = 2 * math.atan2(start.pose.orientation.z, start.pose.orientation.w)
                turn = math.remainder(2 * math.atan2(q.z, q.w) - yaw, 2 * math.pi)
                distance = math.hypot(p.x - start.pose.position.x, p.y - start.pose.position.y)
                corner_radius = max(math.hypot(x, y) for x, y in self._footprint)
                steps = math.ceil((distance + corner_radius * abs(turn)) / (grid[1] / 4))
                # Check the connection from the measured body to the remaining lattice route.
                for step in range(1, steps):
                    fraction = step / steps
                    sample = PoseStamped()
                    sample.pose.position.x = start.pose.position.x + fraction * (p.x - start.pose.position.x)
                    sample.pose.position.y = start.pose.position.y + fraction * (p.y - start.pose.position.y)
                    sample.pose.orientation.z = math.sin((yaw + fraction * turn) / 2)
                    sample.pose.orientation.w = math.cos((yaw + fraction * turn) / 2)
                    if not self._footprint_clear(sample, grid):
                        return False
        return True

    def _accept(self, request):
        try:
            if self._shutdown_requested.is_set():
                raise RuntimeError("exploration server is shutting down")
            if not self.config["allow_exploration"]:
                raise RuntimeError("exploration is disabled for fixed-map/RMF operation")
            _, radius = task_limits(request.max_duration_sec, request.max_radius_m,
                                    self.config["max_duration_sec"], self.config["max_radius_m"])
            if radius <= self._region_margin:
                raise ValueError("exploration radius is too small for the footprint")
            if request.resume_task_id:
                state = self._resume_task(request.resume_task_id)
                if (request.max_duration_sec or request.max_radius_m
                        or request.revisit_known != state["revisit_known"]):
                    raise ValueError("resume must retain the saved task's bounds and revisit setting")
            with self._lock:
                reason = self._readiness_reason()
                if reason:
                    raise RuntimeError(reason)
                if self._busy or self._nav_uncertain:
                    raise RuntimeError("another goal is active or its navigation stop is unconfirmed")
                self._busy = True
                if self._mapping is False:
                    self._quota_reason = ""
                self._rejection = ""
                self._stage = "starting"
            return GoalResponse.ACCEPT
        except (RuntimeError, ValueError, OSError, TransformException) as error:
            self._rejection = str(error)
            self.get_logger().warning(f"exploration rejected: {error}")
            return GoalResponse.REJECT

    def _resume_task(self, identifier):
        return load_task_checkpoint(
            self._task_file, identifier, maximum_duration=self.config["max_duration_sec"],
            maximum_radius=self.config["max_radius_m"], region_margin=self._region_margin,
            maximum_points=self.config["max_map_cells"])

    def _readiness_reason(self, require_permission=True, mapping_quota=True):
        try:
            if self._shutdown_requested.is_set():
                return "exploration server is shutting down"
            if not self.config["allow_exploration"]:
                return "exploration disabled for fixed-map/RMF operation"
            self._healthy(require_permission=require_permission)
            self._pose()
            if (not self._navigation.server_is_ready() or not self._planner.server_is_ready()
                    or not self._costmap.service_is_ready()
                    or not self._mapping_client.service_is_ready()
                    or not self._localization_client.service_is_ready()):
                return "Nav2 or mapping services are unavailable"
            if self._nav_uncertain:
                return "previous navigation stop is unconfirmed"
            if not mapping_quota:
                return ""
            if self._freeze_unconfirmed or (self._freeze_future is not None and not self._freeze_future.done()):
                return "waiting for mapping quota freeze to finish"
            return self._quota_limit(time.monotonic())
        except (RuntimeError, OSError, TransformException) as error:
            return str(error)

    def _checkpoint(self, goal, deadline, center, radius, mapping_quota=True):
        if self._shutdown_requested.is_set():
            raise RuntimeError("exploration server is shutting down")
        if goal.is_cancel_requested:
            raise InterruptedError("exploration canceled")
        if task_time() >= deadline:
            raise TimeoutError("exploration duration reached")
        # Searching a known map does not grow it; the idle quota guard still freezes RTAB-Map.
        reason = (self._quota_reason or self._quota_limit(time.monotonic())) if mapping_quota else ""
        if reason:
            raise RuntimeError(reason)
        self._healthy()
        pose = self._pose()
        xy = pose.pose.position
        if math.hypot(xy.x - center[0], xy.y - center[1]) >= radius - self._region_margin:
            raise RuntimeError("exploration region stopping margin reached")
        return pose

    def _wait(self, future, timeout, check=None):
        deadline = time.monotonic() + timeout
        while not future.done():
            if check:
                check()
            if time.monotonic() >= deadline:
                raise ResponseTimeout("ROS response deadline exceeded")
            if not rclpy.ok(context=self.context):
                raise RuntimeError("ROS is shutting down")
            time.sleep(0.05)
        if check:
            check()
        return future.result()

    def _set_mapping(self, mapping):
        client = self._mapping_client if mapping else self._localization_client
        if not client.service_is_ready():
            raise RuntimeError("mapping mode service is unavailable")
        self._wait(client.call_async(Empty.Request()), self.config["service_timeout_sec"])
        requested_at = time.monotonic()
        response = self._mapping_parameters.get_parameters(["Mem/IncrementalMemory"])
        self._wait(response, self.config["service_timeout_sec"])
        self._mode_response(response, requested_at)
        if self._mapping is not mapping:
            raise RuntimeError("RTAB-Map did not confirm the requested mapping mode")

    def _cancel_late(self, future):
        try:
            handle = future.result()
            if handle.accepted:
                handle.cancel_goal_async()
                handle.get_result_async().add_done_callback(self._late_stopped)
            else:
                with self._lock:
                    self._owned_id = None
                    self._nav_uncertain = False
        except Exception as error:
            self.get_logger().error(f"late navigation cancellation failed: {error}")

    def _late_stopped(self, future):
        try:
            if future.result().status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                          GoalStatus.STATUS_ABORTED):
                with self._lock:
                    self._owned_id = None
                    self._nav_uncertain = False
        except Exception as error:
            self.get_logger().error(f"late navigation stop is unconfirmed: {error}")

    def _stop_navigation(self):
        if self._nav_request is None:
            if not self._nav_uncertain:
                self._owned_id = None
            return
        self._nav_uncertain = True
        try:
            handle = self._nav_handle or self._wait(self._nav_request, self.config["service_timeout_sec"])
            if handle.accepted:
                result = self._nav_result or handle.get_result_async()
                if not result.done():
                    self._wait(handle.cancel_goal_async(), self.config["service_timeout_sec"])
                response = self._wait(result, self.config["service_timeout_sec"])
                if response.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                           GoalStatus.STATUS_ABORTED):
                    raise RuntimeError("Nav2 did not confirm a terminal status")
            self._nav_uncertain = False
            self._owned_id = None
        except Exception as error:
            # Preserve ownership until a late acceptance/result is canceled.
            # An unconfirmed stop always ends the task, never pauses it.
            self._nav_request.add_done_callback(self._cancel_late)
            self._owned_id = None
            raise RuntimeError(f"navigation stop unconfirmed: {error}") from error
        finally:
            self._nav_request = self._nav_handle = self._nav_result = None

    def _pause(self, goal, check, reason, progress, feedback=Explore.Feedback):
        """Hold the robot still until every input recovers.

        Only the task's own end conditions (cancel, quota, region,
        shutdown) or an unconfirmed navigation stop end it while paused.
        """
        self._stop_navigation()
        self.get_logger().warning(f"exploration paused: {reason}")
        self._stage = f"paused: {reason}"
        goal.publish_feedback(feedback(stage=self._stage, **progress()))
        while True:
            try:
                check()
                break
            except Paused as current:
                stage = f"paused: {current}"
                if stage != self._stage:
                    self._stage = stage
                    goal.publish_feedback(feedback(stage=stage, **progress()))
                time.sleep(0.2)
        self.get_logger().info("exploration resumed")

    def _execute(self, goal):
        result = Explore.Result()
        previous_mapping = self._mapping
        visited, blocked = [], []
        mapping_requested = False
        state = None
        try:
            duration, radius = task_limits(goal.request.max_duration_sec, goal.request.max_radius_m,
                                           self.config["max_duration_sec"], self.config["max_radius_m"])
            pose = self._pose()
            center = (pose.pose.position.x, pose.pose.position.y)
            deadline = task_time() + duration
            if goal.request.resume_task_id:
                state = self._resume_task(goal.request.resume_task_id)
                center, radius, deadline = state["center"], state["radius"], state["deadline"]
                visited, blocked = state["visited"], state["blocked"]
                previous_mapping = state["previous_mapping"]
                result.mapped_area_m2 = state["mapped_area_m2"]
            else:
                state = {"schema": 1, "id": bytes(goal.goal_id.uuid).hex(),
                         "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                         "center": center, "radius": radius, "deadline": deadline,
                         "revisit_known": goal.request.revisit_known, "retrying": False,
                         "previous_mapping": previous_mapping}
            mapping_confirmed = False

            def check():
                current = self._checkpoint(goal, deadline, center, radius)
                if mapping_confirmed and self._mapping is not True:
                    raise RuntimeError("mapping mode changed during exploration")
                return current

            def progress():
                return {"visited_targets": len(visited), "unreachable_targets": len(blocked),
                        "mapped_area_m2": result.mapped_area_m2}

            def save():
                state.update(visited=visited, blocked=blocked, mapped_area_m2=result.mapped_area_m2)
                save_task_checkpoint(self._task_file, state)

            save()
            while True:
                try:
                    if not mapping_confirmed:
                        check()
                        mapping_requested = True
                        self._set_mapping(True)
                        mapping_confirmed = True
                    if not self._explore_step(goal, check, center, radius, visited, blocked, result):
                        if blocked and not state["retrying"]:
                            # Retry once after other observations; temporary obstacles can clear.
                            state["retrying"] = True
                            blocked.clear()
                            save()
                            self._stage = "retrying_failed_targets"
                            goal.publish_feedback(Explore.Feedback(stage=self._stage, **progress()))
                            until = time.monotonic() + self.config["settle_sec"]
                            while time.monotonic() < until:
                                check()
                                time.sleep(0.05)
                            continue
                        break
                    save()
                except Paused as reason:
                    paused_at = task_time()
                    state["paused_at"] = paused_at
                    save()

                    def paused_check():
                        nonlocal deadline, paused_at
                        now = task_time()
                        deadline += now - paused_at
                        paused_at = now
                        return check()

                    self._pause(goal, paused_check, reason, progress)
                    state["deadline"] = deadline
                    state.pop("paused_at")
                    save()
        except Exception as error:
            result.message = str(error)
            self.get_logger().warning(f"exploration ended: {error}")
        finally:
            for cleanup in (self._stop_navigation,
                            lambda: self._set_mapping(False) if mapping_requested and previous_mapping is False else None):
                try:
                    cleanup()
                except Exception as error:
                    result.complete = False
                    result.message += f"; cleanup failed: {error}"
                    self.get_logger().error(result.message)
            if state is not None and (not self._shutdown_requested.is_set() or goal.is_cancel_requested):
                try:
                    self._task_file.unlink(missing_ok=True)
                except OSError as error:
                    result.complete = False
                    result.message += f"; checkpoint cleanup failed: {error}"
                    self.get_logger().error(result.message)
            result.visited_targets, result.unreachable_targets = len(visited), len(blocked)
            if self._nav_uncertain:
                result.complete = False
                result.message += "; navigation stop unconfirmed; further exploration refused"
            if goal.is_cancel_requested and not self._nav_uncertain:
                goal.canceled()
            elif result.complete:
                goal.succeed()
            else:
                goal.abort()
            with self._lock:
                self._busy = False
                self._stage = "idle"
        return result

    def _search_profile(self):
        c = self.config
        return Profile(min_object_m=c["search_min_object_m"], min_object_px=c["search_min_object_px"],
                       max_range_m=c["search_max_range_m"], min_height_m=c["search_min_height_m"],
                       max_height_m=c["search_max_height_m"], max_age_sec=c["search_max_age_sec"],
                       require_stationary=c["search_require_stationary"])

    def _search_limits(self, request):
        """Validate untrusted goal fields; zero selects a tracked default and larger limits are refused."""
        label = normalize_label(request.query)
        if not 1 <= len(label) <= 128:
            raise ValueError("search query must be a label of 1..128 characters")
        duration, radius = task_limits(request.max_duration_sec, 0.0, self.config["max_duration_sec"],
                                       self.config["max_radius_m"])
        travel, maximum = request.max_travel_m, self.config["search_max_travel_m"]
        if not math.isfinite(travel) or not 0 <= travel <= maximum:
            raise ValueError(f"max_travel_m must be zero or within (0, {maximum}]")
        target = request.target_coverage
        if not math.isfinite(target) or not 0 <= target <= 1:
            raise ValueError("target_coverage must be zero or within (0, 1]")
        region = (request.region_x, request.region_y, request.region_radius_m)
        if not all(math.isfinite(v) for v in region) or not 0 <= region[2] <= 2 * self.config["max_radius_m"]:
            raise ValueError("search region must be finite with a radius up to twice max_radius_m")
        return label, duration, radius, travel or maximum, target or self.config["search_target_coverage"]

    def _accept_search(self, request):
        try:
            if self._shutdown_requested.is_set():
                raise RuntimeError("exploration server is shutting down")
            self._search_limits(request)
            with self._lock:
                reason = self._readiness_reason(mapping_quota=False)
                if reason:
                    raise RuntimeError(reason)
                if self._busy or self._nav_uncertain:
                    raise RuntimeError("another goal is active or its navigation stop is unconfirmed")
                self._busy = True
                self._rejection = ""
                self._stage = "starting"
            return GoalResponse.ACCEPT
        except (RuntimeError, ValueError, OSError, TransformException) as error:
            self._rejection = str(error)
            self.get_logger().warning(f"search rejected: {error}")
            return GoalResponse.REJECT

    def _execute_search(self, goal):
        """Choose viewpoints by information gain and observe until found, covered or out of budget."""
        result = Search.Result(outcome="failed")
        config, profile = self.config, self._search_profile()
        session, visited, blocked, unproductive = None, [], [], []
        state = {"serial": 0, "plan": None, "dwell": None, "revision": self._map_revision,
                 "generation": self._evidence.generation,
                 "reason": "initial plan", "evidence": []}
        odometry = {"epoch": None, "base": 0.0, "last": 0.0, "before": 0.0}
        started_task = task_time()

        def now():
            return self.get_clock().now().nanoseconds / 1e9

        def progress():
            fraction = session.fraction(session.masks(self._map[0])[0], now()) if session and self._map else 0.0
            return {"coverage_fraction": fraction, "viewpoints_visited": len(visited),
                    "viewpoints_unreachable": len(blocked), "evidence_ids": session.evidence_ids() if session else []}

        def travelled():
            meter = self._odometry.status()
            if not meter["ready"]:
                raise Paused(f"odometry unavailable for the travel budget: {meter['reason']}")
            if meter["epoch"] != odometry["epoch"]:
                # A reset odometry epoch loses at most the motion within one sample gap.
                if odometry["epoch"] is not None:
                    odometry["before"] += odometry["last"] - odometry["base"]
                odometry.update(epoch=meter["epoch"], base=meter["meters"])
            odometry["last"] = meter["meters"]
            return odometry["before"] + odometry["last"] - odometry["base"]

        def weights():
            searchable, _ = session.masks(self._map[0])
            return session.coverage.weights(searchable, now(), config["search_prior_weight"])

        def integrate():
            if self._map is None:
                return []
            session.rebase(self._map)
            generation, revision, log = self._evidence.snapshot()
            if generation != state["generation"]:
                # SLAM revised the map: earlier evidence moves with it before new captures are added.
                state["generation"] = generation
                state["serial"] = log[-1][0] if log else state["serial"]
                if session.reproject([capture for _, capture in log], self._map[0], revision):
                    raise SearchEnded("found", f'found "{session.label}"; see evidence_ids')
                if state["plan"] is not None:
                    raise Replan("map revised by SLAM optimization; coverage reprojected")
            fresh = self._evidence.since(state["serial"])
            for serial, capture in fresh:
                state["serial"] = serial
                if session.integrate(capture, self._map[0]):
                    raise SearchEnded("found", f'found "{session.label}"; see evidence_ids')
                dwell = state["dwell"]
                if (dwell and capture.camera_id == dwell[0] and capture.stamp >= dwell[1]
                        and capture.stationary and capture.pose is not None):
                    state["dwell"] = None
            return fresh

        def check():
            current = self._checkpoint(goal, deadline, center, radius, mapping_quota=False)
            result.travel_m = travelled()
            if result.travel_m >= travel_limit:
                raise SearchEnded("budget_exhausted", "search travel budget reached")
            if not self._evidence.fresh(config["search_perception_timeout_sec"]):
                raise Paused("perception observations are stale")
            fresh = integrate()
            plan = state["plan"]
            if state["revision"] != self._map_revision:
                state["revision"] = self._map_revision
                if plan is not None:
                    raise Replan("map revised by SLAM optimization; planned view revalidated")
            if fresh and plan is not None:
                viewpoint, mount = plan
                remaining = viewpoint_gain(viewpoint, weights(), *self._map, mount, profile)
                if remaining < max(config["search_min_gain_cells"], config["search_replan_fraction"] * viewpoint.gain):
                    raise Replan("new observations already covered the planned view",
                                 [capture.observation_id for _, capture in fresh])
            return current

        try:
            label, duration, radius, travel_limit, target = self._search_limits(goal.request)
            result.target_coverage = target
            pose = self._pose()
            center = (pose.pose.position.x, pose.pose.position.y)
            deadline = started_task + duration
            request = goal.request
            region = (((request.region_x, request.region_y), request.region_radius_m)
                      if request.region_radius_m > 0 else (center, radius))
            session = SearchSession(label, self._evidence.profile, profile, self._map, started=now(),
                                    target=target, region=region)
            if not session.supported:
                raise SearchEnded("incapable", self._evidence.profile_reason
                                  or f'"{label}" is not a label of the perception profile in force')
            if not self._evidence.fresh(config["search_perception_timeout_sec"]):
                raise SearchEnded("incapable", "no fresh perception observations; the instrument is not observing")
            integrate()  # Retained captures: recent evidence of absence and prior sightings shape the first plan.
            while True:
                try:
                    pose = check()
                    searchable, _ = session.masks(self._map[0])
                    outcome = session.outcome(searchable, now())
                    if outcome:
                        raise SearchEnded(outcome, "target coverage reached without finding the label")
                    mounts = {mount.camera_id: mount for mount in self._evidence.mounts()}
                    if not mounts:
                        raise SearchEnded("incapable", "no base camera has intrinsics and a mount transform")
                    grid = self._navigation_grid(check, pose)
                    position = (pose.pose.position.x, pose.pose.position.y)
                    viewpoint = select_viewpoint(
                        weights(), *grid, position, center, radius, list(mounts.values()), profile,
                        clearance=config["clearance_m"], footprint=self._footprint, region_margin=self._region_margin,
                        free_threshold=config["free_threshold"], spacing=config["search_viewpoint_spacing_m"],
                        headings=config["search_headings"], max_candidates=config["search_max_candidates"],
                        overhead_m=config["search_viewpoint_overhead_m"], min_gain=config["search_min_gain_cells"],
                        excluded=blocked + unproductive)
                    if viewpoint is None:
                        raise SearchEnded("inaccessible", "no reachable viewpoint observes the remaining unobserved space")
                    session.revise(state["reason"], state["evidence"], viewpoint)
                    state["reason"], state["evidence"] = "previous viewpoint observed", []
                    state["plan"] = (viewpoint, mounts[viewpoint.camera_id])
                    point = (viewpoint.x, viewpoint.y)
                    destination = self._destination(viewpoint.x, viewpoint.y, viewpoint.yaw)
                    stage = f"searching: viewpoint {session.revision_count} ({viewpoint.camera_id})"
                    goal.publish_feedback(Search.Feedback(stage=stage, current_pose=pose, **progress()))
                    if not self._route(check, pose, destination, grid, center, radius):
                        state["plan"], state["reason"] = None, "previous viewpoint unreachable"
                        blocked.append(point)
                        continue
                    reached = self._drive(check, destination, stage, lambda current: goal.publish_feedback(
                        Search.Feedback(stage=stage, current_pose=current, **progress())))
                    state["plan"] = None
                    if not reached:
                        state["reason"] = "previous viewpoint unreachable"
                        blocked.append(point)
                        continue
                    visited.append(point)
                    # Coverage needs a stationary capture taken at the viewpoint, not the arrival itself.
                    state["dwell"] = (viewpoint.camera_id, now())
                    self._stage = "searching: observing"
                    until = time.monotonic() + config["search_dwell_sec"]
                    while state["dwell"] is not None and time.monotonic() < until:
                        check()
                        time.sleep(0.05)
                    if state["dwell"] is not None:
                        # Never retry a view that produced no usable capture; it would spin until the budget.
                        unproductive.append(point)
                        state["reason"] = "previous viewpoint yielded no stationary capture"
                    state["dwell"] = None
                except Replan as change:
                    self._stop_navigation()
                    state["plan"], state["reason"], state["evidence"] = None, str(change), change.evidence
                except Paused as reason:
                    state["plan"] = state["dwell"] = None
                    state["reason"] = f"resumed after pause: {reason}"
                    paused_at = task_time()

                    def paused_check():
                        nonlocal deadline, paused_at
                        current = task_time()
                        deadline += current - paused_at
                        paused_at = current
                        return check()

                    self._pause(goal, paused_check, reason, progress, Search.Feedback)
        except SearchEnded as end:
            result.outcome, result.message = end.outcome, str(end)
        except TimeoutError:
            result.outcome, result.message = "budget_exhausted", "search duration reached"
        except InterruptedError:
            result.outcome, result.message = "canceled", "search canceled"
        except Exception as error:
            result.outcome, result.message = "failed", str(error)
        finally:
            try:
                self._stop_navigation()
            except Exception as error:
                result.outcome, result.message = "failed", f"{result.message}; cleanup failed: {error}"
                self.get_logger().error(result.message)
            if session is not None and self._map is not None:
                try:
                    session.rebase(self._map)
                    report = session.report(result.outcome, self._map[0], now(), {
                        "viewpoints_visited": len(visited), "viewpoints_unreachable": len(blocked),
                        "viewpoints_without_stationary_capture": len(unproductive),
                        "travel_m": round(result.travel_m, 3), "elapsed_sec": round(task_time() - started_task, 3)})
                    coverage = report["coverage"]
                    result.coverage_fraction = coverage["fraction"]
                    result.covered_area_m2 = coverage["covered_area_m2"]
                    result.unobserved_area_m2 = coverage["unobserved_area_m2"]
                    result.evidence_ids = session.evidence_ids()
                    result.report_json = json.dumps(report, allow_nan=False)
                    result.message = f"{result.message}. {report['statement']}"
                except Exception as error:
                    self.get_logger().error(f"search report failed: {error}")
            result.found = result.outcome == "found"
            result.viewpoints_visited, result.viewpoints_unreachable = len(visited), len(blocked)
            if self._nav_uncertain:
                result.message += "; navigation stop unconfirmed; further navigation refused"
            result.complete = result.outcome in ("found", "coverage_reached") and not self._nav_uncertain
            self.get_logger().info(f"search ended: {result.outcome}: {result.message}")
            if goal.is_cancel_requested and not self._nav_uncertain:
                goal.canceled()
            elif result.complete:
                goal.succeed()
            else:
                goal.abort()
            with self._lock:
                self._busy = False
                self._stage = "idle"
        return result

    def _navigation_grid(self, check, pose):
        """The SLAM map with live costmap obstacles and the robot's own body cleared."""
        q = pose.pose.orientation
        grid, resolution, origin = self._map
        request = self._costmap.call_async(GetCostmap.Request())
        try:
            costmap = self._wait(request, self.config["service_timeout_sec"], check).map
        except ResponseTimeout as error:
            raise Paused("navigation costmap service is unavailable") from error
        finally:
            self._costmap.remove_pending_request(request)
        metadata = costmap.metadata
        if len(costmap.data) > self.config["max_map_cells"]:
            raise Paused("navigation costmap exceeds the cell limit")
        observed = OccupancyGrid(header=costmap.header, info=MapMetaData(
            resolution=metadata.resolution, width=metadata.size_x, height=metadata.size_y,
            origin=metadata.origin))
        # Inflation is a soft cost; only lethal and unknown cells block the body.
        costs = np.asarray(costmap.data, dtype=np.int16)
        observed.data = np.where(costs == 255, -1, np.where(costs == 254, 100, 0)).tolist()
        try:
            live, live_resolution, live_origin = map_geometry(observed, self.config["max_map_cells"])
        except ValueError as error:
            raise Paused(f"invalid navigation costmap: {error}") from error
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(costmap.header.stamp)).nanoseconds / 1e9
        if not -0.3 <= age <= self.config["service_timeout_sec"]:
            raise Paused("navigation costmap is stale")
        if (live.shape != grid.shape or abs(live_resolution - resolution) > 1e-6
                or not np.allclose(live_origin, origin, atol=1e-4, rtol=0)):
            raise Paused("navigation and SLAM maps are not aligned yet")
        grid = np.where((live < 0) | (live == 100), live, grid)
        # Cells under the robot's own body are free: it is standing there.
        return (with_body_free(grid, resolution, origin, (pose.pose.position.x, pose.pose.position.y),
                               2 * math.atan2(q.z, q.w), self._footprint), resolution, origin)

    def _destination(self, x, y, yaw):
        destination = PoseStamped()
        destination.header.frame_id = "map"
        destination.header.stamp = self.get_clock().now().to_msg()
        destination.pose.position.x, destination.pose.position.y = x, y
        destination.pose.orientation.z, destination.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return destination

    def _route(self, check, pose, destination, grid, center, radius):
        """Plan through known space; False when the destination is unreachable or its route is not clear."""
        plan = self._wait(self._planner.send_goal_async(ComputePathToPose.Goal(
            goal=destination, planner_id="ExploreKnown")), self.config["service_timeout_sec"], check)
        if not plan.accepted:
            return False
        try:
            route = self._wait(plan.get_result_async(), self.config["service_timeout_sec"], check)
        finally:
            plan.cancel_goal_async()
        if route.result.error_code == ComputePathToPose.Result.INVALID_PLANNER:
            raise RuntimeError("Nav2 planner plugin is invalid")
        if route.result.error_code == ComputePathToPose.Result.START_OCCUPIED:
            # A blocked start says nothing about whether this target is reachable.
            until = time.monotonic() + self.config["settle_sec"]
            while time.monotonic() < until:
                check()
                time.sleep(0.05)
            raise Paused("navigation starting footprint is occupied")
        if (route.status != GoalStatus.STATUS_SUCCEEDED or route.result.error_code
                or route.result.path.header.frame_id != "map" or not route.result.path.poses):
            return False
        return self._route_clear(route.result.path.poses, pose, grid, center, radius)

    def _drive(self, check, destination, stage, feedback):
        """Drive one guarded Nav2 goal: whether Nav2 reached it, or None if it never ran to a result."""
        self._owned_id = owned_goal_id()
        until = time.monotonic() + self.config["service_timeout_sec"]
        while (self._guard_ack is None or self._guard_ack[0] != bytes(self._owned_id.uuid)
               or time.monotonic() - self._guard_ack[1] > self.config["data_timeout_sec"]):
            check()
            if time.monotonic() >= until:
                self._owned_id = None
                raise Paused("navigation ownership guard did not acknowledge this goal")
            time.sleep(0.05)
        self._stage = stage
        self._nav_request = self._navigation.send_goal_async(NavigateToPose.Goal(
            pose=destination, behavior_tree=self.config["navigation_tree"]),
            goal_uuid=self._owned_id, feedback_callback=lambda message: feedback(message.feedback.current_pose))
        self._nav_handle = self._wait(self._nav_request, self.config["service_timeout_sec"], check)
        if not self._nav_handle.accepted:
            self._stop_navigation()
            return None
        self._nav_result = self._nav_handle.get_result_async()
        try:
            navigation = self._wait(self._nav_result, self.config["navigation_timeout_sec"], check)
        except ResponseTimeout:
            # A target Nav2 cannot reach in time is unreachable, not a fault.
            self._stop_navigation()
            check()
            return None
        self._stop_navigation()
        if navigation.status == GoalStatus.STATUS_CANCELED:
            raise RuntimeError("Nav2 goal canceled or replaced by another client")
        check()
        if navigation.result.error_code == FollowPath.Result.INVALID_CONTROLLER:
            raise RuntimeError("Nav2 controller plugin is invalid")
        return navigation.status == GoalStatus.STATUS_SUCCEEDED

    def _explore_step(self, goal, check, center, radius, visited, blocked, result):
        """Plan and drive to one target; return False when no target remains."""
        pose = check()
        grid = self._navigation_grid(check, pose)
        target, stage, result.mapped_area_m2 = select_target(
            *grid, (pose.pose.position.x, pose.pose.position.y), center, radius, visited, blocked,
            clearance=self.config["clearance_m"], footprint=self._footprint, region_margin=self._region_margin,
            observation_distance=self.config["observation_distance_m"],
            spacing=self.config["target_spacing_m"], revisit_spacing=self.config["revisit_spacing_m"],
            free_threshold=self.config["free_threshold"], revisit=goal.request.revisit_known,
        )
        if target is None:
            result.complete = not blocked
            result.message = "no further reachable observation targets" if not blocked else "remaining targets failed navigation"
            return False
        yaw = math.atan2(target[1] - pose.pose.position.y, target[0] - pose.pose.position.x)
        destination = self._destination(*target, yaw)
        feedback = Explore.Feedback(stage=stage, current_pose=pose, visited_targets=len(visited),
                                    unreachable_targets=len(blocked), mapped_area_m2=result.mapped_area_m2)
        goal.publish_feedback(feedback)
        if not self._route(check, pose, destination, grid, center, radius):
            blocked.append(target)
            return True
        reached = self._drive(check, destination, stage, lambda current: goal.publish_feedback(Explore.Feedback(
            stage=stage, current_pose=current, visited_targets=len(visited),
            unreachable_targets=len(blocked), mapped_area_m2=result.mapped_area_m2)))
        (visited if reached else blocked).append(target)
        if reached is None:
            return True
        until = time.monotonic() + self.config["settle_sec"]
        while time.monotonic() < until:
            check()
            time.sleep(0.05)
        return True


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotExplorer()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _sig, _frame: node._shutdown_requested.set())
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        # Keep DDS alive while the worker cancels its owned Nav2 goal on shutdown.
        while not node._shutdown_requested.is_set() or node._busy:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._stop_navigation()
        finally:
            executor.shutdown(timeout_sec=5)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
