#!/usr/bin/env python3
"""Explicit, bounded Nav2 exploration and an always-on RTAB-Map quota guard."""

import math
from pathlib import Path
import signal
import threading
import time

import yaml

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid
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
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from unique_identifier_msgs.msg import UUID

from lekiwi_rmf.action import Explore
from lekiwi_rmf.exploration_owner_guard import owned_goal_id
from lekiwi_rmf.exploration import (
    database_size, footprint_is_free, load_navigation_footprint, map_geometry, select_target, task_limits,
    with_body_free,
)


class Paused(RuntimeError):
    """A recoverable fault: stop navigation, wait for recovery, then resume."""


class ResponseTimeout(Paused):
    """A ROS peer did not answer before its deadline."""


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
                or self.config["max_radius_m"] <= 2 * self._region_margin
                or self.config["observation_distance_m"] <= self.config["clearance_m"]):
            raise ValueError("invalid exploration clearance, observation distance or region")
        self.database = Path(self.config["database_path"]).expanduser()
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
        self.create_subscription(Info, "/info", lambda msg: self._record("slam", msg.header), 1)
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
        self._action = ActionServer(
            self, Explore, "/robot/explore", self._execute, goal_callback=self._accept,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT, callback_group=self._group,
        )
        # Wall time quotas remain effective even if simulation time stops.
        self._timer = self.create_timer(0.25, self._monitor, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def destroy_node(self):
        # rclpy Node.destroy_node does not destroy action waitables.
        self._action.destroy()
        self._navigation.destroy()
        self._planner.destroy()
        return super().destroy_node()

    def _record(self, name, value):
        self._inputs[name] = (value, time.monotonic())

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
            if not value or now - received > timeout:
                raise Paused(f"missing, stale or denied {name}")
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
            if require_permission and state.action_type == CollisionMonitorState.STOP:
                raise Paused(f"collision monitor stop: {state.polygon_name}")

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

    def _accept(self, request):
        try:
            if self._shutdown_requested.is_set():
                raise RuntimeError("exploration server is shutting down")
            if not self.config["allow_exploration"]:
                raise RuntimeError("exploration is disabled for fixed-map/RMF operation")
            _, radius = task_limits(request.max_duration_sec, request.max_radius_m,
                                    self.config["max_duration_sec"], self.config["max_radius_m"])
            if radius <= 2 * self._region_margin:
                raise ValueError("exploration radius is too small for the footprint")
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

    def _readiness_reason(self, require_permission=True):
        try:
            if not self.config["allow_exploration"]:
                return "exploration disabled for fixed-map/RMF operation"
            self._healthy(require_permission=require_permission)
            self._pose()
            if (not self._navigation.server_is_ready() or not self._planner.server_is_ready()
                    or not self._mapping_client.service_is_ready()
                    or not self._localization_client.service_is_ready()):
                return "Nav2 or mapping services are unavailable"
            if self._nav_uncertain:
                return "previous navigation stop is unconfirmed"
            if self._freeze_unconfirmed or (self._freeze_future is not None and not self._freeze_future.done()):
                return "waiting for mapping quota freeze to finish"
            return self._quota_limit(time.monotonic())
        except (RuntimeError, OSError, TransformException) as error:
            return str(error)

    def _checkpoint(self, goal, deadline, center, radius):
        if self._shutdown_requested.is_set():
            raise RuntimeError("exploration server is shutting down")
        if goal.is_cancel_requested:
            raise InterruptedError("exploration canceled")
        if time.monotonic() >= deadline:
            raise TimeoutError("exploration duration reached")
        self._healthy()
        reason = self._quota_reason or self._quota_limit(time.monotonic())
        if reason:
            raise RuntimeError(reason)
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

    def _pause(self, goal, check, reason, progress):
        """Hold the robot still until every input recovers.

        Only the task's own end conditions (cancel, duration, quota, region,
        shutdown) or an unconfirmed navigation stop end it while paused.
        """
        self._stop_navigation()
        self.get_logger().warning(f"exploration paused: {reason}")
        self._stage = f"paused: {reason}"
        goal.publish_feedback(Explore.Feedback(stage=self._stage, **progress()))
        while True:
            try:
                check()
                break
            except Paused as current:
                stage = f"paused: {current}"
                if stage != self._stage:
                    self._stage = stage
                    goal.publish_feedback(Explore.Feedback(stage=stage, **progress()))
                time.sleep(0.2)
        self.get_logger().info("exploration resumed")

    def _execute(self, goal):
        result = Explore.Result()
        previous_mapping = self._mapping
        visited, blocked = [], []
        mapping_requested = False
        try:
            duration, radius = task_limits(goal.request.max_duration_sec, goal.request.max_radius_m,
                                           self.config["max_duration_sec"], self.config["max_radius_m"])
            pose = self._pose()
            center = (pose.pose.position.x, pose.pose.position.y)
            deadline = time.monotonic() + duration
            mapping_confirmed = False

            def check():
                current = self._checkpoint(goal, deadline, center, radius)
                if mapping_confirmed and self._mapping is not True:
                    raise RuntimeError("mapping mode changed during exploration")
                return current

            def progress():
                return {"visited_targets": len(visited), "unreachable_targets": len(blocked),
                        "mapped_area_m2": result.mapped_area_m2}
            while True:
                try:
                    if not mapping_confirmed:
                        check()
                        mapping_requested = True
                        self._set_mapping(True)
                        mapping_confirmed = True
                    if not self._explore_step(goal, check, center, radius, visited, blocked, result):
                        break
                except Paused as reason:
                    self._pause(goal, check, reason, progress)
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

    def _explore_step(self, goal, check, center, radius, visited, blocked, result):
        """Plan and drive to one target; return False when no target remains."""
        pose = check()
        q = pose.pose.orientation
        grid, resolution, origin = self._map
        # Cells under the robot's own body are free: it is standing there.
        grid = (with_body_free(grid, resolution, origin, (pose.pose.position.x, pose.pose.position.y),
                               2 * math.atan2(q.z, q.w), self._footprint), resolution, origin)
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
        destination = PoseStamped()
        destination.header.frame_id = "map"
        destination.header.stamp = self.get_clock().now().to_msg()
        destination.pose.position.x, destination.pose.position.y = target
        yaw = math.atan2(target[1] - pose.pose.position.y, target[0] - pose.pose.position.x)
        destination.pose.orientation.z, destination.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        feedback = Explore.Feedback(stage=stage, current_pose=pose, visited_targets=len(visited),
                                    unreachable_targets=len(blocked), mapped_area_m2=result.mapped_area_m2)
        goal.publish_feedback(feedback)
        plan = self._wait(self._planner.send_goal_async(ComputePathToPose.Goal(
            goal=destination, planner_id="ExploreKnown")), self.config["service_timeout_sec"], check)
        if not plan.accepted:
            blocked.append(target)
            return True
        try:
            route = self._wait(plan.get_result_async(), self.config["service_timeout_sec"], check)
        finally:
            plan.cancel_goal_async()
        if (route.status != GoalStatus.STATUS_SUCCEEDED or route.result.error_code
                or route.result.path.header.frame_id != "map" or not route.result.path.poses):
            blocked.append(target)
            return True
        for waypoint in route.result.path.poses:
            p = waypoint.pose.position
            if (math.hypot(p.x - center[0], p.y - center[1]) >= radius - self._region_margin
                    or not self._footprint_clear(waypoint, grid)):
                blocked.append(target)
                return True
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
            goal_uuid=self._owned_id,
            feedback_callback=lambda _msg: goal.publish_feedback(Explore.Feedback(
                stage=stage, current_pose=_msg.feedback.current_pose, visited_targets=len(visited),
                unreachable_targets=len(blocked), mapped_area_m2=result.mapped_area_m2)))
        self._nav_handle = self._wait(self._nav_request, self.config["service_timeout_sec"], check)
        if not self._nav_handle.accepted:
            blocked.append(target)
            self._stop_navigation()
            return True
        self._nav_result = self._nav_handle.get_result_async()
        try:
            navigation = self._wait(self._nav_result, self.config["navigation_timeout_sec"], check)
        except ResponseTimeout:
            # A target Nav2 cannot reach in time is unreachable, not a fault.
            self._stop_navigation()
            check()
            blocked.append(target)
            return True
        self._stop_navigation()
        if navigation.status == GoalStatus.STATUS_CANCELED:
            raise RuntimeError("Nav2 goal canceled or replaced by another client")
        check()
        (visited if navigation.status == GoalStatus.STATUS_SUCCEEDED else blocked).append(target)
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
