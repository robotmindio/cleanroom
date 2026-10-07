#!/usr/bin/env python3
"""Explicit, bounded Nav2 exploration and an always-on RTAB-Map quota guard."""

import math
from pathlib import Path
import signal
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
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

from lekiwi_rmf.action import Explore
from lekiwi_rmf.exploration import (
    database_size, known_safe_cells, map_geometry, select_target, task_limits, world_to_cell,
)


class RobotExplorer(Node):
    def __init__(self, **kwargs):
        super().__init__("robot_explorer", **kwargs)
        defaults = {
            "allow_exploration": True, "database_path": str(Path.home() / ".ros/lekiwi_rtabmap.db"),
            "mapping_max_bytes": 536870912, "mapping_max_seconds": 14400.0,
            "max_duration_sec": 900.0, "max_radius_m": 5.0, "clearance_m": 0.38,
            "observation_distance_m": 0.8, "target_spacing_m": 0.5,
            "revisit_spacing_m": 1.0, "free_threshold": 20, "max_map_cells": 250000,
            "data_timeout_sec": 1.0, "service_timeout_sec": 3.0,
            "navigation_timeout_sec": 180.0, "settle_sec": 1.0,
            "navigation_tree": str(Path(get_package_share_directory("lekiwi_rmf"))
                                   / "config/explore_nav_to_pose.xml"),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        for name, value in self.config.items():
            if type(value) in (float, int) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and positive")
        if (not 0 < self.config["free_threshold"] < 100
                or self.config["clearance_m"] < 0.38
                or self.config["max_radius_m"] <= 2 * self.config["clearance_m"]
                or self.config["observation_distance_m"] <= self.config["clearance_m"]):
            raise ValueError("invalid exploration clearance, observation distance or region")
        self.database = Path(self.config["database_path"]).expanduser()
        self._lock = threading.Lock()
        self._busy = False
        self._nav_uncertain = False
        self._nav_request = self._nav_handle = self._nav_result = None
        self._map = None
        self._inputs = {}
        self._mapping = None
        self._mode_at = 0.0
        self._mode_query_at = 0.0
        self._mapping_started = None
        self._quota_reason = ""
        self._mode_future = self._freeze_future = None
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
        self._parameters = AsyncParameterClient(self, "/rtabmap", callback_group=self._group)
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

    def _record(self, name, value):
        self._inputs[name] = (value, time.monotonic())

    def _on_map(self, message):
        try:
            self._map = map_geometry(message, self.config["max_map_cells"])
        except ValueError as error:
            self._map = None
            self.get_logger().error(str(error), throttle_duration_sec=5)

    def _mode_response(self, future, requested_at):
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
        now = time.monotonic()
        if self._parameters.services_are_ready():
            if self._mode_future is not None and not self._mode_future.done():
                if now - self._mode_requested_at > self.config["service_timeout_sec"]:
                    self._mode_future.cancel()
                    self._mapping = None
            if self._mode_future is None or self._mode_future.done():
                self._mode_requested_at = now
                self._mode_future = self._parameters.get_parameters(["Mem/IncrementalMemory"])
                self._mode_future.add_done_callback(lambda future, sent=now: self._mode_response(future, sent))
        if self._mapping_started is None:
            return
        try:
            reason = self._quota_limit(now)
        except OSError as error:
            reason = f"cannot measure mapping storage: {error}"
        if reason:
            self._quota_reason = reason
            if (self._freeze_future is not None and not self._freeze_future.done()
                    and now - self._freeze_requested_at > self.config["service_timeout_sec"]):
                self._freeze_future.cancel()
            if self._localization_client.service_is_ready() and (
                    self._freeze_future is None or self._freeze_future.done()):
                self.get_logger().warning(f"freezing RTAB-Map: {reason}", throttle_duration_sec=5)
                self._freeze_future = self._localization_client.call_async(Empty.Request())
                self._freeze_requested_at = now
                self._freeze_future.add_done_callback(self._check_freeze)

    def _check_freeze(self, future):
        try:
            future.result()
        except Exception as error:
            self.get_logger().error(f"cannot freeze RTAB-Map: {error}")

    def _quota_limit(self, now):
        if database_size(self.database) >= self.config["mapping_max_bytes"]:
            return "mapping database quota reached"
        if (self._mapping_started is not None
                and now - self._mapping_started >= self.config["mapping_max_seconds"]):
            return "mapping session duration reached"
        return ""

    def _pose(self):
        transform = self._tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        stamp = rclpy.time.Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        t, q = transform.transform.translation, transform.transform.rotation
        if (not -0.3 <= age <= self.config["data_timeout_sec"]
                or not all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w))):
            raise RuntimeError("map-to-robot transform is stale or non-finite")
        pose = PoseStamped(header=transform.header)
        pose.pose.position.x, pose.pose.position.y = t.x, t.y
        pose.pose.orientation = q
        return pose

    def _healthy(self):
        now = time.monotonic()
        if self._map is None:
            raise RuntimeError("no valid occupancy map")
        for name in ("base_motion_permitted", "arm_stowed", "slam"):
            value, received = self._inputs.get(name, (None, 0))
            if not value or now - received > self.config["data_timeout_sec"]:
                raise RuntimeError(f"missing, stale or denied {name}")
            if name == "slam":
                age = (self.get_clock().now() - rclpy.time.Time.from_msg(value.stamp)).nanoseconds / 1e9
                if not -0.3 <= age <= self.config["data_timeout_sec"]:
                    raise RuntimeError("SLAM observations are stale")
        if self._mapping is None or now - self._mode_at > self.config["service_timeout_sec"]:
            raise RuntimeError("mapping mode is unknown or stale")

    def _accept(self, request):
        try:
            if self._shutdown_requested.is_set():
                raise RuntimeError("exploration server is shutting down")
            _, radius = task_limits(request.max_duration_sec, request.max_radius_m,
                                    self.config["max_duration_sec"], self.config["max_radius_m"])
            if radius <= 2 * self.config["clearance_m"]:
                raise ValueError("exploration radius is too small for the footprint")
            if not self.config["allow_exploration"]:
                raise RuntimeError("exploration is disabled for fixed-map/RMF operation")
            self._healthy()
            self._pose()
            if (not self._navigation.server_is_ready() or not self._planner.server_is_ready()
                    or not self._mapping_client.service_is_ready()
                    or not self._localization_client.service_is_ready()):
                raise RuntimeError("Nav2 or mapping services are unavailable")
            if self._quota_limit(time.monotonic()):
                raise RuntimeError(self._quota_limit(time.monotonic()))
            with self._lock:
                if self._busy or self._nav_uncertain:
                    raise RuntimeError("another goal is active or its navigation stop is unconfirmed")
                self._busy = True
            return GoalResponse.ACCEPT
        except (RuntimeError, ValueError, OSError, TransformException) as error:
            self.get_logger().warning(f"exploration rejected: {error}")
            return GoalResponse.REJECT

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
        if math.hypot(xy.x - center[0], xy.y - center[1]) >= radius - self.config["clearance_m"]:
            raise RuntimeError("exploration region stopping margin reached")
        return pose

    def _wait(self, future, timeout, check=None):
        deadline = time.monotonic() + timeout
        while not future.done():
            if check:
                check()
            if time.monotonic() >= deadline:
                raise TimeoutError("ROS response deadline exceeded")
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
        response = self._parameters.get_parameters(["Mem/IncrementalMemory"])
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
                self._nav_uncertain = False
        except Exception as error:
            self.get_logger().error(f"late navigation cancellation failed: {error}")

    def _late_stopped(self, future):
        try:
            if future.result().status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                          GoalStatus.STATUS_ABORTED):
                self._nav_uncertain = False
        except Exception as error:
            self.get_logger().error(f"late navigation stop is unconfirmed: {error}")

    def _stop_navigation(self):
        if self._nav_request is None:
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
        except Exception:
            # Preserve ownership until a late acceptance/result is canceled.
            self._nav_request.add_done_callback(self._cancel_late)
            raise
        finally:
            self._nav_request = self._nav_handle = self._nav_result = None

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
            def check():
                current = self._checkpoint(goal, deadline, center, radius)
                if mapping_requested and self._mapping is not True:
                    raise RuntimeError("mapping mode changed during exploration")
                return current
            check()
            mapping_requested = True
            self._set_mapping(True)
            while True:
                pose = check()
                target, stage, result.mapped_area_m2 = select_target(
                    *self._map, (pose.pose.position.x, pose.pose.position.y), center, radius, visited, blocked,
                    clearance=self.config["clearance_m"], observation_distance=self.config["observation_distance_m"],
                    spacing=self.config["target_spacing_m"], revisit_spacing=self.config["revisit_spacing_m"],
                    free_threshold=self.config["free_threshold"], revisit=goal.request.revisit_known,
                )
                if target is None:
                    result.complete = not blocked
                    result.message = "no further reachable observation targets" if not blocked else "remaining targets failed navigation"
                    break
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
                    continue
                try:
                    route = self._wait(plan.get_result_async(), self.config["service_timeout_sec"], check)
                finally:
                    plan.cancel_goal_async()
                if (route.status != GoalStatus.STATUS_SUCCEEDED or route.result.error_code
                        or route.result.path.header.frame_id != "map" or not route.result.path.poses):
                    blocked.append(target)
                    continue
                grid, resolution, origin = self._map
                safe = known_safe_cells(grid, resolution, self.config["clearance_m"], self.config["free_threshold"])
                valid = True
                for waypoint in route.result.path.poses:
                    p = waypoint.pose.position
                    x, y = world_to_cell((p.x, p.y), resolution, origin)
                    if (math.hypot(p.x - center[0], p.y - center[1]) >= radius - self.config["clearance_m"]
                            or not 0 <= y < grid.shape[0] or not 0 <= x < grid.shape[1]
                            or not safe[y, x]):
                        valid = False
                        break
                if not valid:
                    blocked.append(target)
                    continue
                self._nav_request = self._navigation.send_goal_async(NavigateToPose.Goal(
                    pose=destination, behavior_tree=self.config["navigation_tree"]),
                    feedback_callback=lambda _msg: goal.publish_feedback(Explore.Feedback(
                        stage=stage, current_pose=_msg.feedback.current_pose, visited_targets=len(visited),
                        unreachable_targets=len(blocked), mapped_area_m2=result.mapped_area_m2)))
                self._nav_handle = self._wait(self._nav_request, self.config["service_timeout_sec"], check)
                if not self._nav_handle.accepted:
                    blocked.append(target)
                    self._stop_navigation()
                    continue
                self._nav_result = self._nav_handle.get_result_async()
                navigation = self._wait(self._nav_result, self.config["navigation_timeout_sec"], check)
                self._stop_navigation()
                if navigation.status == GoalStatus.STATUS_CANCELED:
                    raise RuntimeError("Nav2 goal canceled or replaced by another client")
                (visited if navigation.status == GoalStatus.STATUS_SUCCEEDED else blocked).append(target)
                until = time.monotonic() + self.config["settle_sec"]
                while time.monotonic() < until:
                    check()
                    time.sleep(0.05)
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
        return result


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
