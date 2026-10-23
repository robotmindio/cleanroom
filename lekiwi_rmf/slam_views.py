#!/usr/bin/env python3
"""Join the cameras that are alive into one RGB-D view array for RTAB-Map.

rtabmap_sync's rgbdx_sync emits only when every camera delivers a frame, so
one camera wedging on USB stopped SLAM although the lidar and the other camera
were fine. This node publishes the Astra and front views together while both
arrive, and whichever one is still alive while the other is silent, so SLAM
stops only when every camera is gone.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rtabmap_msgs.msg import RGBDImage, RGBDImages

# Inputs in output order: the array keeps the same camera order whenever both are present.
INPUTS = ("rgbd_image0", "rgbd_image1")


def stamp_seconds(message) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class ViewSet:
    """Decide which camera views to publish after each arrival.

    A camera is alive while its last frame arrived within ``stale_after``. Each
    arrival is joined with the closest unpublished frame of every other live
    camera within ``max_interval``; a silent camera is left out instead of
    holding the others back. Frames at or before a published one are not reused.
    """

    def __init__(self, count, max_interval, stale_after, depth=5):
        self.max_interval, self.stale_after, self.depth = max_interval, stale_after, depth
        self.frames = [[] for _ in range(count)]  # (stamp, message), oldest first
        self.arrival = [None] * count
        self.published = [float("-inf")] * count

    def offer(self, index, stamp, arrival, message):
        self.frames[index] = (self.frames[index] + [(stamp, message)])[-self.depth:]
        self.arrival[index] = arrival
        chosen = {}
        for i, last in enumerate(self.arrival):
            if last is None or arrival - last > self.stale_after:
                continue
            candidates = [frame for frame in self.frames[i]
                          if frame[0] > self.published[i] and abs(frame[0] - stamp) <= self.max_interval]
            if not candidates:
                return []
            chosen[i] = min(candidates, key=lambda frame: abs(frame[0] - stamp))
        for i, (frame_stamp, _) in chosen.items():
            self.published[i] = frame_stamp
        return [(i, chosen[i][1]) for i in sorted(chosen)]


class SlamViews(Node):
    def __init__(self):
        super().__init__("slam_rgbd_views")
        self._views = ViewSet(
            len(INPUTS),
            self.declare_parameter("approx_sync_max_interval", 0.35).value,
            self.declare_parameter("stale_after_sec", 1.5).value)
        self._alive = set(range(len(INPUTS)))
        self._publisher = self.create_publisher(RGBDImages, "rgbd_images", 5)
        for index, topic in enumerate(INPUTS):
            self.create_subscription(RGBDImage, topic, lambda message, i=index: self._on_view(i, message), 5)

    def _on_view(self, index, message):
        views = self._views.offer(index, stamp_seconds(message), time.monotonic(), message)
        if not views:
            return
        alive = {i for i, _ in views}
        if alive != self._alive:
            names = ", ".join(self.resolve_topic_name(INPUTS[i]) for i in sorted(alive))
            self.get_logger().warning(f"SLAM views now from: {names}")
            self._alive = alive
        self._publisher.publish(RGBDImages(header=message.header, rgbd_images=[view for _, view in views]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamViews()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
