"""Bounded capture-time stationary evidence from existing read-only ROS feedback."""

from collections import deque
import math

from lekiwi_rmf.arm_trajectory import ARM_JOINTS


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


class ObservationMotion:
    def __init__(self):
        self.odometry = deque(maxlen=512)
        self.joints = deque(maxlen=512)

    def odom(self, message):
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        values = [p.x, p.y, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z]
        if (stamp(message) <= 0 or not all(math.isfinite(x) for x in values) or
                abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .01):
            self.odometry.clear()
            return
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self._append(self.odometry, (stamp(message), p.x, p.y, yaw,
                                    math.hypot(v.linear.x, v.linear.y), abs(v.angular.z)))

    def joint(self, message):
        positions = dict(zip(message.name, message.position))
        if (stamp(message) <= 0 or len(message.name) != len(set(message.name)) or
                len(message.name) != len(message.position) or
                any(name not in positions or not math.isfinite(positions[name]) for name in ARM_JOINTS)):
            self.joints.clear()
            return
        self._append(self.joints, (stamp(message), *(positions[name] for name in ARM_JOINTS)))

    @staticmethod
    def _append(samples, sample):
        if samples and sample[0] <= samples[-1][0]:
            samples.clear()
        samples.append(sample)

    @staticmethod
    def _window(samples, captured):
        window = [sample for sample in samples if captured - .4 <= sample[0] <= captured]
        if len(window) < 2 or window[0][0] > captured - .25 or window[-1][0] < captured - .1:
            return []
        if any(right[0] - left[0] > .15 for left, right in zip(window, window[1:])):
            return []
        return window

    def stationary(self, captured, wrist=False):
        odom = self._window(self.odometry, captured)
        if not odom or any(sample[4] > .01 or sample[5] > .03 for sample in odom):
            return False
        if any(math.hypot(s[1] - odom[0][1], s[2] - odom[0][2]) > .005 or
               abs(math.atan2(math.sin(s[3] - odom[0][3]), math.cos(s[3] - odom[0][3]))) > .02 for s in odom):
            return False
        if wrist:
            joints = self._window(self.joints, captured)
            if not joints or any(abs(v - initial) > .01 for s in joints for v, initial in zip(s[1:], joints[0][1:])):
                return False
        return True
