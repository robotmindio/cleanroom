"""Read-only odometry accounting; reset epochs make uncertainty explicit."""

import math
import time
import uuid


class NavigationDistance:
    def __init__(self):
        self.epoch = uuid.uuid4().hex
        self.meters = 0.0
        self._previous = None
        self._received = 0.0
        self.reason = 'waiting for odometry'

    def update(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 10**9
        position = message.pose.pose.position
        point = (position.x, position.y)
        if message.header.frame_id != 'odom' or not all(math.isfinite(v) for v in (*point, stamp)) or stamp <= 0:
            self.reason = 'invalid odometry'
            return
        if self._previous:
            old_stamp, old_point = self._previous
            if stamp == old_stamp:
                return
            elapsed, delta = stamp - old_stamp, math.dist(point, old_point)
            if not 0 < elapsed <= 1 or delta > elapsed + .02:
                self.epoch, self.meters = uuid.uuid4().hex, 0.0
                self.reason = 'odometry reset or gap'
                self._previous = (stamp, point)
                self._received = 0.0
                return
            self.meters += delta
        self._previous, self._received, self.reason = (stamp, point), time.monotonic(), ''

    def status(self):
        ready = bool(self._received and time.monotonic() - self._received <= 1 and not self.reason)
        return {'schema': 1, 'ready': ready, 'epoch': self.epoch, 'meters': self.meters,
                'reason': self.reason or ('' if ready else 'odometry stale')}
