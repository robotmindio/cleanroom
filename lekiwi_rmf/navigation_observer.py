"""Observe Nav2 execution below the parent action before admitting another task."""

from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.qos import DurabilityPolicy, QoSProfile


TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
CHILD_ACTIONS = ('/follow_path', '/spin', '/backup', '/drive_on_heading', '/assisted_teleop',
                 '/navigate_through_poses')


def server_instance(node, name):
    if any(node.count_services(f'{name}/_action/{service}') != 1
           for service in ('send_goal', 'get_result', 'cancel_goal')):
        return ''
    return status_instance(node, name)


def status_instance(node, name):
    publishers = node.get_publishers_info_by_topic(f'{name}/_action/status')
    return bytes(publishers[0].endpoint_gid).hex() if len(publishers) == 1 else ''


class NavigationObserver:
    def __init__(self, node):
        self.node = node
        self.active = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in CHILD_ACTIONS:
            node.create_subscription(GoalStatusArray, f'{name}/_action/status',
                                     lambda msg, action=name: self._status(action, msg), latched)

    def _status(self, name, message):
        self.active[name] = (status_instance(self.node, name),
                             any(item.status not in TERMINAL for item in message.status_list))

    def busy(self):
        for name, (observed, active) in self.active.items():
            publishers = self.node.get_publishers_info_by_topic(f'{name}/_action/status')
            if active and (not observed or any(bytes(item.endpoint_gid).hex() == observed for item in publishers)):
                return True
        return False
