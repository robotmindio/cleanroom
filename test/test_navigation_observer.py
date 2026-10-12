"""A parent result cannot authorize another task while Nav2 children execute."""

from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus, GoalStatusArray

from lekiwi_rmf.navigation_observer import NavigationObserver, server_instance


def test_children_block_until_terminal_or_original_publisher_disappears():
    publisher = SimpleNamespace(endpoint_gid=list(range(16)))
    node = Mock()
    node.get_publishers_info_by_topic.return_value = [publisher]
    observer = NavigationObserver(node)
    observer._status('/follow_path', GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)]))
    assert observer.busy()
    observer._status('/follow_path', GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_CANCELED)]))
    assert not observer.busy()
    observer._status('/follow_path', GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)]))
    node.get_publishers_info_by_topic.return_value = []
    assert not observer.busy()


def test_ambiguous_servers_cannot_prove_a_generation_change():
    publisher = SimpleNamespace(endpoint_gid=list(range(16)))
    node = Mock()
    node.count_services.return_value = 1
    node.get_publishers_info_by_topic.return_value = [publisher]
    assert server_instance(node, '/navigate_to_pose') == bytes(range(16)).hex()
    node.get_publishers_info_by_topic.return_value = [publisher, publisher]
    assert not server_instance(node, '/navigate_to_pose')
    observer = NavigationObserver(node)
    observer._status('/follow_path', GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)]))
    assert observer.busy()
    node.get_publishers_info_by_topic.return_value = [publisher]
    node.count_services.return_value = 2
    assert not server_instance(node, '/navigate_to_pose')
