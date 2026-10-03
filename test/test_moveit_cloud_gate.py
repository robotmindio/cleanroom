from types import SimpleNamespace

import pytest

from builtin_interfaces.msg import Time as TimeMsg

from lekiwi_rmf.moveit_cloud_gate import MAX_WAIT_NS, ready
from lekiwi_rmf import moveit_cloud_gate


@pytest.mark.parametrize("shutdown", [KeyboardInterrupt, moveit_cloud_gate.ExternalShutdownException])
def test_main_closes_cleanly_after_ros_shutdown(monkeypatch, shutdown):
    calls = []
    node = SimpleNamespace(destroy_node=lambda: calls.append("destroy"))
    monkeypatch.setattr(moveit_cloud_gate, "MoveItCloudGate", lambda: node)
    monkeypatch.setattr(moveit_cloud_gate.rclpy, "init", lambda: calls.append("init"))

    def spin(_):
        raise shutdown()

    monkeypatch.setattr(moveit_cloud_gate.rclpy, "spin", spin)
    monkeypatch.setattr(moveit_cloud_gate.rclpy, "try_shutdown", lambda: calls.append("shutdown"))
    moveit_cloud_gate.main()
    assert calls == ["init", "destroy", "shutdown"]


def test_cloud_waits_for_matching_arm_transform_and_expires():
    cloud = SimpleNamespace(header=SimpleNamespace(
        frame_id="astra_camera_optical_frame", stamp=TimeMsg(sec=10)
    ))

    class Transforms:
        available = False

        def can_transform(self, target, source, stamp):
            assert target == "gripper_collision_proxy"
            assert source == cloud.header.frame_id
            assert stamp.nanoseconds == 10_000_000_000
            return self.available

    transforms = Transforms()
    assert not ready(cloud, 1, 2, transforms)
    transforms.available = True
    assert ready(cloud, 1, 2, transforms)
    assert not ready(cloud, 1, MAX_WAIT_NS + 2, transforms)


def test_new_cloud_cannot_replace_the_frame_waiting_for_its_joint_transform(monkeypatch):
    monkeypatch.setattr(moveit_cloud_gate.time, "monotonic_ns", lambda: 1)
    published = []
    transforms = SimpleNamespace(can_transform=lambda *_: True)
    node = SimpleNamespace(_pending=None, _transforms=transforms,
                           _publisher=SimpleNamespace(publish=published.append))
    first = SimpleNamespace(header=SimpleNamespace(frame_id="depth", stamp=TimeMsg(sec=1)))
    newer = SimpleNamespace(header=SimpleNamespace(frame_id="depth", stamp=TimeMsg(sec=2)))
    moveit_cloud_gate.MoveItCloudGate._on_cloud(node, first)
    moveit_cloud_gate.MoveItCloudGate._on_cloud(node, newer)
    assert node._pending == (first, 1)
    moveit_cloud_gate.MoveItCloudGate._flush(node)
    assert published == [first] and node._pending is None
    moveit_cloud_gate.MoveItCloudGate._on_cloud(node, newer)
    assert node._pending == (newer, 1)
