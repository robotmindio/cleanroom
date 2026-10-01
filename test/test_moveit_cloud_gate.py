from types import SimpleNamespace

from builtin_interfaces.msg import Time as TimeMsg

from lekiwi_rmf.moveit_cloud_gate import MAX_WAIT_NS, ready


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
