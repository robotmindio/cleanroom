"""Camera provenance, capture-time motion and bounded inference scheduling."""

import threading
import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Image
from vision_msgs.msg import Detection2DArray

from lekiwi_rmf.arm_trajectory import ARM_JOINTS
from lekiwi_rmf.object_cameras import camera_bindings
from lekiwi_rmf.observation_motion import ObservationMotion
from test_object_observations import capture


@pytest.mark.parametrize('names', [[], ['unknown'], ['front', 'front'], [['front']], ['front', 'wrist', 'astra_color', 'extra']])
def test_camera_bindings_fail_closed(names):
    with pytest.raises(ValueError):
        camera_bindings(names)


def test_camera_bindings_keep_depth_and_optical_frame_with_their_camera():
    cameras = camera_bindings(['front', 'wrist', 'astra_color'])
    assert not cameras['front']['depth_topic'] and not cameras['wrist']['depth_topic']
    assert cameras['astra_color']['depth_topic'] == '/camera/astra/depth/image_raw'
    assert len({item['prefix'] for item in cameras.values()}) == 3


def motion_sample(at):
    m = Odometry()
    m.header.stamp = Time(sec=100, nanosec=at * 50000000)
    m.pose.pose.orientation.w = 1.
    j = JointState(header=m.header, name=list(ARM_JOINTS), position=[0.] * len(ARM_JOINTS))
    return m, j


def test_stationary_requires_recent_continuous_motion_feedback_and_still_wrist():
    motion = ObservationMotion()
    assert not motion.stationary(100.3)
    for index in range(8):
        odom, joints = motion_sample(index)
        motion.odom(odom)
        motion.joint(joints)
    assert motion.stationary(100.35, wrist=True)
    assert not motion.stationary(101, wrist=True)
    odom, joints = motion_sample(8)
    odom.twist.twist.linear.x = .02
    motion.odom(odom)
    assert not motion.stationary(100.4)
    motion = ObservationMotion()
    for index in range(8):
        odom, joints = motion_sample(index)
        joints.position[0] = index * .01
        motion.odom(odom)
        motion.joint(joints)
    assert motion.stationary(100.35) and not motion.stationary(100.35, wrist=True)
    odom.pose.pose.position.x = float('nan')
    motion.odom(odom)
    assert not motion.stationary(100.35)


def wait(predicate, timeout=5):
    end = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > end:
            raise AssertionError('isolated perception operation timed out')
        time.sleep(.01)


def test_receipt_keeps_latest_frame_while_inference_is_busy_and_expires_old_result(monkeypatch):
    from lekiwi_rmf import object_detector

    entered, release = threading.Event(), threading.Event()
    active, maximum, calls = 0, 0, 0
    def predict(image):
        nonlocal active, maximum, calls
        active += 1
        maximum = max(maximum, active)
        calls += 1
        if calls == 1:
            entered.set()
            release.wait(5)
        active -= 1
        return [[10, 10, 20, 20, .9, 'cup']]
    monkeypatch.setattr(object_detector, 'Recognizer', lambda *args: SimpleNamespace(predict=predict, sha256='a' * 64, default_query=None, labels=['cup']))
    context = Context()
    rclpy.init(context=context)
    node = object_detector.ObjectDetector(context=context, parameter_overrides=[
        Parameter('max_rate_hz', value=10.), Parameter('max_source_age_sec', value=.8)])
    peer = Node('latest_frame_peer', context=context)
    source = peer.create_publisher(Image, '/camera/front/image_raw', 1)
    received = []
    peer.create_subscription(Detection2DArray, '/robot/perception/front/detections', received.append, 10)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    executor.add_node(peer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    def send():
        image, _ = capture()
        image.header.frame_id = 'front_camera_optical_frame'
        image.header.stamp = peer.get_clock().now().to_msg()
        source.publish(image)
        return image.header
    try:
        wait(lambda: source.get_subscription_count() == 1)
        original = send()
        wait(entered.is_set)
        for _ in range(12):
            send()
            time.sleep(.02)
        wait(lambda: node._stats['front']['overwritten'] > 0)
        assert len(node._frames) <= 1
        time.sleep(.8)
        latest = send()
        time.sleep(.05)
        release.set()
        end = time.monotonic() + 8
        while not received and time.monotonic() < end:
            send()
            time.sleep(.05)
        assert received
        assert maximum == 1
        assert all(item.header != original for item in received)
        assert received[-1].header.stamp.sec >= latest.stamp.sec
        assert node._stats['front']['rejected'] >= 1
    finally:
        release.set()
        executor.shutdown(timeout_sec=5)
        thread.join(5)
        node.destroy_node()
        peer.destroy_node()
        context.try_shutdown()


def test_clock_reset_clears_cached_captures_and_readiness():
    from lekiwi_rmf.object_detector import ObjectDetector

    state = SimpleNamespace(_frame_lock=threading.Lock(), _generation=0,
                            _frames={'front': object()}, _last_stamp={'front': 100},
                            _stats={'front': {'published_stamp': 100}})
    ObjectDetector._clock_reset(state, None)
    assert state._generation == 1
    assert not state._frames and not state._last_stamp
    assert state._stats['front']['published_stamp'] == 0


def test_cli_uses_the_pinned_private_runtime(monkeypatch, tmp_path):
    from lekiwi_rmf import object_recognition

    python = tmp_path / 'bin/python'
    monkeypatch.setattr(object_recognition, 'model_runtime', lambda name: python)
    with pytest.raises(ValueError, match='install the selected model'):
        object_recognition.use_model_runtime('rtdetr_v2_r18')
    python.parent.mkdir()
    python.touch()
    calls = []
    monkeypatch.setattr(object_recognition.os, 'execve', lambda *args: calls.append(args))
    object_recognition.use_model_runtime('rtdetr_v2_r18')
    assert calls[0][0] == python
    assert calls[0][2]['PYTHONNOUSERSITE'] == '1'
    object_recognition.use_model_runtime('yolox')
    assert len(calls) == 1


def test_diagnostics_reads_clock_before_frame_lock_to_avoid_jump_deadlock():
    from rclpy.time import Time as ClockTime
    from lekiwi_rmf.object_detector import ObjectDetector

    lock, published = threading.Lock(), []
    def now():
        assert not lock.locked()
        return ClockTime(nanoseconds=100000000000)
    state = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=now), _frame_lock=lock,
                            _stats={'front': {'reason': '', 'published_stamp': 0}},
                            _durations={'front': []}, _detector=SimpleNamespace(sha256='a' * 64), _fallback=None,
                            _bindings={'front': {'sha256': 'b' * 64}}, _last_stamp={},
                            _frames={}, _age=2., _diagnostics=SimpleNamespace(publish=published.append))
    ObjectDetector._health(state)
    assert published[0].status[0].message == 'observation output stale'


def test_operating_point_parameters_reject_runtime_changes_instead_of_silently_ignoring_them(monkeypatch):
    from lekiwi_rmf import object_detector

    monkeypatch.setattr(object_detector, 'Recognizer', lambda *args: SimpleNamespace(predict=list, sha256='a' * 64, default_query=None, labels=[]))
    context = Context()
    rclpy.init(context=context)
    node = object_detector.ObjectDetector(context=context)
    try:
        result, = node.set_parameters([Parameter('confidence_threshold', value=.3)])
        assert not result.successful and 'confidence_threshold' in result.reason
        assert node.get_parameter('confidence_threshold').value == .5
        assert node.set_parameters([Parameter('use_sim_time', value=False)])[0].successful
    finally:
        node.destroy_node()
        context.try_shutdown()
