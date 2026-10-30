import math
import struct

import numpy as np
import pytest

pytest.importorskip("rclpy")
from sensor_msgs.msg import PointCloud2, PointField

from lekiwi_rmf.astra_cloud_filter import compact_cloud


def test_compact_cloud_decimates_and_drops_invalid_xyz():
    cloud = PointCloud2()
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 2, 4, 12, 48
    cloud.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    points = [(float(column), float(row), 1.0) for row in range(2) for column in range(4)]
    points[2] = (math.nan, 0.0, 1.0)
    points[4] = (math.nan, 0.0, 1.0)
    cloud.data = b"".join(struct.pack("<fff", *point) for point in points)

    compact = compact_cloud(cloud, 2)

    assert compact is not None
    assert compact.height == 1
    assert compact.width == 1
    assert compact.row_step == compact.point_step
    assert struct.unpack("<fff", compact.data) == (0.0, 0.0, 1.0)


def _reference_compact(message, stride):
    """The straightforward per-point decimation the vectorised version must equal."""
    offsets = {field.name: field.offset for field in message.fields}
    unpack = struct.Struct((">" if message.is_bigendian else "<") + "f")
    output = bytearray()
    for row in range(0, message.height, stride):
        for column in range(0, message.width, stride):
            start = row * message.row_step + column * message.point_step
            xyz = [unpack.unpack_from(message.data, start + offsets[name])[0] for name in "xyz"]
            if all(math.isfinite(value) for value in xyz):
                output.extend(message.data[start:start + message.point_step])
    return bytes(output)


@pytest.mark.parametrize("big_endian", [False, True])
@pytest.mark.parametrize("stride", [1, 3, 4, 12])
def test_vectorised_compaction_matches_the_per_point_reference(stride, big_endian):
    rng = np.random.default_rng(7)
    height, width, point_step, padding = 11, 14, 20, 8
    row_step = width * point_step + padding  # rows carry trailing padding
    end = ">" if big_endian else "<"
    raster = bytearray(rng.integers(0, 256, height * row_step, dtype=np.uint8).tobytes())
    for row in range(height):
        for column in range(width):
            start = row * row_step + column * point_step
            xyz = [float(rng.normal()) for _ in range(3)]
            if rng.random() < 0.25:
                xyz[rng.integers(0, 3)] = float(rng.choice([math.nan, math.inf, -math.inf]))
            # x, y, z are not the first fields, and the rest is arbitrary payload.
            raster[start + 4:start + 16] = struct.pack(end + "fff", *xyz)
    cloud = PointCloud2()
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = height, width, point_step, row_step
    cloud.is_bigendian = big_endian
    cloud.fields = [
        PointField(name=name, offset=4 + index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate("xyz")
    ]
    cloud.data = bytes(raster)

    compact = compact_cloud(cloud, stride)

    expected = _reference_compact(cloud, stride)
    assert compact is not None and bytes(compact.data) == expected
    assert compact.width * compact.point_step == len(expected)


def test_a_field_that_overruns_the_point_is_rejected():
    cloud = PointCloud2()
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 1, 12, 12
    cloud.fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in (("x", 0), ("y", 4), ("z", 10))
    ]
    cloud.data = bytes(12)

    assert compact_cloud(cloud, 1) is None


def test_blind_frame_is_published_empty_with_its_capture_stamp():
    cloud = PointCloud2(height=1, width=2, point_step=12, row_step=24)
    cloud.header.stamp.sec = 7
    cloud.fields = [PointField(name=name, offset=index * 4,
                              datatype=PointField.FLOAT32, count=1)
                    for index, name in enumerate("xyz")]
    cloud.data = struct.pack("<fff", math.nan, math.nan, math.nan) * 2
    compact = compact_cloud(cloud, 1)
    assert compact is not None and compact.width == 0 and compact.height == 1
    assert not compact.data and compact.row_step == 0 and compact.header.stamp.sec == 7


def test_sparse_returns_between_sampling_pixels_are_kept_within_the_budget():
    cloud = PointCloud2(height=8, width=8, point_step=12, row_step=96)
    cloud.fields = [PointField(name=name, offset=index * 4,
                              datatype=PointField.FLOAT32, count=1)
                    for index, name in enumerate("xyz")]
    points = [(math.nan, math.nan, math.nan)] * 64
    for index in (9, 18, 27):
        points[index] = (0.1, 0.2, 0.6)
    cloud.data = b"".join(struct.pack("<fff", *point) for point in points)
    compact = compact_cloud(cloud, 4)
    assert compact.width == 3 <= 4
    assert bytes(compact.data) == struct.pack("<fff", 0.1, 0.2, 0.6) * 3


def test_valid_frame_exceeding_budget_cannot_become_an_empty_measured_frame():
    cloud = PointCloud2(height=8, width=8, point_step=12, row_step=96)
    cloud.fields = [PointField(name=name, offset=index * 4,
                              datatype=PointField.FLOAT32, count=1)
                    for index, name in enumerate("xyz")]
    points = [(math.nan, math.nan, math.nan)] * 64
    for index in range(9, 17):
        points[index] = (0.1, 0.2, 0.6)
    cloud.data = b"".join(struct.pack("<fff", *point) for point in points)
    compact = compact_cloud(cloud, 4)
    assert compact.width == 4
    assert bytes(compact.data) == struct.pack("<fff", 0.1, 0.2, 0.6) * 4


@pytest.fixture
def image_filter(monkeypatch):
    import rclpy
    from types import SimpleNamespace
    from lekiwi_rmf.astra_cloud_filter import AstraCloudFilter

    rclpy.init()
    node = AstraCloudFilter()
    frames = {"color": [], "depth": []}
    node._image_publishers = {c: SimpleNamespace(publish=values.append) for c, values in frames.items()}
    clock = [10.0]
    monkeypatch.setattr("lekiwi_rmf.astra_cloud_filter.time.monotonic", lambda: clock[0])
    try:
        yield node, frames, clock
    finally:
        node.destroy_node()
        rclpy.shutdown()


def send_image(node, camera, stamp_ns):
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image

    if camera == 'color':
        image = Image(width=2, height=2, encoding='bgr8', step=6, data=bytes(12))
    else:
        depth = np.array([[0, 456], [1000, 2345]], dtype=np.uint16)
        image = Image(width=2, height=2, encoding='16UC1', step=4, data=depth.tobytes())
    image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    node._on_image(serialize_message(image), camera)


def test_rgb_and_depth_encoding_is_eager_paired_rate_limited_and_keeps_capture_stamps(image_filter):
    import cv2
    from rclpy.qos import ReliabilityPolicy

    node, frames, clock = image_filter
    subscriptions = {s.topic_name: s.qos_profile.reliability for s in node.subscriptions}
    assert subscriptions["/camera/astra/color/image_raw"] == ReliabilityPolicy.RELIABLE
    assert subscriptions["/camera/astra/depth/image_raw"] == ReliabilityPolicy.BEST_EFFORT
    send_image(node, 'color', 7_000_000_000)
    assert not any(frames.values())
    send_image(node, 'depth', 7_025_000_000)
    color, depth = frames['color'][0], frames['depth'][0]
    assert color.header.stamp.sec == depth.header.stamp.sec == 7
    assert depth.header.stamp.nanosec == 25_000_000
    assert color.format == 'jpeg' and depth.format == '16UC1; png compressed'
    _, expected = cv2.imencode('.jpg', np.zeros((2, 2, 3), dtype=np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 70])
    assert bytes(color.data) == expected.tobytes()
    assert np.array_equal(cv2.imdecode(np.frombuffer(depth.data, np.uint8), cv2.IMREAD_UNCHANGED),
                          np.array([[0, 456], [1000, 2345]], dtype=np.uint16))
    clock[0] = 10.1
    send_image(node, 'color', 7_100_000_000)
    send_image(node, 'depth', 7_100_000_000)
    assert len(frames['color']) == len(frames['depth']) == 1
    clock[0] = 10.6
    send_image(node, 'depth', 7_600_000_000)
    send_image(node, 'color', 7_600_000_000)
    assert len(frames['color']) == len(frames['depth']) == 2


@pytest.mark.parametrize('first', ['color', 'depth'])
def test_unmatched_frames_are_replaced_until_capture_stamps_are_within_50ms(image_filter, first):
    node, frames, _ = image_filter
    other = 'depth' if first == 'color' else 'color'
    send_image(node, first, 7_000_000_000)
    send_image(node, other, 7_200_000_000)
    assert not any(frames.values()) and set(node._pending_images) == {other}
    send_image(node, first, 7_149_999_999)
    assert not any(frames.values())
    send_image(node, first, 7_150_000_000)
    assert len(frames['color']) == len(frames['depth']) == 1
    assert not node._pending_images


def test_an_invalid_image_cannot_publish_half_a_pair(image_filter):
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image

    node, frames, _ = image_filter
    send_image(node, 'color', 7_000_000_000)
    invalid = Image(width=2, height=2, encoding='32FC1', step=8, data=bytes(16))
    invalid.header.stamp.sec = 7
    node._on_image(serialize_message(invalid), 'depth')
    assert not any(frames.values()) and not node._pending_images
    send_image(node, 'color', 0)
    send_image(node, 'depth', 7_000_000_000)
    assert not any(frames.values())
    send_image(node, 'color', 7_000_000_000)
    assert len(frames['color']) == len(frames['depth']) == 1
