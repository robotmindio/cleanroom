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


def test_rgb_encoding_is_eager_rate_limited_and_keeps_capture_stamp(monkeypatch):
    import cv2
    import rclpy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image
    from types import SimpleNamespace
    from lekiwi_rmf.astra_cloud_filter import AstraCloudFilter

    rclpy.init()
    node = AstraCloudFilter()
    try:
        from rclpy.qos import ReliabilityPolicy
        subscriptions = {s.topic_name: s.qos_profile.reliability for s in node.subscriptions}
        assert subscriptions["/camera/astra/color/image_raw"] == ReliabilityPolicy.RELIABLE
        assert subscriptions["/camera/astra/depth/image_raw"] == ReliabilityPolicy.BEST_EFFORT
        frames = []
        publisher = SimpleNamespace(publish=frames.append)
        clock = iter([10.0, 10.1, 10.6, 11.0])
        monkeypatch.setattr("lekiwi_rmf.astra_cloud_filter.time.monotonic", lambda: next(clock))
        image = Image(width=2, height=2, encoding="bgr8", step=6, data=bytes(12))
        image.header.stamp.sec = 7
        for _ in range(3):
            node._on_image(serialize_message(image), "color", ".jpg", "jpeg", publisher)
        assert len(frames) == 2
        assert all(f.header.stamp.sec == 7 and f.format == "jpeg" for f in frames)
        assert cv2.imdecode(np.frombuffer(frames[0].data, np.uint8), cv2.IMREAD_COLOR).shape == (2, 2, 3)
        _,expected = cv2.imencode('.jpg',np.zeros((2,2,3),dtype=np.uint8),[cv2.IMWRITE_JPEG_QUALITY,70])
        assert bytes(frames[0].data) == expected.tobytes()
        depth = np.array([[0, 456], [1000, 2345]], dtype=np.uint16)
        image = Image(width=2, height=2, encoding="16UC1", step=4, data=depth.tobytes())
        node._on_image(serialize_message(image), "depth", ".png", "16UC1; png compressed", publisher)
        assert np.array_equal(cv2.imdecode(np.frombuffer(frames[-1].data, np.uint8), cv2.IMREAD_UNCHANGED), depth)
    finally:
        node.destroy_node()
        rclpy.shutdown()
