import math
import struct

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
