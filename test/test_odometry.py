import math

from lekiwi_rmf.odometry import integrate_pose


def test_integrates_body_velocity_at_heading():
    x, y, yaw = integrate_pose((1.0, 2.0, math.pi / 2), (1.0, 0.5, 0.2), 2.0)
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 4.0, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 2 + 0.4)
