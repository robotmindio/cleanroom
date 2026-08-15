from math import cos, sin


def integrate_pose(pose, velocity, dt):
    """Integrate body-frame x/y/yaw velocity into a world-frame 2D pose."""
    x, y, yaw = pose
    vx, vy, wz = velocity
    return (
        x + (vx * cos(yaw) - vy * sin(yaw)) * dt,
        y + (vx * sin(yaw) + vy * cos(yaw)) * dt,
        yaw + wz * dt,
    )
