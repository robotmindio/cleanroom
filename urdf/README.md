# LeKiwi robot description

`lekiwi_cad.urdf` and `meshes/` are vendored from
[`robotmindio/LeKiwi`](https://github.com/robotmindio/LeKiwi), commit
`efa608d7ee5a495a4803b1d28cd0c955b4f1e033` (2026-08-05). They are the CAD
export for the physical V1 LeKiwi assembly and retain the stable nine joint
names documented upstream.

The ROS copy makes three intentional, documented adaptations: drive-wheel
joints are fixed because the hardware publishes no wheel encoders; the five
bounded arm servos and gripper are revolute with the same limits enforced by
the driver; and the CAD triangle collision meshes are removed. The latter are
far too dense for real-time MoveIt (they starve navigation); arm collision
proxies must be introduced as simple, measured geometry rather than restoring
the raw meshes. `lekiwi.urdf.xacro` supplies the resulting conservative
low-poly base/tool/arm collision proxies. Their dimensions are CAD-derived
starting envelopes and must be physically verified before relying on close
self-collision plans.

`lekiwi.urdf.xacro` is the ROS wrapper. It must remain the description loaded
by launch and MoveIt. The wrapper intentionally owns only ROS integration:

- REP-103 `base_footprint` and `base_link` frames;
- normalized front/wrist camera optical frames;
- the optional LD06 lidar mount and `tool0` frame.

Do not replace CAD geometry or kinematic transforms with primitive stand-ins.
If the physical build uses a different mount, adjust the named wrapper frame
and record the measurement; never edit CAD joint origins to compensate for a
camera, lidar, or calibration error.
