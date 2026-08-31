# LeKiwi robot description

`lekiwi_cad.urdf` and the matching CAD mesh set are vendored from
[`robotmindio/LeKiwi`](https://github.com/robotmindio/LeKiwi). The exact
source revision and input/output digests are tracked in
`lekiwi_cad.manifest.json`; this deployable snapshot does not require a CAD
checkout at runtime. Refresh it explicitly, then verify it before committing:

```bash
python3 scripts/vendor-lekiwi-cad.py --source ../LeKiwi --write
python3 scripts/vendor-lekiwi-cad.py --source ../LeKiwi
```

The source checkout's `URDF/LeKiwi.urdf` and `URDF/meshes/` must be clean.
The RobotSkin LD06 mount/body belongs to the ROS wrapper, not this upstream
CAD snapshot.

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
