# Robot description consistency

Completed:
- Vendored LeKiwi `b9bcd7e`, whose generated Xacro now expands correctly.
- The vendor rejects source Xacros that cannot be expanded.
- Arm position, velocity, and acceleration limits now come from
  `config/joint_limits.yaml`; vendoring verifies the upstream position ranges.

Verified:
- 38 focused model, vendor, trajectory, and camera tests.
- Real and simulation Xacro expansion and URDF parsing.
- Vendored model and meshes match the recorded LeKiwi revision.

Pending:
- Verify the fixed-camera optical axis against its physical CAD geometry.
- Reproduce the intermittent Gazebo arm-command delivery failure.
- Complete physical sensor placement and arm calibration when robot-1 is online.

Next:
- Investigate the fixed-camera frame without changing the established +X forward
  convention.
