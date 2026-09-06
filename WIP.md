# Robot description consistency

Completed:
- Vendored LeKiwi `88f5cb7`; its generated Xacro and mount manifest agree.
- The fixed camera optical frame points along `base_link` +X.
- Gazebo streams interruptible arm setpoints through a native watchdog.
- Simulated sensor frames use bridge-supported frame overrides.
- CTest owns one isolated Python/ROS environment helper.
- Stale RViz cameras, readiness assertions, and safety documentation were corrected.

Verified:
- All 41 CTests pass, including physical Gazebo arm, watchdog, sensor-frame,
  MoveIt, model, vendor, trajectory, and camera coverage.
- Simulation SDF renders without parser warnings.
- The reference image is generated from URDF materials.

Pending hardware work:
- Confirm Astra, fixed-camera, and LIDAR alignment and complete arm calibration
  when robot-1 is online.

Next:
- Run the physical calibration checklist before enabling movement on robot-1.
