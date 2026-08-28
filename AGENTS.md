# Repository guidance

- ROS 2 logs are in `/home/nex/.ros/log`; `latest` points to the newest launch directory, while per-node logs are stored directly in that log directory.
- Runtime behavior must be configured in tracked repository files and launched by the repository scripts. Do not rely on users changing RViz widgets, ROS parameters, environment variables, or ad-hoc shell commands after startup as a substitute for a repository configuration change.
- Do not add automatic disarm actions, auto-arm inhibition markers, or other autonomous motion-state changes unless the user explicitly requests that behavior.
- When changing a MoveIt/RViz default, verify the value in the newly launched RViz MotionPlanning panel as well as in `move_group`; `move_group` configuration alone does not prove the RViz plugin received it.
- `urdf/lekiwi_cad.urdf` and `urdf/meshes/` are the vendored physical CAD model. Keep ROS-only frames and calibration offsets in `urdf/lekiwi.urdf.xacro`; do not reintroduce simplified substitute arm geometry.
