# Troubleshooting

## Arm model moves opposite to the robot

The arm's folded-CAD-zero capture and its joint directions are separate calibrations. Do
not change directions merely because the model looks wrong while the arm is at the
captured zero pose: a direction sign has no effect at zero. In that case, recapture
the vendor CAD's folded home pose with `scripts/calibrate.sh pose` instead.

To validate a direction after a successful zero-pose capture:

1. Keep the area around the arm clear and support it if needed.
2. In RViz's **MotionPlanning** panel, select planning group **arm**, open the
   **Joints** tab, and set **Start State** to **Current**.
3. Preview exactly one joint by `+0.10` rad (about 5.7 degrees) from its displayed
   current value. Do not change any other joint.
4. Click **Plan** and confirm the green preview makes the intended small movement.
   Then execute it, or use `scripts/arm-jog.sh shoulder_pan +0.1` (substitute the
   tested joint). The jog tool requires confirmation and moves only that one joint.
5. If that one physical link moves opposite to the preview, set only that joint's
   value in `directions` to `-1` in `~/.ros/lekiwi_arm_calibration.json`; leave its
   `zero_positions` value unchanged. Restart with `scripts/up.sh` and repeat the
   same one-joint test.

Test joints one at a time. A `-0.10` rad target is the corresponding opposite
direction test.

## Navigation goal does not move the base

Do not send a goal until the robot is on a clear, level floor and an operator is
present. Navigation needs all three live inputs: `/scan`, `/map`, and Nav2.

```bash
ros2 topic info /scan
ros2 topic info /map
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

`/scan` must have one publisher and ranges beyond `0.11 m`; a scan in which every
beam is `0.11` is the camera-scan fail-safe and Nav2 will deliberately not move.
All three lifecycle nodes must report `active [3]`. Use a goal inside the visible,
mapped free space—not beyond an unknown map boundary. A fresh RTAB-Map map grows
only after the robot has observed the area, so its first goal should be nearby.

If RTAB-Map reports `Did not receive data since 5 seconds`, stop the stack and
start it again with `scripts/up.sh`. The startup sequence delays the camera-scan
and RTAB-Map nodes until the camera is streaming. Old RTAB databases are archived
for 14 days and then automatically pruned at startup.

## Wrist or front camera disconnects while the arm moves

`Error dequeueing buffer: No such device (19)` means the USB camera reset or
lost electrical contact. The stack and `scripts/calibrate.sh camera|wrist` now
supervise each V4L2 camera: they stop the dead node, wait for its stable
`/dev/v4l/by-id/...` path to return, and reopen it automatically. Keep the
calibration window open; it resumes receiving frames after reconnection.

If the by-id device does not reappear, software cannot restore power or a loose
cable. Stop moving the arm, reseat the wrist-camera cable and strain relief, then
wait for the device to return. Repeated resets at one arm position are a hardware
fault (cable flex, connector, hub, or power), not a ROS calibration problem.

## Host reports every motor as model `777`

`777` is LeRobot's no-response sentinel. If every ID from 1 through 9 is
missing while the expected `/dev/serial/by-id/...USB_Single_Serial...` port still
exists, ROS is not the cause: the serial adapter is visible but the shared servo
bus returned no data. Do not rerun motor calibration.

With the robot powered off, check the battery/power switch, servo-bus power lead,
and the USB-to-servo-controller data cable. Restore power and reseat those two
connections, then run `scripts/up.sh` again. A successful host startup prints
`host: up`; it must do so before any ROS navigation or arm issue can be diagnosed.
