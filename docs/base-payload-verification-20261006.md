# Loaded base verification — 2026-10-06

**The installed, operator-reported 200 g passed loaded rotations and two
production Nav2 goals. Physical stopping acceptance is not yet complete.**
The arm remained in travel_stow. Tests used the normal running production stack,
within the authorized 30 cm radius, at its existing 0.03 m/s and 0.06 rad/s
limits. Motor services were not restarted and no torque-off request was issued.
This covers the installed load; holder mass and attachment location were not
independently verified. It is not a maximum payload rating.

## Completed movement tests

| Test | Result |
| --- | --- |
| Counterclockwise turn | Wheel yaw 0.2845 rad; independent LiDAR yaw 0.2871 rad |
| Clockwise turn | Wheel yaw -0.2780 rad; independent LiDAR yaw -0.2661 rad |
| Return heading | Wheel yaw -0.0081 rad; LiDAR yaw -0.0119 rad |
| Nav2 forward and return | Both action results status 4, succeeded |
| Forward translation observed by onboard RGB-D odometry | 46.80 mm |
| Maximum Nav2 test radius, wheel / RGB-D odometry | 51.59 / 48.58 mm |
| Final position error after the bounded manual correction, wheel odometry | 3.63 mm |
| Nav2 test supervisor fault samples | 0 |

Yaw values are the recorded observer coordinates, not zero-subtracted angles.
The rotation client paused for a scan/depth freshness fault and for fresh
independent Astra tracking, then recovered and completed. The Nav2 run recorded
no supervisor fault. Both runs ended ARMED, arm_stowed true, both motion
permissions true. Onboard observations verify this functional route; they do
not establish the independent stopping measurement uncertainty. These runs do
not claim a new SLAM loop closure.

## Remaining physical acceptance

Two finite external-camera preflights found no measurement tags: the USB view
did not include the robot. They issued no movement and left production running.
Loaded stopping trials need the two 35 mm chassis markers and 44 mm black floor
marker visible in a fixed camera view. Re-aiming changes calibration, so obtain
a fresh floor reference rather than reusing the old camera transform.

Once the measurement view is ready, use the tracked runner:

```sh
export LEKIWI_WS=/home/nex/lekiwi_ws
source scripts/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/test-braking.py --production --payload-g 200
```

This records the declared mass in both the run profile and result. Its
production option preserves running ROS and motor services. Each of six
directions needs five qualified stopping trials, plus affected moving fault
cases. Review frame freshness, achieved speeds, the 20 mm measurement
uncertainty and the unchanged 50 mm / 1.5 s stop budgets before registering
`payload_kg: 0.2`. Record actual loaded counts and new evidence; do not relabel
the historical unloaded measurements.

`config/safety_acceptance.yaml` still identifies 0 kg as the load used in its
physical stopping evidence. That field is not a runtime payload comparison or
an instruction to refuse the completed 200 g arm/navigation tests. Runtime
permissions were true during the loaded Nav2 test.

The production runner regression passed: **2 tests**, including rejection of
invalid mass and preservation of running services/lifecycle nodes. Finite
movement and camera clients exited; normal robot services and RViz remain.

Raw functional observations and client logs are hashed in
[the evidence manifest](base-payload-evidence-20261006.json). Raw artifacts are
retained under `.benchmarks/arm-payload-200g/` and
`.benchmarks/onboard-verification/`; the manifest does not contain camera images.
