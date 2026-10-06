# Arm payload trial — 2026-10-06

**Result: the reported 200 g installation passed a controlled arm cycle at
0.10 velocity and acceleration scaling, after correcting the contact rule.**
The earlier stopped trial below is retained as diagnosis history. The arm was
recovered in software with motor torque enabled, reached HOME, held there for
60 seconds and returned to travel_stow. Final MoveIt validity was true, driver
ARMED and both permissions true.

Attachment location and whether the reported mass includes the holder remain
unconfirmed. This verifies this installed load and tested route/speed; it is
not a maximum payload rating, retention measurement or endurance test. Subsequent
loaded rotations and production Nav2 goals passed; see
[the base test record](base-payload-verification-20261006.md). Physical braking
acceptance remains recorded for 0 g added payload until loaded stops are measured.

## Corrected restriction and powered recovery

The real held pose was confirmed valid by the operator. The old plugin revoked
all resting-contact permissions when any non-pan arm joint left a 0.10 rad
neighborhood of travel_stow. Coupled shoulder/elbow movement crossed that
threshold even though the contacting parts remained near their resting
placement. An unrelated wrist roll could also revoke forearm/shoulder contact.
The previously proposed physical reset was unnecessary.

RestFCL now bounds each explicitly listed pair's relative transform: at most
25 mm displacement and 0.10 rad orientation difference from travel_stow. The
observed release/current poses displaced the relevant frames by up to 17 mm;
the displacement bound includes an 8 mm margin. The shoulder rotation part /
wrist roll holder was added as a fourth bounded resting interface because the
confirmed valid pose also produced that modeled contact. Other pairs, the
floor, chassis and world obstacles retain collision checks. The physical CAD,
HOME, travel_stow, controller tolerances and servo gains were unchanged.

Only MoveIt was respawned to load the new library and collision pairs. The Pi
motor-host PID stayed **631571** before and after recovery and the complete
loaded cycle. No motor service was restarted or torque-off request issued.
All nine servos reported torque enabled in the before/after snapshots and
recorded movement/hold samples.

The reload utility addresses systemd `PrivateTmp`: the planner's parameter file
is accessed through `/proc/<pid>/root`, not the shell's unrelated `/tmp`. It
preserves initialized live parameters and calibration, reads optional limits
individually because uninitialized declarations reject a whole parameter batch,
and omits empty array defaults that lose their type when rendered as YAML `[]`.
It waits for the replacement parameter service and verifies the new semantic
description. Initial reload attempts exposed these errors; the final planner
was restored and its live validity service confirmed the held pose valid.

Collision code/configuration was built from `57d20c0`; the completed compute
installation is `832181b`. This was a selective compute planner update. The Pi
motor stack stayed on the earlier `60c518b` deployment; no Pi runtime code needed
changing. RViz was restarted with the same updated model and four bounded
interfaces; the visible MotionPlanning defaults remain 0.50 / 0.40.

## Completed loaded verification

All six MoveIt actions returned status 4 / error code 1. The measured release
waypoint was `[0, -1.73, 1.52, 1.12, -0.0169]`; medium reach was
`[0, -1.00, 0.85, 0.80, -0.0169]`. Gripper commands were unchanged. The base
received no navigation goal or test velocity command.

| Stage | Execution seconds | Hold seconds | Final maximum joint error, rad |
| --- | ---: | ---: | ---: |
| Release from the formerly blocked pose | 1.825 | 10 | 0.00559 |
| Medium reach | 5.108 | 15 | 0.00267 |
| HOME | 6.329 | 60 | 0.00921 |
| Return through medium reach | 6.898 | 7 | 0.01224 |
| Return through release | 5.169 | 5 | 0.00689 |
| travel_stow | 1.968 | 15 | 0.00922 |

| Observation over the completed cycle | Result |
| --- | ---: |
| Non-OK servo diagnostic samples / supervisor faults | 0 / 0 |
| Torque-off samples | 0 |
| Maximum servo temperature | 48 °C, gripper |
| Test ceiling / programmed temperature limit | 60 / 70 °C |
| Peak reported motor current | 370.5 mA, shoulder lift |
| Maximum controller tracking error during movement | 0.05282 rad |
| HOME hold maximum target error after first two seconds | 0.00921 rad |
| HOME hold largest joint variation, peak to peak | 0.01534 rad |
| Folded hold largest joint variation, peak to peak | 0.00307 rad |
| Maximum base displacement in wheel odometry | 9.15 mm |

The hold variations are joint telemetry, not an independent tip displacement
measurement. Current is reported servo telemetry, not calibrated joint torque.
The wrist view remained obscured; payload attachment and retention were not
independently observed. Higher speeds, other load placements, higher masses,
repeated cycles and longer thermal endurance remain untested. The subsequent
loaded base test covers the bounded route described in its separate record.

Regression verification: **18 Python tests passed** and the native resting
contact test passed, including the full collision matrix at both recorded
loaded poses, unrelated-roll isolation, outside-neighborhood rejection and
retained floor exclusions. The final eight-second snapshot found travel_stow
collision-valid, all safety permissions healthy and no supervisor fault.
Finite test and diagnostic clients exited; normal services and RViz remain.

Evidence is under `.benchmarks/arm-payload-200g/`: `verified-summary.json`, the
six `20261006-1442*` through `20261006-1446*` action directories,
`after-reload.json`, `final-snapshot.log`, `host-before.txt`, `host-final.txt`,
`final-model-tests.log`, `rest-regression.log` and `rviz-folded.png`.

## Earlier stopped trial

Runtime was `60c518b8ddd3`. HOME, travel_stow, servo gains, collision exceptions
and controller tolerances were unchanged. The arm started collision-valid in
travel_stow with both permissions true, driver ARMED and servo temperatures
35–38 °C. The wrist camera was obscured by the payload; the external USB view
did not include the robot, so attachment/retention was not visually verified.

## First loaded movement

The finite test requested a small lift through MoveIt's `/move_action`, using
0.10 velocity and acceleration scaling. Its target arm coordinates were
`[0, -1.60, 1.40, 1.20, -0.0169]`, corresponding to a tool height increase from
about 15.6 to 18.5 cm. This candidate was endpoint-valid, but it was a different
release route from the measured waypoint in `test/test_rest_contacts.cpp`.
Gripper commands were not changed and the base was commanded by no test route.

Execution stopped after 1.890 seconds, returning action status 6 and MoveIt error
-4. The workspace monitor reported
`shoulder_holder_collision_proxy/roll_holder_collision_proxy`; existing safety
logic withdrew permission and logically disarmed the driver. The arm remained
held by enabled motor torque. No wider move or long hold was attempted.

| Observation | Result |
| --- | ---: |
| Largest recorded controller feedback error | 0.02309 rad |
| Wrist tracking lag at contact onset | 0.01839 rad |
| Maximum reported servo temperature | 38 °C |
| Configured servo temperature limit | 70 °C |
| Maximum reported current, elbow | 201.5 mA |
| Non-OK hardware diagnostic samples | 0 |
| Maximum wheel-odometry base displacement | 0.261 mm |

Current/load values are servo telemetry, not calibrated torque measurements.
There was no observed electrical or temperature fault. The arm did not reach
the requested target, so its held target error cannot establish payload capacity.

## Collision diagnosis

Forty-four recorded/candidate states were checked with the live MoveIt validity
service, without commanding motion. All recorded desired states were valid.
Two recorded actual states and the held pose were invalid due to the same
shoulder/wrist-holder pair, with reported overlap depths 0.428 and 0.721 mm.

The resting-contact plugin permits the measured contact only while each arm
variable other than shoulder pan stays within 0.10 rad of travel_stow. At
contact onset the actual elbow had left that neighborhood while wrist lag
kept the wrist bracket close to the shoulder. This establishes the observed
tracking/clearance failure; it does not establish that the CAD overlap was
harmless physical contact or that payload mass caused the lag. The obscured
views could not resolve that distinction.

The existing measured release waypoint
`[0, -1.73, 1.52, 1.12, -0.0169]` checked valid. The next trial should use that
release route, checking clearance with a tracking allowance before attempting
the larger lift. An endpoint validity check alone did not cover the actual
motion with its tracking error.

## Superseded recovery assessment

The initial model blocked ordinary MoveIt movement, prompting a proposed
supported torque-off reset. The operator subsequently confirmed that the held
real pose was valid. Correcting the contact rule enabled powered recovery and
the completed sequence above. No physical reset or torque cut was needed.

Ignored local evidence is in `.benchmarks/arm-payload-200g/`: `preflight.json`,
`20261006-132314-arm-release/result.json`, `blocked-preflight.json`,
`recorded-path-validity.json` and `summary.json`. Finite test and diagnostic
clients exited; no agent monitoring process remains.
