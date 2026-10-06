# Arm payload trial — 2026-10-06

**Result: incomplete; 200 g has not been qualified.** The operator reported an
installed 200 g payload. Attachment location and whether that mass includes
the holder remain unconfirmed. No base navigation or braking acceptance was
changed; `config/safety_acceptance.yaml` remains qualified for 0 g added payload.

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

## Pending recovery and tests

- The held pose is collision-invalid and arm/base permissions are false. A
  further normal MoveIt move is blocked. Payload/wrist support must be confirmed
  before disabling torque for a physical return to the fold. Torque has not
  been cut and no collision gate has been bypassed.
- Confirm payload attachment and total added mass, restore the fold, then test
  a clearance-checked release, a longer reach/finite hold and return to stow.
- Full loaded reach, thermal endurance, grip retention, repeated cycles and
  loaded base operation remain unverified.

Ignored local evidence is in `.benchmarks/arm-payload-200g/`: `preflight.json`,
`20261006-132314-arm-release/result.json`, `blocked-preflight.json`,
`recorded-path-validity.json` and `summary.json`. Finite test and diagnostic
clients exited; no agent monitoring process remains.
