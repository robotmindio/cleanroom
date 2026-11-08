# Native qualification observations — 2026-11-08

The perception trials below used sealed release
`7f51fed145694a51aa13afdb0b613069653bb5c6`. The robot was not commanded to move.
Private images and full traces are retained under
`~/.ros/lekiwi/qualification/20261108-software-followup/`.

## Three positive stationary cup trials

The existing disabled `config/fiber_perception_wrist_canary.json` binds the
GPU recognizer to the wrist and the tracked `near_field` proposal profile.
The profile was applied through its optimistic-concurrency ROS service and
verified before admission. In daylight, all three consecutive objectives
finished `done/found`; admission to verification took 6.28, 4.13 and 5.46 seconds.

Each result contains two distinct, stationary captures. Their verification
ages were 1.45–1.85 seconds, separations 0.40–0.61 seconds and scores 0.728–0.748.
All six original JPEGs were downloaded through the goal evidence endpoint;
independently computed SHA-256 hashes matched their immutable receipts.
Retrying each request returned the same goal. The canary was disabled and
its recovery timer stopped afterward.

The 0.5 minimum score, two-second evidence/confirmation windows, 15-second
goal budget, camera/model identities and motion guards were unchanged.
The private runner's source preflight used the profile's existing two-second
age limit; an earlier stricter 1.5-second preflight did not admit an objective.
No final verifier age gate was widened.

Machine-readable results: [stationary-cup-20261108.json](evidence/stationary-cup-20261108.json).
This completes the three-positive series in this one image-only scene.
It does not qualify a general detector, a metric location, moving search,
an absent/ambiguous-object matrix or grasp/place behavior.

## Preserved failures and timing evidence

YOLOX did not supply eligible cup detections from the current top-down view.
The GPU `standard` profile recognized the cup at approximately 0.78–0.81 but
two admitted objectives ended `unverifiable`, with final reason
`fresh_unambiguous_pair_required`. These failures remain evidence; they were
not replaced by the smaller-profile successes.

For the first failed GPU objective, Temporal started its first workflow task
after 10.6 ms and verification activities promptly. The earlier long workflow
startup delay did not recur in that trial. A separate native preparation probe
returned pairs in 7 of 24 calls, with a maximum local round trip of 260 ms.
The observed standard-profile captures reached storage approximately
1.06–1.36 seconds after acquisition, leaving limited margin for an older
second capture and remote verification. This supports a freshness/transport
margin problem; it does not identify every source of delay.

The tracked `near_field` profile uses a 480-pixel shortest edge rather than
800, retaining its own distinct proposal revision and the same score gate.
Its successes establish the observed scene, not equivalent recall at lower
resolution across an independent corpus.

## Software and remaining acceptance

[PR #176](https://github.com/robotmindio/cleanroom/pull/176) requires known-free
capture-time 3D volumes for negative search evidence, supports full optical
roll/vertical views and retains bounded geometric proofs through anchor
reprojection. Its sealed revision `737312ef11b37e0b0ceb4ade74b7ba9c4743ef6b`
passed 105/105 compute CTests, 16/16 Pi CTests and full hosted CI/CodeQL.
Search remains disabled; live search/SLAM optimization and calibrated
detectability are still required for #88 and #91.

The separate isolated failover qualification and stage measurements are in
[provider recovery](perception-provider-recovery.md). They do not establish
independent GPU saturation or an actual server/network outage for #102.

Current frames were illuminated, unlike the previous day's dark samples;
darkness is not a current blanket blocker. The measuring tape was not visible
in these captures. Metric calibration, physical sensing/braking/arm acceptance,
independent hard-condition labels, DDS/deployment authorization decisions and
formal site/operator acceptance remain in [DEFERRED.md](../DEFERRED.md).
The intermittent busy-receipt failure #148 was not reproduced by the complete
native or hosted test runs; no speculative runtime fix is claimed.
