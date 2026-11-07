# Native qualification observations — 2026-11-07

These observations apply to deployed release
`bbdf3863a973c13d270d143c16fabc0bab7cbe28`. They supplement the
[acceptance backlog](../DEFERRED.md); they do not extend physical acceptance.
Private source artifacts are retained under
`~/.ros/lekiwi/qualification/20261107-completion/` on the compute host.

## Data recovery

With fresh `travel_stow` confirmed and `lekiwi-stack.service` stopped, the
release's `scripts/robot-data.py` completed backup, restore, separate verify
and permission audit against the real domain-0 paths. All four stores were
included: task journal, observations, spatial memory and arm calibration.
Previous files were retained as `.pre-restore-*` copies.

Restore and separate verification reported `healthy: true`. SQLite integrity
and supported schemas passed, indexes were rebuilt, and all **16,557**
observation records matched their retained image hashes, with zero mismatches
and `visual_evidence: verified`. Permission audit returned no findings.
Compute restarted and `stack_readiness --exploration --armed` passed, including
another check while the isolated simulation was running. The Pi motor service
was not restarted and arm torque was not cut.

Evidence: `data-drill/result.json`, `data-drill/readiness.log` and
`readiness-under-simulation.log`. This closes the repeated recovery drill,
not the latency and saturation measurements still required by issue #102.

## Arm workspace gate

The canonical workspace-gate test inserted a temporary MoveIt collision object
at the wrist while the physical arm remained stowed. MoveIt reported contacts
with five CAD collision links. The arm action was rejected, measured joint
change was zero, and the object was removed. Permissions recovered and fresh
joint states confirmed `travel_stow` afterward. Base odometry excursion was
0.3 mm. Evidence: `workspace-gate/`.

This tests the software gate against an explicit planning-scene intrusion;
it does not establish physical obstacle coverage, stopping distance or payload
retention.

## Transport authentication

Read-only state requests with the registered CURVE client passed before and
after the negative tests. Plaintext and newly generated, unregistered CURVE
clients received no state response. A TLS 1.3 connection to the sensor bridge
without a client certificate was rejected with
`TLSV13_ALERT_CERTIFICATE_REQUIRED`. TLS 1.2 was disabled; its protocol-version
rejection is not client-authorization evidence. No motor commands were issued.
Evidence: `network-negative-auth.json` and its bounded test script.

These observations do not establish firewall enforcement, rosbridge
application authorization, tailnet peer policy or credential rotation.

## Stationary perception limit

The pinned optional YOLOX wrist pilot supplied two distinct, fresh,
non-degraded cup observations before admitting goal
`16962f82-3ce4-583b-9d7b-d393eb90ae53`. Retrying the same request returned the
same goal. The goal remained `starting` and then finished `unverifiable`;
it is **not a positive end-to-end acceptance**. The goal was confirmed
terminal, the pilot profile disabled and the base returned to its recorded
anchor. Evidence: `fiber-yolox-centered/` and `yolox-live-position/result.json`.

No detector score, input-quality, freshness or physical safety threshold was
relaxed. One successful scene does not qualify a general recognition model.

## Stationary perception follow-up on release 5791f96

The later compute/Pi deployment uses
`5791f96c3e49ec1eff01d29cabe50a231ca730d7`; its tree is identical to main's
`ce088ac19f339eac1ef19f45d72d926ce2e9f2f2`. Fresh armed readiness and
`travel_stow` passed before the tests below. No physical motion was required.
Follow-up artifacts are private under
`~/.ros/lekiwi/qualification/20261107-resume/`.

The JPEG-mode goal `4592a753-6ca3-5fb9-98f8-757a6569c8c6` finished
`unverifiable`. Its first workflow task started after 48 ms; five verification
activities ran within 16.54 seconds. The last four checks each retained only
one fresh qualifying capture. This failure was not a workflow queue delay.
The native source supplied the admitted camera, model and profile identities,
stationary captures, no input-quality flags, and cup scores above 0.5.

Both installed endpoints already support native SHA-256 receipts. The owning
Fiber integration was still configured for JPEGs. The tracked
`config/fiber_native_evidence_receipts.json` patch was applied through the
owning integration API with its complete configuration preserved. Read-back
confirmed that only `observation_evidence_format` changed. Credentials were not
logged or committed. Capture selection, two-second freshness, two-capture
requirement, 0.25-second separation and the 15-second goal budget stayed intact.

Goal `50e95a6d-e05f-5bff-a160-2c9dd4ce3bd7` then completed `done/found`, with
verifier version 3, 10.57 seconds after admission. The two cup captures were
0.604 seconds apart and 1.557/0.953 seconds old at verification. Both original
JPEGs were subsequently downloaded through the goal evidence endpoint; their
independently computed SHA-256 digests matched the immutable goal evidence.
The same request returned the same goal on retry. Evidence:
`fiber-native-retry/result.json`, `admitted-0.json`, and its two retained JPEGs.

The intended series of three successes remains incomplete. A preceding goal's
first workflow task timed out and retried at 75.82 seconds; the next goal after
the success also exceeded the pilot's wait. Both were canceled, their API
states confirmed terminal, and the pilot profile disabled. Temporal's server
reported persistence deadlines, while read-only PostgreSQL inspection observed
WAL write/sync waits. These observations identify a separate server-side
latency problem; they do not establish its storage root cause or qualify the
recognizer across scenes. No timeout or safety gate was relaxed to count a
success.
