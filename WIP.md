# Autonomous exploration follow-up, 2026-11-16

Completed: reviewed and merged exploration bootstrap PR #184 (main 0d706b5).
It initializes native mapping before requiring a first map/pose, including an
unlocalized saved map, and retains all driving checks. Final ROS/recovery matrix
69 checks passed. A bounded live Explore task reached native execution with zero
search coverage, paused on safety-source faults, and canceled with confirmed stop.

Search geometry latency PR #185 head bae6578 keeps capture-time metadata through
inference latency and bounds RGB-D/range sync. 16 geometry + 79 launch checks pass;
local long search deadlines also fail unchanged main on this busy host. Hosted
checks must pass before merge. Native real-sensor A/B produced 49 vs 30 maps.

RViz shutdown PR #186 head 46169ae fixes repeated image-filter disconnect and
keeps the configured Cyclone implementation loaded for the process lifetime.
Native disconnect cases and 31 collision/snapshot checks pass. Real-camera
SIGTERM/SIGINT probes with the new plugin returned zero (5.36 s / 1.68 s).
Source and failure evidence are retained; no robot-service restart occurred.

Combined clean candidate: release/robot-autonomy-20261116,
725a5196e1b61f499248290d25dedee5da5034ee. Next: stage/seal compute and device;
review/merge #185/#186 after green hosted checks, verify final tree equivalence,
then canonical split deployment from this exact qualified source.
Pi cleanup uses the guarded retirement script on four explicitly checked old
sealed builds, preserving sources/evidence and protecting current/previous.

Live selected/verified release remains 2d87543 on both hosts. The shoulder drifted
0.022 rad outside travel_stow; a fresh guarded MoveIt correction succeeded without
changing the 0.02 tolerance. Confirm fresh travel_stow again before torque cuts.
Intermittent StopZone points remain at 298.81 deg / 0.197–0.200 m, just outside the
old mask edge. Exact CAD does not identify the observed range: do not broaden
self-filtering or weaken collision/freshness gates. Recheck live exploration and
capture-time search evidence after deployment. No managed task is held.

Private evidence: ~/.ros/lekiwi/qualification/20261116-bootstrap/.
Remove WIP before final delivery; all task source branches are pushed/clean.
