# Autonomous exploration follow-up, 2026-11-16

Completed: reviewed and merged exploration bootstrap PR #184 (main 0d706b5).
It initializes native mapping before requiring a first map/pose, including an
unlocalized saved map, and retains all driving checks. Final ROS/recovery matrix
69 checks passed. A bounded live Explore task reached native execution with zero
search coverage, paused on safety-source faults, and canceled with confirmed stop.

Reviewed/merged search geometry latency PR #185 (main 276efa0); its former head bae6578 keeps capture-time metadata through
inference latency and bounds RGB-D/range sync. 16 geometry + 79 launch checks pass;
local long search deadlines also fail unchanged main on this busy host. Hosted
checks passed before merge. Native real-sensor A/B produced 49 vs 30 maps.

RViz shutdown PR #186 head 1d369a1 fixes repeated image-filter disconnect and
keeps the configured Cyclone implementation loaded for the process lifetime.
Native disconnect cases and 31 collision/snapshot checks pass. Real-camera
SIGTERM/SIGINT probes with the final plugin returned zero (7.05 s / 2.80 s).
The first CI run caught the new native test bypassing isolation; registration was
corrected and ten policy checks plus the native isolated regression pass.
Source and failure evidence are retained; no robot-service restart occurred.

Combined clean candidate: release/robot-autonomy-20261116,
a27b65745539c05ce6ed9a45a598fa90b5738750. Both role staging units are running;
compute log ~/.ros/lekiwi/stage-autonomy-a27b657.log, device log
~/.ros/lekiwi/stage-device-autonomy-a27b657.log. Older candidate 725a519 compute
staging was stopped before activation; its device 16/16 passed. Next: seal both roles;
review/merge #186 after green hosted checks, verify final tree equivalence,
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
A diagnostic turn was NOT submitted: no fresh clear readiness was seen; the latest
scan sample no longer contains the earlier 298.81-degree edge hit, and StopZone
is now persistently active for a different nearby return. Current camera/scan
evidence is being captured before any motion or filter changes.

Private evidence: ~/.ros/lekiwi/qualification/20261116-bootstrap/.
Remove WIP before final delivery; all task source branches are pushed/clean.
