# Autonomous exploration follow-up, 2026-11-16

Current state: [HANDOFF-2026-11-16-autonomia.md](HANDOFF-2026-11-16-autonomia.md),
14:30 UTC continuation. PRs #184–#188 merged with green final checks.
Release fa91885 passed canonical compute 106/106 and Pi 16/16, sealed/verified,
and is selected with verified deployment markers on both hosts. Main dbe89b6
has the same source tree. The successful canonical retry rearmed the robot;
the first post-restart LINK_LOST failure is preserved in its original log.

Explore generation 53 (radius 1.5 m) returned immediately with no targets.
Generation 54 (radius 2 m) physically moved: interval odometry measured 0.251153 m
integrated travel, 0.165840 m net displacement. It paused on StopZone/depth freshness
and was canceled with stop confirmed and all resources free. Neither task visited
a target. Do not replay either ID or interpret region area as map growth.

Robot is stopped, ARMED, in travel_stow (last fresh maximum error 0.009253 rad).
StopZone remains intermittent; nearby chair casters are visible. A user request
to clear the chair/nearby objects is pending. Keep motion stopped until clearance
and fresh readiness checks. No collision masks, margins or freshness gates changed.

Remaining technical findings: multi-stream transport gaps up to 17.4 s and map
gaps of 19.34 s, Pi DNS failures/DERP relay, and the installed main RTAB-Map node
does not declare the launch-provided approx_sync_max_interval=0.15. Queue depth
15 is effective, but the interval bound is unverified/ineffective. No new source
fix or manual parameter/network override was applied. The explicit LAN motor
firewall restriction stays intact.

Private evidence: ~/.ros/lekiwi/qualification/20261116-bootstrap/.
No task-owned movement, recorder, staging or deployment process remains active.
Source branches are clean and pushed. Keep WIP while physical/technical follow-up
remains; remove at completed delivery and update ROLLOUT.md with final results.
