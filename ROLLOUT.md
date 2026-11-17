# Current rollout checkpoint — 17 November 2026, 11:07 MYT

PRs #190/#191 merged; main 5f0b79f matches qualified source e74683f. Canonical
qualification passed 109/109 compute and 16/16 Pi, both seals verified, and all five
hosted checks passed. e74683f is deployed on both hosts with previous=9503b92,
readiness twice and normal rearm after fresh stow checks.

The USB failure was the Ubuntu 1068 VL805 DMA regression. A pinned exact-kernel
backport to 40 bits restored Astra/lidar and survived a guarded reboot; the wrist
camera also recovered. Supported torque-off poses can use guarded USB recovery;
normal MoveIt returned the arm to nominal stow at 0.1 scaling. Powered torque cuts
retain the strict 0.02 rad stow requirement. All safety limits and CAD are intact.

Final robot: ARMED, travel_stow error .012322 rad, fresh front/wrist/Astra and
scan/depth, collision clear, all resources free. Mapper sync interval is really
0.15. Pi has ~2.3G free after a verified journal archive and safe vacuum.

Generation 56 completed fixed exploration initialization, then returned no further
reachable observation targets. It visited 0 targets and moved exactly 0 m over
129 odometry samples. This is startup/termination proof; useful physical base
exploration and search coverage remain unqualified. Readonly inspection found no
reachable targets for radii 2–4 m in the observed map. Investigate known-space
connectivity and map/pose consistency while keeping safety limits. Broader find,
calibration and trained arm skills remain open; reach/skills availability stays off.

No own temporary process or goal remains active. See the handoff for exact IDs,
logs, backup hashes, failed diagnostics and continuation instructions.
