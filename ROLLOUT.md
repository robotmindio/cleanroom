# Agent search rollout, 2026-11-16

## Delivered

- PRs #180–#188 merged with green final hosted checks. Search enablement is tracked
  in supported native Astra mapping topologies. Explore initializes mapping before
  requiring an initial map/localized pose. Search preserves capture-time geometry
  through inference latency; frustum projection batching retains the full octree
  volume proof. RViz safely disconnects image filters and retains Cyclone at exit.
  Deployment now requires fresh SRDF travel_stow before every torque cut, including
  post-restart disarm and retries, within the unchanged 0.02 rad tolerance.
- Applied the tracked Fiber service union to the actual organization tenant,
  preserving settings, and created its missing default stationary profile.
  Verified schema-2 calls and scoped stationary goal completion.
- PR #183 records the operator-measured supported_rest and recognizes existing
  MechanicalRest pairs around either reference within unchanged pair-relative
  bounds. A guarded physical MoveIt recovery returned the arm to travel_stow.
  Actual RViz MotionPlanning widgets and move_group received the semantic and
  selected RestFCL library. RViz's later shutdown fix passed native regression
  and real-camera SIGTERM/SIGINT probes with zero exit status.
- Release `fa91885a069d92d48e9d247811fd0010763033d4` passed canonical **106/106
  compute and 16/16 device**, sealed inventory verification and canonical split
  deployment. Both current pointers and verified deployed-revision markers select
  fa91885; previous is 2d87543 on both hosts. Main dbe89b6 has the same source tree.
  The first deployment failed a post-restart DISARMED check on transient LINK_LOST;
  after read-only recovery/readiness checks, one canonical retry succeeded and
  rearmed. Both attempts are preserved; no safety limit was relaxed.
- Two post-deployment Explore tasks were admitted/dispatched exactly once. Radius
  1.5 m returned immediately with no target; the existing 1.1 m stopping margin
  leaves only 0.4 m useful radius. Radius 2 m produced physical movement, then
  StopZone/depth freshness pauses. It was canceled with stop confirmed, no retained
  resources and no target visited. Odometry over its admission-to-terminal interval
  measured **0.251153 m integrated travel and 0.165840 m net displacement**.
  This establishes bounded movement and cancellation, not completed exploration.
- The earlier Fiber search trial returned no_coverage and was canceled with stop
  confirmed. Repeated cancel retained its receipt; resubmission of that same ID
  was not admitted. Zero usable coverage is not evidence of object absence.

## Remaining acceptance

- StopZone remains intermittent. The latest stationary scene includes 0.206–0.234 m
  returns and nearby chair casters; their identity is not fully established.
  The user has been asked to clear nearby objects. Keep motion stopped pending
  clearance and fresh checks. Do not shrink the polygon or mask unverified returns.
- Continuous capture-time search geometry remains unqualified. A post-deployment
  30 s stationary sample caught multi-stream gaps up to 17.4 s, map gaps of 19.34 s
  and invalid-source stops. Pi DNS/controlplane resolution fails and Tailscale
  uses DERP; clock checks were healthy. No single cause is established and no
  DNS/firewall configuration changed. The explicit tailnet motor guard remains.
- Main RTAB-Map's live sync_queue_size is 15, but its installed implementation does
  not declare approx_sync_max_interval. The launch-provided 0.15 value is therefore
  **not verified as effective**. The live parameter probe preserves null and fails
  its assertion. The standalone RGB-D synchronizer's separate interval setting
  does not prove the mapper's RGB-D/scan interval is bounded.
- Draft PR #189 (9503b92) implements the missing native interval without changing
  the subscriber ABI. Native message regressions and an isolated packaged-mapper
  ABI/parameter probe pass, as does the complete native package build. Compute
  staging and hosted CI are active;
  Pi staging and deployment await connectivity. The fix is not selected.
- The Pi subsequently became unreachable over both tailnet and LAN, and a fresh
  eight-second probe received no joints/images. The user has been asked about
  power/Wi-Fi without powering off. Restore connection and verify fresh stow,
  scene, odometry/localization and readiness before any torque cut or movement.
- High-level mobile find still needs an approved map ID and surveyed views.
  Physical calibration, useful-view search evidence, broader navigation/perception
  acceptance, mapless physical exploration and trained arm skills remain open.
  Production map data was not erased for a test. RViz endurance remains open.

Current private evidence is under
`~/.ros/lekiwi/qualification/20261116-bootstrap/`; earlier search proofs and the
consistent pre-activation map backup are under `20261116-agent-search/`.
Deployment logs are `~/.ros/lekiwi/deploy-autonomy-fa91885{,-retry}.log`.
`explore-fa91885-trial2-summary.json` links the terminal task/odometry metrics;
the raw JSONL and task journals are retained. The first trial's recorder failed
serialization and supplies no physical odometry proof. Failed staging attempts,
the guarded Pi retirement receipts and successful replacement qualification are
preserved separately.

Robot has no managed task held. Its last confirmed ARMED/travel_stow state is
historical; current posture/torque cannot be verified during the outage. Own compute staging
remains active; no deployment or recorder is
active. Source branches are clean
and pushed; the operational handoff/WIP records the remaining follow-up.
