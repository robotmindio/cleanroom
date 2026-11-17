# Current rollout checkpoint — 17 November 2026, 09:51 MYT

9503b92 is live on compute/Pi after complete 107/107 + 16/16 qualification and
canonical deployment with fresh stow checks and two successful readiness checks.
Native 0.15 sync interval is now enforced. VL805 kernel regression is repaired
and persistent. MoveIt returned the supported pose to exact travel_stow without
further manual adjustment. See the handoff for source IDs, receipts and limitations.

PR #190 is draft at 67530c3 with 139 passing relevant local tests and Ruff; final
hosted build-test-lint is pending. The new Explore trial generation 55 stopped
before motion because localization->mapping changed map coordinates; stop and
resource release confirmed. That startup-frame issue and wrist camera USB -71
remain open. A Pi journal archive is the only own ongoing diagnostic process.

The following earlier checkpoint is retained as history and is superseded above.

---

# Agent search rollout

Checkpoint: 17 November 2026, 00:07 MYT / 16 November, 16:07 UTC.

## Delivered

- PRs #180–#188 merged with green final hosted checks. Search enablement is tracked
  in supported native Astra mapping topologies. Explore initializes mapping before
  requiring a first map/localized pose. Search preserves capture-time geometry
  across inference and batches frustum projection while retaining the full octree
  volume proof. RViz safely disconnects image filters and retains Cyclone at exit.
  Deployment requires fresh SRDF travel_stow before every torque cut, including
  retries and post-restart disarm, within the unchanged 0.02 rad tolerance.
- Applied the tracked Fiber service union to the actual tenant, preserved settings
  and created its missing default stationary profile. Verified schema-2 calls,
  scoped stationary goal completion, fencing and cancellation/resource receipts.
- PR #183 records operator-measured supported_rest within unchanged MechanicalRest
  pair bounds. A guarded physical MoveIt recovery returned the arm to travel_stow.
  Actual RViz MotionPlanning widgets and move_group received the semantic and
  RestFCL library. The later shutdown fix passed native and real-camera signal probes.
- Release fa91885 passed canonical **106/106 compute + 16/16 Pi**, sealed inventories
  and canonical split deployment. Both markers/pointers were verified; previous was
  2d87543. One post-restart LINK_LOST failure and its successful canonical retry are
  preserved. The successful deployment rearmed; no safety limit was relaxed.
- Two post-deployment Explore tasks were admitted/dispatched once each. Radius 1.5 m
  returned no targets; the existing 1.1 m stopping margin leaves 0.4 m useful radius.
  Radius 2 m moved, then paused on StopZone/depth freshness and was canceled with
  stop confirmed and all resources free. Interval odometry measured **0.251153 m
  travel and 0.165840 m net displacement**, with zero targets visited. This proves
  bounded movement/cancellation, not completed exploration or map growth.
- **PR #189 merged as b8984e4** after all five hosted checks and canonical compute
  **107/107** (806.34 s), sealing and independent inventory verification. Its tree
  equals qualified head 9503b92. Pinned RTAB-Map sync now enforces the previously
  ignored 0.15 s interval without changing the subscriber ABI. Six native sync
  scenarios, three invalid-value cases and the real packaged-mapper ABI/parameter
  probe passed. This fix is merged but **not deployed**; actual Pi qualification
  remains required before selection. Author reviews were labeled self-review.

## Remaining acceptance

- The Pi is unreachable over tailnet and LAN. Final stationary probe received no
  joints/images, invalid sensor sources and false motion permissions. Current arm
  posture/torque cannot be verified. Restore connectivity and verify fresh travel_stow
  before any torque cut, motor/Astra/host restart or deployment. The user has been
  asked about power/Wi-Fi without powering off, and clearing nearby objects.
- Before the outage, StopZone saw 0.206–0.234 m returns and camera-visible chair
  casters. Their full identity remains unestablished. Do not shrink the polygon,
  mask unverified returns or bypass freshness/stow/permission gates.
- Stage device 9503b92 completely and verify both role seals before canonical deploy.
  Check Pi disk space first; preserve current/previous and active releases. Then
  verify the newly launched mapper reports 0.15 and uses the new native library.
  The selected fa91885 mapper still does not declare that parameter.
- Continuous capture-time geometry remains unqualified. A post-deployment sample
  caught multi-stream gaps up to 17.4 s and map gaps of 19.34 s. Pi DNS/controlplane
  failures and DERP relay were observed, while earlier clock checks were healthy;
  no single cause is established. No DNS/firewall changes were made. Keep the
  explicit tailnet motor guard. Recheck cadence, odometry/TF/localization and scene
  after connection returns, before another bounded physical trial.
- The earlier search trial credited no usable coverage; no_coverage is not evidence
  of object absence. Useful-view search, physical calibration, broader navigation,
  mapless physical exploration and trained arm skills remain open. High-level mobile
  find needs an approved map ID and surveyed views. Production map data was retained.

Current proofs are under ~/.ros/lekiwi/qualification/20261116-bootstrap/; earlier
search proofs and a consistent map backup are under 20261116-agent-search/.
Deployment logs are ~/.ros/lekiwi/deploy-autonomy-fa91885{,-retry}.log.
The new sealed compute log is ~/.ros/lekiwi/stage-sync-9503b92.log; its unit exited
zero and is inactive. The first Explore recorder failed serialization; only the
second task has complete interval odometry/JSONL proof. Original failures and
retirement receipts are preserved.

Fiber's final resource snapshot is free, with no own movement task or temporary
staging/deployment/recorder process active. Compute robot services remain active
on fa91885. Pi selection/armed/stow observations are historical pending reconnection.
Own source changes and operational checkpoint are committed and pushed. WIP remains
for the physical/device/deployment follow-up.
