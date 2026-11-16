# Agent search rollout, 2026-11-16

## Delivered

- PRs #180–#183 merged with green hosted checks. Tracked `allow_search` is
  effective in supported native Astra mapping topologies, in both SLAM and
  Explorer. Earlier SLAM PRs #118, #176 and #179 were already merged.
- PRs #184–#186 are also merged. Explore starts mapping before requiring an
  initial map/localized pose; search retains capture-time geometry metadata
  across inference latency and bounds RGB-D/range synchronization; RViz releases
  image callbacks safely and keeps Cyclone loaded through shutdown. All final
  hosted checks passed before each merge. Main `0bff4a2` has the same tree as
  the staged autonomy candidate `a27b657`.
- Applied the tracked Fiber service union to the actual organization tenant,
  preserving existing settings, and created its missing default stationary
  profile. Verified schema-2 calls and scoped stationary goal completion.
- PR #183 records the operator-confirmed measured `supported_rest` and recognizes
  the existing MechanicalRest pairs around either reference within unchanged
  25 mm / 0.10 rad pair-relative bounds. Transport stow and torque-off rules stay
  unchanged. Its sealed planner-only reload recovered the stopped planner without
  restarting motor/device services or cutting torque.
- Rearmed through the native permission gate. A guarded physical MoveIt recovery
  returned the arm from supported_rest to travel_stow at 0.10 scaling; fresh joint
  states confirmed stow. An earlier invalid plan was rejected before execution.
- Canonical split deployment succeeded and rearmed. Both selected releases and
  verified deployment markers are `2d87543c0c3e770e68b969c6e531d01fc71c27f7`,
  whose source tree equals main merge `0b36c5c`. Compute 105/105, device 16/16,
  81 targeted Python tests, native collision regression and lint passed.
- Fresh RViz and move_group used the selected RestFCL library and semantic.
  Actual RViz MotionPlanning start/goal widgets listed supported_rest, verified
  through AT-SPI. The task-owned RViz process subsequently segfaulted on requested
  shutdown; its UI verification passed, but shutdown reliability is unresolved.
- Fiber search trial 3 was accepted by the native action server, ran, paused on
  safety/freshness gates and was canceled. Terminal state is canceled, stop is
  confirmed, resources released. Repeated cancellation retained its receipt;
  resubmission of the same task returned admitted=false and generation 51.
  The journal contains one admission and one dispatch. Odometry-derived travel
  was 0.013664 m within the 0.12 m bound. Coverage was zero, claim no_coverage;
  this is no evidence of object absence. Trials 1/2 were safely rejected by StopZone.

## Remaining acceptance

- Astra now produces real depth (~17,400 valid pixels per 320x240 frame), and
  stationary native OctoMaps contain free/occupied leaves with matching SLAM graph
  stamps. Their cadence is intermittent: the fixed-window one-second freshness
  check failed. Trial 3 credited no observations because geometry was missing or
  stale; perception and collision-source freshness also caused pauses.
- LD06 StopZone still trips intermittently. Do not shrink its polygon, mask
  unverified physical returns or bypass its stop/freshness guards. Next step is
  determine the actual nearby obstruction or measured chassis return, then repeat
  geometry/perception cadence checks and a viewpoint search with usable evidence.
- High-level mobile find goals still require an approved map ID and surveyed
  views. Physical calibration, broader perception/navigation acceptance and trained
  arm skills remain open; native search enablement does not qualify them.
- The final RViz fix passed native regression and two real-camera signal exits
  (SIGTERM/SIGINT, zero exit status); endurance qualification remains open.
  No robot service was stopped during these UI checks.
- The combined autonomy candidate is not deployed: device staging passed 16/16
  and its sealed inventory verifies, but compute staging passed 104/106.
  `test_perception_training` timed out at 300 s; two `test_search_ros` cases
  exceeded their action-result deadlines with costmap pauses. The original
  evidence is preserved privately under
  `20261116-bootstrap/staging-a27b657-failed/`. Investigate and obtain a complete
  passing compute qualification before selection; do not expand deadlines or
  safety limits to pass. A fresh post-deployment Explore test is still pending.

Private proofs and a consistent pre-activation map backup are in
`~/.ros/lekiwi/qualification/20261116-agent-search/`. Canonical deployment log is
`~/.ros/lekiwi/deploy-supported-rest-2d87543.log`. Failed earlier attempts are
retained. Task source branches are clean and pushed; no source implementation is
left uncommitted. The robot remains armed in travel_stow with no managed task held.
