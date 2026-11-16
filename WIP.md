# Agent search rollout: supported-rest recovery

## Completed

- Merged PRs #180, #181 and #182, with green hosted CI. Native search uses the
  effective tracked switch in both SLAM and Explorer, only in supported mapping
  topologies; hardware occupancy uses the native Astra RGB-D origin.
- Applied the tracked Fiber integration service union to the actual organization
  tenant, preserving its existing settings. Verified schema-2 validation/events,
  odometry and spatial reads. Created the missing default stationary profile from
  its tracked configuration and verified scoped stationary goal completion.
- Canonical release `ebb7d8f6dc79d0600e61f0aeacd13a58f50fb1a5` passed compute
  105/105 and device 16/16. Its source tree equals merged main `b55085e`.
- Both hosts now select that sealed release; both retain `f503b96` as previous.
  Fresh joint states confirmed travel_stow before every torque-cutting restart.
- Actual Fiber capabilities advertise search and validate a schema-2 request
  bounded to 10 seconds and 0.12 m. No motion task was submitted.

## Blocker and current state

The Astra depth obstruction was removed: a direct read now has about 17,400 valid
pixels per 320x240 frame. The disarmed arm settled into a different stable posture,
confirmed by the operator as supported. Its 120 fresh joint samples have zero
spread. It fits the tracked footprint in CAD, but the installed RestFCL only knows
transport stow and rejects three existing MechanicalRest interfaces there. The
arm is NOT in travel_stow; do not cut torque or restart motor/device services.

PR #183, branch `fix/measured-supported-rest`, commit
`2d87543c0c3e770e68b969c6e531d01fc71c27f7`, adds the measured supported_rest
reference with the same bounded pair-relative tolerances. It preserves transport
stow/acceptance and adds a sealed planner-only reload path without torque changes.
81 targeted Python tests, native collision regression and repository lint pass.
Hosted CI and full compute release staging are running. No runtime changes from
this PR have been applied.

Canonical deployment therefore failed its required re-arm and did not update
either `deployed-revision` marker; both still record `f503b96`. The selected source
and the deployment-verification marker intentionally differ until readiness is
proved. Release seals on both selected hosts were verified after this failure.

Private evidence, map backup and read-only runtime probe are in
`~/.ros/lekiwi/qualification/20261116-agent-search/`. Logs include the failed first
full attempt (teleop interpreter teardown crash and MoveIt search deadline) and
the subsequent complete passing run; these intermittent failures are not claimed
fixed by rerunning.

## Next

1. Finish sealed compute/device qualification and hosted CI for #183; merge the
   exact green head. Do not rebuild any selected/sealed release.
2. With fresh DISARMED state, use the tracked new `reload-moveit.py --release`
   against its qualified compatible compute release. It locks deployment,
   verifies both seals/settings/dependencies and restricted source changes,
   checks managed current paths and verifies the replacement's mapped library.
   Require the actual supported pose to pass MoveIt; rearm only through the
   existing permission gate, then plan/execute a guarded return to the unchanged
   accepted travel_stow. Confirm that posture with fresh joint states.
3. From the clean, pushed `fix/measured-supported-rest` worktree, rerun
   `scripts/deploy-split.sh nex@100.107.20.66 --workspace /home/nex/lekiwi_ws --rosbridge-tailnet`.
   Both release seals already exist. Restore SSH authentication using the existing
   key without exposing it. Before any torque cut, require fresh travel_stow again.
4. Require canonical armed readiness and matching deployment markers. Run the
   read-only `check-search-runtime.py` in the evidence directory against current:
   native parameters, nonempty free/occupied voxels, matching graph stamps and
   geometry no older than one second must pass.
5. Submit one explicitly identified, bounded native search through Fiber, cancel
   that same task, poll to terminal stop confirmation and check journal/idempotency.
   Never interpret admission or a cancel receipt as completion.
6. Remove this WIP record before final delivery. Keep the physical calibration,
   approved site-map/views for high-level mobile goals and trained arm skills
   outstanding until their real acceptance evidence exists.
