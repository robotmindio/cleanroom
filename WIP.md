# Agent search rollout: physical depth blocks final verification

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

The physical Astra publishes color but all 76,800 depth pixels are zero, including
after a guarded service restart. Matching ColorOcTree publications have empty
payloads; previous flattened local grids cannot establish 3D coverage. The safety
supervisor denies arm permission for invalid depth and an unhealthy arm workspace
monitor. The driver is DISARMED, the arm is stowed, and no managed resource is held.
Do not bypass those guards or credit negative coverage.

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

1. Obtain valid physical depth with the Astra view clear in suitable lighting.
   The user was asked to put the robot in interior/shade, preserving travel_stow.
2. From the clean, pushed `fix/qualification-matrix-budget` worktree, rerun
   `scripts/deploy-split.sh nex@100.107.20.66 --workspace /home/nex/lekiwi_ws --rosbridge-tailnet`.
   Both release seals already exist. Restore SSH authentication using the existing
   key without exposing it. Before any torque cut, require fresh travel_stow again.
3. Require canonical armed readiness and matching deployment markers. Run the
   read-only `check-search-runtime.py` in the evidence directory against current:
   native parameters, nonempty free/occupied voxels, matching graph stamps and
   geometry no older than one second must pass.
4. Submit one explicitly identified, bounded native search through Fiber, cancel
   that same task, poll to terminal stop confirmation and check journal/idempotency.
   Never interpret admission or a cancel receipt as completion.
5. Remove this WIP record before final delivery. Keep the physical calibration,
   approved site-map/views for high-level mobile goals and trained arm skills
   outstanding until their real acceptance evidence exists.
