# Live qualification, 2026-10-28

Deployed revision: `f6598a1de867a946fba95194c8a6c24582075493`, split Pi/compute robot,
ROS domain 0. The operator authorized robot control in the safe test environment.
This is a diagnostic and recovery record, not physical acceptance. Existing speed,
collision, freshness and stow gates remained active. Production perception remains
disabled pending independent qualification.

Private images, logs and JSON reports are retained outside Git under
`~/.ros/lekiwi/qualification/20261028-live-review/`.

## Health and recovery

- All five Pi services were active. Compute systemd was active, but its controller,
  planner and collision monitor had exited on October 26 with
  `cannot store a negative time point in rclcpp::Time`. The cause of that exception
  remains unresolved; service activity alone did not establish navigation health.
- Fresh joint states confirmed `travel_stow` before stopping compute. Following
  the backup/restore drill, the managed compute service was restarted and all
  three Nav2 nodes reached lifecycle `active`. The collision monitor subsequently
  published 1,300 state messages over a 65-second observation window.
- Front, wrist and Astra RGB arrived at approximately 2 Hz. Measured image-age
  p95 was 52, 52 and 31 ms respectively. The stationary wheel-odometry trace had
  negligible translation. Astra depth contained 9.37% positive finite pixels in
  this view; this does not establish metric camera calibration.

## Data restore drill

The original backup silently omitted observations and spatial memory on domain 0:
their owners write `observations_0.sqlite3` and `spatial_0.sqlite3`, while the data
tool looked for unsuffixed names. `robot_data.store_paths` now matches each owner's
convention; the task journal retains its existing unsuffixed domain-0 filename.

Using the corrected tool, an online backup included all four stores. With compute
stopped, a fresh backup was restored to the robot's real paths. Previous files
were preserved as `.pre-restore-*`. Restore and separate verification both reported
`healthy: true`, SQLite integrity and schema checks passed, indexes were rebuilt,
and all 525 observation records matched their evidence digests (zero mismatches;
`visual_evidence: verified`). The permission audit reported no findings.

Regression fixtures now use the owners' filenames independently of the backup
helper and cover real and simulated stores on domains 0 and 5.

## Perception

The remote provider reported ready, but its operating-point hash was
`0986e83de3b27f7fa69634a0d13270681423446b248338da60b984a9a0c0a87e`;
the tracked pin is
`54f38a2f87dff6dba126ce8095a826b8a270cb235f08214b4687d7c93c40de0d`.
Its responses were refused and RT-DETR served fallback frames. No pin or
freshness limit was changed to admit these responses.

A temporary run used the repository perception launcher with front and Astra
cameras, then stopped it. In the final sampled latency window:

| Measurement | p50 (s) | p95 (s) | p99 (s) |
| --- | --- | --- | --- |
| Sensor age | 0.033 | 0.056 | 0.060 |
| Queue | 0.190 | 0.493 | 0.514 |
| Inference | 0.353 | 1.555 | 1.927 |

The detector counted 151 fresh and four late answers; the late answers were
rejected by the existing two-second gate. Network timing had no samples because
the primary provider responses were not admitted. Skill, navigation and managed
task latency diagnostics had zero samples during this collection. These are
fallback measurements, not completion of the normal-load/saturation suite in #102.

The retained patterned-mug development manifest produced three correct cup
detections, no false positives and no misses with RT-DETR; processing p95 was
0.596 s. All three images belong to one scene group. Threshold calibration
returned `insufficient_corpus` / `hard_conditions_below_minimum` and wrote no
configuration. #98 therefore remains unqualified: the required independent,
human-labeled hard-condition corpus is absent.

## Navigation and spatial memory

A production-configured Nav2 smoke trial requested a goal 0.10 m forward using
the existing test helper. It did not finish within the helper's 40-second result
deadline; the helper requested cancellation and sent zero manual commands.
Maximum measured displacement from the start was 0.1133 m. No safety-supervisor
faults were recorded. The trial failed; reaching a nearby position did not count
as action success. A separate slow guarded return restored the starting pose.

Spatial memory was queryable after restart, but initially reported a new map
identity and no relocalization timestamp. This does not satisfy #91's stable-map,
loop-closure reprojection, merge, relink or capture/graph-race requirements.

## Remaining acceptance

Resolve the Nav2 negative-time failure and the failed short navigation trial;
qualify the GPU provider's new runtime before changing its pin; collect the
independent camera corpus; then repeat perception saturation and managed-task,
skill and navigation latency measurements. #88, #91, #98 and the full #102
acceptance remain open. Existing physical acceptance limits are unchanged.
