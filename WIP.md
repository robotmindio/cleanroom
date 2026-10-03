# Physical acceptance pending independent measurement and fault fixtures

Completed: thirty algorithm-eligible nominal stopping repetitions in all six directions; five optical linear and five rotation fault cases, whole-ROS crash, twelve Nav2 goals, fresh-map growth to eleven nodes and spatial closure with 35 visual inliers. Surface dry concrete, 0 kg payload, confirmed 35 mm black marker edges. HOME and compact travel_stow unchanged; production database retained.

Root fixes deployed: folded lidar mask geometry, shared test speed defaults/cleanup, 5 GHz Pi reception, 10 Hz compact depth clouds, persistent 8 MiB Pi DDS buffers, removal of MoveIt's second cloud throttle, and no replacement of a cloud awaiting its joint TF. Final 20-second pipeline probe had a 0.211 s processed-cloud maximum gap and zero workspace denials; subsequent four-goal navigation likewise had zero workspace withdrawals. Affected sensor faults were repeated successfully after those fixes.

Tracked limits: 0.04 m/s and 0.08 rad/s, 5 cm StopZone clearance, depth/perception deadlines 0.5 s. Higher 0.05 m/s depth-loss and 0.10 rad/s lidar-loss rotation exceeded the budget including the declared 10 mm uncertainty. Characterization is provisional; validated remains false and qualifying fields unset.

Next: independently bound floor/lens/timing measurement error, then complete motor-diagnostic, telemetry replay/duplicate, external obstacle and arm-workspace intrusion fault evidence before reviewing formal acceptance. Global appearance-only relocalization and whole-house accuracy are not established by this small route. Evidence: docs/physical-verification-20261003.md.
