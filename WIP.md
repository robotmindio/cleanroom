Front evidence cadence trial

- Confirmed: local RT-DETR labels the real patterned cup as vase; explicit remote cup query recognizes it at score >= 0.5. Three local, three remote-standard and three remote-near-field deployed goals failed safely.
- Measured: camera receipt sensor p95 0.048 s, wait p95 0.459 s, inference p95 0.443 s, remote network p95 0.856 s. The 2 Hz front bridge and serial remote inference leave too little margin for two captures plus WAN verification.
- Cadence-only trial: 5 Hz front, 4 Hz detector reduced queue p95 to 0.196 s, lidar p95 age 0.122 s at 4.97 Hz, but all three deployed goals failed. Restored ordinary bridge; no promotion.
- Candidate now: 5 Hz front, original 2 Hz inference starts, two bounded remote requests in flight on separate reusable connections. Local models stay exclusive. Query/profile snapshots, per-thread timings, cooldown and publication freshness preserved. No age/model/score changes.
- Verified: 121 affected ROS/HTTP/profile/config tests passed; Ruff and whitespace checks passed. Regression comparison and live overlap qualification pending.
- Next: run source scripts with candidate config on both hosts, compare native pair ages and deployed goals, verify lidar freshness, restore existing runtime after trial.
- Pending: Temporal scheduling incident (171.5 s first task delay) and permanent clock fallback installation (compute sudo expired). Do not claim either fixed by this trial.
