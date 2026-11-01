Front evidence cadence trial

- Confirmed: local RT-DETR labels the real patterned cup as vase; explicit remote cup query recognizes it at score >= 0.5. Three local, three remote-standard and three remote-near-field deployed goals failed safely.
- Measured: camera receipt sensor p95 0.048 s, wait p95 0.459 s, inference p95 0.443 s, remote network p95 0.856 s. The 2 Hz front bridge and serial remote inference leave too little margin for two captures plus WAN verification.
- Cadence-only trial: 5 Hz front, 4 Hz detector reduced queue p95 to 0.196 s, lidar p95 age 0.122 s at 4.97 Hz, but all three deployed goals failed. Restored ordinary bridge; no promotion.
- Candidate now: 5 Hz front, original 2 Hz inference starts, two bounded remote requests in flight on separate reusable connections. Local models stay exclusive. Query/profile snapshots, per-thread timings, cooldown and publication freshness preserved. No age/model/score changes.
- Verified: 121 affected ROS/HTTP/profile/config, 71 store and 3 observation-node tests passed (195 total); Ruff/whitespace pass. Three added regressions fail the serial baseline; the local exclusive control passes.
- Live: built the isolated overlay with repository build-lekiwi.sh --device. Installed detector SHA256 116097e0... equals source. Ten of twelve local compound reads returned two fresh hash-checked, pinned captures; ordinary bridge restored afterwards. Three actual overlap goals and three preset-query controls still failed deployed admission. This is a native throughput improvement, not a complete latency fix.
- Next: inspect complete activity timings and qualify an alternate wrist view on local CPU; do not claim end-to-end acceptance or promote the cadence-only trial.
- Pending: Temporal scheduling incident (171.5 s first task delay) and permanent clock fallback installation (compute sudo expired). Do not claim either fixed by this trial.
