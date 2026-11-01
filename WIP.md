Front evidence cadence trial

- Confirmed: local RT-DETR labels the real patterned cup as vase; explicit remote cup query recognizes it at score >= 0.5. Three local, three remote-standard and three remote-near-field deployed goals failed safely.
- Measured: camera receipt sensor p95 0.048 s, wait p95 0.459 s, inference p95 0.443 s, remote network p95 0.856 s. The 2 Hz front bridge and serial remote inference leave too little margin for two captures plus WAN verification.
- Candidate: front bridge 5 Hz, remote detector timer 4 Hz; wrist/lidar priorities, model/profile identities and two-second admission unchanged. Requires finite live comparison before promotion.
- Next: run source scripts with candidate config on both hosts, compare native pair ages and deployed goals, verify lidar freshness, restore existing runtime after trial.
- Pending: Temporal scheduling incident (171.5 s first task delay) and permanent clock fallback installation (compute sudo expired). Do not claim either fixed by this trial.
