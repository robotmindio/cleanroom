# Three-object physical qualification

- Completed: inspected all worktrees (clean), confirmed deployed a9d07ef and ARMED/stowed state; captured all three cameras. Front view is occluded by a close mug. Guarded 6 cm retreat completed within the original reference's conservative bounds.
- Next: run all-camera production recognizers, inspect three objects, evaluate fresh stationary observations and managed objective contracts; fix reproduced issues and qualify deployment.
- Verified: fresh camera frames, active production stack, no torque interruption; original 30 cm boundary preserved.
- Pending: perception accuracy/latency, provider identity and availability, end-to-end evidence/recovery, broader physical coverage. No new qualification claims yet.

- Findings: shared GPU retry window lost degradation reason on other cameras (regression reproduced; fixed; 78 tests pass). Current provider hash is 0986e83..., recomputed from the exact pinned weights plus its advertised runtime; candidate scene tests still show recognition errors and latency excursions. Do not promote this candidate on compatibility alone. Fiber stationary API returns `robot profile not configured` (409); no goal admitted.
