# Perception continuation

- Completed: recovered PR120 deployment/test evidence; all worktrees clean. New interleaved HTTP trial (16 per transport) preserves detections and shows a material warm-connection latency improvement.
- Completed: persistent HTTP implemented at RemoteProvider; 87 targeted tests passed plus profile configuration check. Official Fiber connector configured with additive observation allowlists and a disabled, stationary profile. Live query/evidence hash checks passed; disabled goal admission returned 409.
- Next: finish source-verified camera canary (the initial launch loaded the installed module and is baseline only), then review/CI/deploy the sealed revision. New profile model identity must be checked with PYTHONNOUSERSITE=1 in the deployed perception runtime.
- Verified: robot runtime active; prior 30 cm authorization reference is retained in qualification/20261030-three-objects/qualified-origin.json. No motion or torque cuts in this phase.
- Pending: sustained latency/freshness, independent scene qualification, Fiber journey, independent metric calibration and later manipulation.
