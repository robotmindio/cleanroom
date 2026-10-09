# 0.25 m/s final qualification

Completed: PR #51 deployed as 6fb9713 after 67 compute and 16 Pi checks; all GitHub CI passed. The attended 0.25 stage uses the captured held fold with unchanged 0.02 rad tolerance. No arm command was issued.
Verified: first new-reference forward stop qualifies at 0.240528 m/s, conservative distance 0.111945 m, modeled uncertainty 0.029735 m, upper stop time 0.803612 s. Evidence: docs/base-speed-stage-025-held-fold-forward-20261009.json. Production remains 0.03/0.06.
Active: lekiwi-025-held-fold-acceptance is running all eight moving faults, then completing five stops per direction from the matching profile. Log: .benchmarks/base-speed-qualification/025-held-fold-acceptance.log. First eligible run: .benchmarks/onboard-braking/20261009-132215.
Next: inspect each outcome; stop at the first failed bound. After full physical passage, size all-direction production zones/exploration/test bounds, update stow/SRDF and acceptance together with Nav2 limits, run focused checks, PR, deploy clean main, and verify live caps/permission. Remove this file before PR/final delivery.
