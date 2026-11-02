Adaptive perception recovery

- Implemented bounded exponential recovery for primary provider failures.
- Verified: 117 affected tests, two focused recovery/race tests, Ruff and whitespace checks pass. New recovery regression fails baseline.
- Earlier deployed compact-receipt batch: 2/3 positive goals, with independently checked original JPEGs and final ages below two seconds. First failure was dominated by local fallback while primary was in fixed backoff.
- Next: build this checkout, verify installed executable provenance, run isolated live tests, remove this file and publish the PR for full CI.
- All preceding finite robot pilots stopped; ordinary sensor bridge restored. No torque cuts or base commands. Native core remains B1; no custom classifier activated.
