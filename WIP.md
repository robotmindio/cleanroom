# Astra image pairing

- Completed: shared device compressor pairs RGB/depth by capture stamps within 50 ms before encoding, publishes each pair at the existing 2 Hz rate, keeps independent collision-cloud path unchanged. Tests cover both arrival orders, exact boundary, unmatched/invalid inputs, original codecs/stamps and rate.
- Verified: 31 targeted cloud/filter/configuration tests pass; configured flake8 passes.
- Next: isolated Pi canary on private output topics without restarting camera/motor services; verify pairing rate/skew, then CI/sealed releases/review/deployment.
- Pending: independent metric calibration target/measurements; no metric or recognition qualification claimed. Authorized 30 cm origin remains 20261030-three-objects/qualified-origin.json; no robot motion or torque cuts.
