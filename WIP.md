# 0.25 m/s physical qualification

Completed: shared 0.25 stage, native directional test zones, wheel conversion fix, independent metrology and detached recovery timers (PRs 43–50).
Verified: one 0.25 forward stop at 0.242 m/s; scan-loss distance 0.221 m and stop-time bound 1.252 s. The scan-loss attempt is excluded because modeled uncertainty 0.045159 m exceeded its 0.04 m declaration. Stage 0.25 now reserves 0.05 m; production stays 0.03/0.06.
Next: deployment 83cf67c (`lekiwi-025-measured-reserve-deploy`), then queued `lekiwi-025-full-acceptance`: recover reference run 20261009-121900, new forward stop, all moving faults, five stops per direction.
Pending: full target acceptance, measured final zones/exploration margin, production configuration/acceptance PR, deployment and live parameter/permission verification. Do not raise production limits before those measurements pass. Remove this file before final delivery.
Artifacts: .benchmarks/base-speed-qualification/025-measured-reserve-deploy.log and 025-full-acceptance.log; .benchmarks/onboard-braking/; committed measurement reports in docs/.
