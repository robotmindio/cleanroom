# Loaded base verification — 2026-10-06

**The installed, operator-reported 200 g passed loaded arm/navigation checks
and completed physical stopping revalidation. `payload_kg: 0.2` is registered
in `config/safety_acceptance.yaml`.** This covers attended operation on dry
concrete, the unchanged travel_stow and the tested installed load. Holder mass
and attachment location remain unconfirmed; this is not a maximum payload rating.

The record is installed and validates against the installed production profiles
on both compute and Pi. Matching file hashes and unchanged service PIDs are
recorded in the evidence report; installation did not restart the motor host.

## Completed checks

| Check | Result |
| --- | --- |
| Arm | Controlled cycle at 10% scaling; HOME held 60 s, then travel_stow |
| Production Nav2 | Forward/return both succeeded; observed by onboard RGB-D |
| Nominal stopping | 30 qualified loaded stops: five in each of six directions |
| Moving faults | Scan, depth, compute-command and telemetry loss: linear and angular |
| Worst stopping bound including the unchanged 20 mm error allowance | 49.752 mm, within 50 mm |
| Worst measured stopping-time upper bound | 1.199 s, within 1.5 s |
| Maximum modeled measurement uncertainty | 19.668 mm, within the declared 20 mm |
| Largest recorded displacement from the original test center | 22.814 cm in wheel odometry; authorized radius 30 cm |
| Recovery | ARMED, arm stowed, both motion permissions true |

Speeds are the maximum production commands: **0.03 m/s and 0.06 rad/s**.
Actual ground speeds are retained in the report; they are not assumed equal to
those commands. The original loaded rotations agreed between wheel and LiDAR
yaw; the Nav2 test had zero supervisor fault samples. No new SLAM loop closure
is claimed. The production database was preserved.

The front, wrist and Astra cameras supplied observations. Independent stopping
measurements used masked raw LiDAR, point-to-line registration, geometry
covariance, whole-sector sensitivity, stationary repeatability and scale/error
reserves. Missing capture windows and slowdown-zone trials were excluded.
The test center was shifted away from a nearby obstacle, retaining the collision
guard and the authorized radius. Fault baselines use sensor capture stamps;
earlier nominal arrival-based baselines were retained conservatively. Mixed
clock cases include an additional 35 ms / 2 mm reserve.

RTAB-Map 0.23.7 scales correspondence residuals into its ICP covariance;
using that directly as pose uncertainty caused a false measurement rejection.
The verification now estimates uncertainty from the actual scan geometry.
[RTAB-Map implementation](https://github.com/introlab/rtabmap/blob/0.23.7/corelib/src/RegistrationIcp.cpp).

New loaded counts and bounds replace the unloaded stopping counts for this
acceptance. Previous hardware/authentication tests remain explicitly referenced
as unchanged historical evidence; they were not relabeled as 200 g trials.
Motor torque stayed enabled and the motor host was not restarted. Finite clients
exit after testing; normal robot services and RViz remain available.

The complete report, measured windows, exclusions and raw-artifact hashes are
in [physical acceptance evidence](physical-acceptance-evidence-200g-20261006.json).
The earlier functional navigation observations remain in
[the payload evidence manifest](base-payload-evidence-20261006.json).

For future revalidation, the finite runner preserves production services:

```sh
export LEKIWI_WS=/home/nex/lekiwi_ws
source scripts/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1 \
  /usr/bin/python3 scripts/test-onboard-braking.py --payload-g 200
```

It records observations and exclusions without granting acceptance automatically.
