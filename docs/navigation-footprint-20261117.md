# CAD-derived navigation footprint

The previous 46 x 44 cm rectangle included empty space around the folded robot.
On 17 November, the stationary base reported continuous `StopZone` stops from
front-left laser points at x=0.177–0.229 m, y=0.171–0.219 m. All seven points in
the recorded scan were outside the projection of the physical CAD. Intersecting
their laser rays with the CAD at the live joint positions found no physical
self-return. The live laser transform agreed with the expanded Xacro.

Those are external measurements, not points to erase from the scan. The old
rectangle incorrectly classified them as already overlapping the body.

## Geometry

`scripts/generate-navigation-footprint.py` expands the unchanged physical CAD and
ROS Xacro. It includes every vendored physical visual mesh and the mounted Astra
body. Virtual collision keepouts and the visual chassis keepout marker are not
physical parts and are excluded. Mesh source hashes are recorded in
[`evidence/navigation-footprint-20261117.json`](evidence/navigation-footprint-20261117.json).

The generator projects every physical part into `base_footprint` and bounds all
joint angles within the production `travel_stow` tolerance of 0.02 rad. It uses
per-vertex directional joint derivatives and a conservative Taylor remainder.
For ancestor joints j <= k, the mixed derivative is `axis_j × J_k`; its norm is
bounded by the perpendicular reach of distal joint k. That reach can grow by at
most the sum of distal `2*sin(tolerance/2)*radius` displacements. Summing these
bounds over the Hessian and multiplying by `tolerance²/2` encloses every
intermediate joint angle. The independent 5 mm model allowance is retained.
Sixteen supporting lines are rounded outward to a millimetre and intersected.
The certificate records both the original motion bounds and the tighter Taylor
remainders (at most 0.898 mm).

The generated body has a 0.136 m inscribed radius and a 0.236128 m maximum radius.
Both costmaps, the overlap stop and predictive braking use this same polygon.
Explore's centre prefilter uses its inscribed radius; its full-body route checks
remain active. The existing 1.10 m task-region stopping reserve and 0.33 m RMF/map
reservation remain unchanged. No sensor masking, stow tolerance, velocity limit,
freshness gate or task budget is relaxed.

Regenerate the certificate from the source checkout:

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/generate-navigation-footprint.py \
  --output docs/evidence/navigation-footprint-20261117.json
```

The generator produces geometry only; it cannot grant physical acceptance.

## Braking and degraded cameras

The stopping evidence, payload/surface/sensor conditions, maximum velocities,
stop latency and 60 mm measurement uncertainty in `safety_acceptance.yaml` are
retained. Its expected geometry is corrected to the CAD certificate. With the
smaller physical radius, conservatively expressing the original rotational
stopping-distance bound as a time requires a longer predictive horizon:

`max(0.447666/0.25, 0.307951/(0.236127100*0.40)) + 0.05 + 1.524829 < 5 s`.

`FootprintApproach` therefore uses 5 seconds, including a separate reserve for
the measured command-stop latency. It still considers the commanded
translation and rotation; nearby points outside the swept body permit travel,
and actual overlap or stale laser data still stops the base.

The wrist camera is optional in the camera launcher and supervisor. Production
object perception selects the front camera. Its missing USB device does not
block Explore, navigation, SLAM or front-camera perception. A skill explicitly
requiring wrist images must still reject missing or stale wrist input.

## Verification

The geometry regression regenerates the certificate and checks all physical
parts at interior joint angles. The existing arm-envelope test also checks the
full tolerance box at its corners. The real native Nav2 collision-monitor
fixture exercises the recorded front-left returns while stationary and moving
forward/reverse, actual body overlap, projected collisions and stale scans.
These isolated tests do not establish new physical stopping performance.

The attended speed-qualification fixture retains its original larger rectangle,
directional braking reserves and bounded test limits in
`base_speed_qualification.yaml`; refining production geometry cannot shrink that
fixture. The read-only startup checker also limits lifecycle discovery/RPCs to
4 Hz per node, independently of incoming sensor callback frequency, while
retaining its existing reply timeout and readiness/freshness criteria.

The first physical trial with an isotropic joint-displacement expansion stopped
on a return at (0.181711, 0.068426) m. Its x+y was 0.250138 m, while every physical
CAD part at the measured pose stayed below 0.222941 m. The directional bound
retains the full joint/model allowances while excluding this excessive isotropic
margin. The measured return remains an obstacle for predicted motion toward it.
