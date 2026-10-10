# Mapping and exploration from ROS or MCP

Normal visual-SLAM bringup starts `robot_explorer` with the rest of the repository
stack. It stays idle until an explicit goal arrives on `/robot/explore`, whose
type is `lekiwi_rmf/action/Explore`. AMCL has no exploration server; fixed-map
and RMF configurations refuse exploration. There is no startup movement or
change to arming, disarming, stow, payload acceptance or production speeds.

Start attended exploration with the default ten-metre radius from the starting
robot pose, for at most 15 minutes:

```bash
scripts/explore.sh
```

The launcher uses `~/lekiwi_ws/current` when a managed deployment is installed,
otherwise the checkout's build. It sources `scripts/setup.bash`, revisits known
space and prints action feedback and the terminal result. A workspace containing
a managed `current` release selects that release, including `LEKIWI_WS=~/lekiwi_ws`.
Python imports use
the selected installation, avoiding source packages hiding generated ROS types.
Ctrl-C requests cancellation of its own goal. To select another workspace:

```bash
LEKIWI_WS=/path/to/workspace scripts/explore.sh
```

The operator must remain at the motor-power stop with the accepted payload and
folded arm. The robot must fit through the planned route; braking protection
reduces speed when the moving body would approach an obstacle.

To set reduced limits or search frontiers only, source `scripts/setup.bash` and
use the existing ROS action directly:

```bash
ros2 action send_goal /robot/explore lekiwi_rmf/action/Explore \
  '{revisit_known: true, max_duration_sec: 0.0, max_radius_m: 0.0}' --feedback
```

`revisit_known: false` searches frontiers only. `true` first searches reachable
frontiers, then visits spaced observation poses in already-known space, so an
existing map can be refreshed even when no unknown frontier remains. This is
coverage of accessible observation poses, not a claim to see through closed
doors, walls or occlusions. Feedback reports the stage, current pose, successful
and failed target counts, and known map area in the bounded region.

`max_duration_sec` and `max_radius_m` equal to zero select the tracked defaults
in `config/exploration.yaml` (900 seconds and 10 metres). Positive goal values
can only reduce these maxima. The centre reachability prefilter is 0.22 m,
the body's inscribed radius (conservatively 0.25 m on a 5 cm grid). The former
0.33 m circle rounded to 0.35 m and unnecessarily blocked close side clearance.
Goals and preflight waypoints check the whole oriented 46 x 44 cm footprint
against occupied and unknown cell areas, including the interior and corners
during turns. Cells under the robot's current body count as free: the robot
occupies them, so a mapped obstacle or unknown cell there is stale or
self-observed, and a stationary robot adds no map node that could clear it.
Real nearby obstacles are handled live by the Nav2 collision monitor's stop
zone and the local costmap. The footprint comes from the same selected
Nav2 parameter file as navigation, with zero additional padding. `StopZone` is
exactly that body, with no fixed clearance beyond it. `FootprintApproach` uses
the same body and Nav2's native holonomic collision prediction to reduce speed
only when its projected motion would intersect scan returns. Parallel walls
outside the body therefore permit straight travel; turning still checks the
corners swept by the rotation. The 2.5-second horizon covers the measured
linear/angular stopping bounds and uncertainty, checked against the acceptance
record at startup. Costmap inflation is a soft cost field, not extra footprint
padding; local inflation still encloses the corners so
MPPI's native footprint-check optimization remains valid. The exploration
region retains a 1.10 m body/stopping inset; targets and preflight paths stay
inside it, and the active task ends if the live pose approaches the boundary.
This inset is only the reserve at the task's maximum travel radius, not
clearance from walls or furniture.
A new goal is rejected while the map, services, SLAM/TF, motion permission or
arm stow are not healthy.

During a task, those conditions are recoverable faults: a stale SLAM update or
TF, withdrawn motion permission, an unfolded arm, a stale mapping-mode reading,
or a ROS peer that answers late. The task cancels its Nav2 goal so the robot stops, reports a
`paused: <reason>` feedback stage, and resumes target selection once every input
is healthy again. Only cancellation, the duration limit, the storage/session
quota, a mapping-mode change, the region boundary, another Nav2 client or an
unconfirmed navigation stop end the task. A Nav2 goal that exceeds the
navigation deadline marks that target unreachable. Hardware operation remains within the existing attended physical
acceptance; room-scale coverage has not been physically qualified by the tests.

The `ExploreKnown` Nav2 planner and the tracked `explore_nav_to_pose.xml` behavior
tree prohibit planning through unknown space. Ordinary `GridBased` navigation
retains its existing behavior. Failed approaches are excluded for the task;
exhausting them produces an incomplete result. A canceled/replaced Nav2 goal
ends exploration instead of fighting another navigation client. Cancel an
exploration before sending an unrelated navigation task.

The physical Nav2 pose progress checker counts either 0.10 m of translation or
0.05 rad of turning within 15 seconds. This permits slow turns toward an
observation without mistaking them for a stationary robot.

SLAM freshness uses RTAB-Map's canonical `/info` topic, matching normal bringup.
RTAB-Map starts with `--ros-args --log-level warn` to suppress routine ROS
informational logs while retaining warnings and errors.
The mode-switching services remain private to the node at `/rtabmap/set_mode_*`.
SLAM's camera-acquisition stamps have a separate tracked four-second age/liveness
budget to cover processing, transport and inter-update gaps (the latest observation
exceeded 2.5 seconds in a live timing sample). TF and motion-permission inputs retain
the one-second task budget; the independent production motion guards are unchanged.

To cancel the sole active exploration through standard ROS action services:

```bash
ros2 service call /robot/explore/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

The zero UUID/stamp selects all goals on this action, which accepts only one
at a time. Cancellation waits for the owned Nav2 goal to reach a terminal
status. If acknowledgment is delayed beyond the bounded cleanup deadline, the
result explicitly reports that the stop is unconfirmed, blocks further
exploration, and cancels a late acceptance when it arrives. Normal SIGINT and
SIGTERM keep DDS alive for that cleanup before shutting the server down.

An independent `exploration_owner_guard` watches each Explorer-generated Nav2
UUID with a one-second wall-time ownership lease. The Explorer obtains its
acknowledgment before sending navigation. If the Explorer dies, the guard cancels
that UUID and waits for a terminal Nav2 status before allowing another owner.
A restarted guard discovers prior ownership through Nav2's retained action status.
Ordinary Nav2 client UUIDs are unaffected. The mux, merged SLAM cloud, Explorer and
owner guard restart after an unexpected exit; permission and sensor leases still
apply during recovery.

`scripts/explore.sh` returns 0 only for a successful action result, 1 for an
aborted/canceled result, 2 for rejection, unavailable peers or an unconfirmed stop,
and 130 after Ctrl-C confirms cancellation. Discovery and cancellation are bounded;
the server owns the configured task duration. `/diagnostics`, status
`lekiwi/exploration`, exposes current readiness, stage and the last refusal reason.
Paused feedback updates when the blocking cause changes.

## Mapping lifecycle and quotas

The task verifies `/rtabmap/set_mode_mapping` before navigating. It preserves
`~/.ros/lekiwi_rtabmap.db`, without resetting, rotating or renaming the active
database. If the task began in localization, it restores localization when it
finishes, fails or is canceled; an existing mapping session remains mapping
unless it reaches a quota.

The always-running monitor reads RTAB-Map's `Mem/IncrementalMemory` and counts
the database plus `-wal`, `-shm` and `-journal` sidecars. Both mapping-startup
and later direct calls to `/rtabmap/set_mode_mapping` are covered. A transition
from localization starts a fresh wall-time session; a transient parameter RPC
failure does not renew its budget. At `rtabmap_mapping_max_seconds` or
`rtabmap_mapping_max_bytes`, the monitor calls `/rtabmap/set_mode_localization`;
an active exploration cancels navigation and returns incomplete. Switching
back to mapping cannot bypass a full storage quota. After a completed duration
freeze is confirmed, the next accepted task starts a fresh session without
inheriting the previous session's stop reason. The robot's normal safety
and torque policy stays unchanged.

The mapper and this quota monitor must remain running. This task runner does
not replace the existing independent motion guards or physical supervision.
The standalone `rtabmap-session-guard.py` remains a finite quota-report utility,
not the bringup monitor.

## MCP / Fiber

Use the existing rosbridge connector, with `/robot/explore` explicitly included
in its tracked `allowed_actions`. No raw velocity access is needed. An example
frame for `rosbridge.exchange` is:

```json
{
  "op": "send_action_goal",
  "id": "explore-42",
  "action": "/robot/explore",
  "action_type": "lekiwi_rmf/action/Explore",
  "feedback": true,
  "args": {"revisit_known": true, "max_duration_sec": 0.0, "max_radius_m": 0.0}
}
```

Pair it with the correlated cancel frame
`{"op":"cancel_action_goal","id":"explore-42","action":"/robot/explore"}`
and an exchange deadline longer than the requested task plus cleanup. A sent
frame alone is not task completion: inspect the action status and `complete`
result. The integration's external allow-list is not changed by this ROS package.

## Verification

The source includes map/geometry tests and isolated DDS action/service tests
with simulated Nav2 and RTAB-Map peers, including success, cancellation,
concurrent-goal rejection, permission/SLAM loss, direct mapping transitions,
duration/storage quotas, delayed navigation acceptance and orderly shutdown.
The real Jazzy BT navigator also loads and executes the tracked exploration
tree against a simulated planner/controller, including controller cancellation.
Run through CTest after building; domains 85 and 86 are separate from the live
robot. These tests exercise the task runner without commanding hardware.
