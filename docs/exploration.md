# Mapping and exploration from ROS or MCP

Normal visual-SLAM bringup starts `robot_explorer` with the rest of the repository
stack. It stays idle until an explicit goal arrives on `/robot/explore`, whose
type is `lekiwi_rmf/action/Explore`. AMCL has no exploration server; fixed-map
and RMF configurations refuse exploration. There is no startup movement or
change to arming, disarming, stow, payload acceptance or production speeds.

After building and sourcing `scripts/setup.bash`, request exploration within a
two-metre radius of the starting robot pose, for at most five minutes:

```bash
ros2 action send_goal /robot/explore lekiwi_rmf/action/Explore \
  '{revisit_known: true, max_duration_sec: 300.0, max_radius_m: 2.0}' --feedback
```

`revisit_known: false` searches frontiers only. `true` first searches reachable
frontiers, then visits spaced observation poses in already-known space, so an
existing map can be refreshed even when no unknown frontier remains. This is
coverage of accessible observation poses, not a claim to see through closed
doors, walls or occlusions. Feedback reports the stage, current pose, successful
and failed target counts, and known map area in the bounded region.

`max_duration_sec` and `max_radius_m` equal to zero select the tracked defaults
in `config/exploration.yaml` (900 seconds and 5 metres). Positive goal values
can only reduce these maxima. Map clearance is 0.33 m, enclosing the folded
robot's 0.326 m corner radius and matching the tracked Nav2/RMF envelope. It
rounds to a conservative 0.35 m disk on the 5 cm grid; the former 0.38 m setting
rounded to 0.40 m. Nav2 retains its exact rectangular footprint and its stop
polygon with 5 cm on every side. The exploration radius retains a 0.38 m
body/stopping margin (0.33 m plus 0.05 m); targets and preflight paths stay inside that margin, and the active
task cancels navigation if the live pose approaches it. It rejects malformed
maps, unavailable services, stale SLAM/TF, missing motion permission or an
unfolded arm. Hardware operation remains within the existing attended physical
acceptance; room-scale coverage has not been physically qualified by the tests.

The `ExploreKnown` Nav2 planner and the tracked `explore_nav_to_pose.xml` behavior
tree prohibit planning through unknown space. Ordinary `GridBased` navigation
retains its existing behavior. Failed approaches are excluded for the task;
exhausting them produces an incomplete result. A canceled/replaced Nav2 goal
ends exploration instead of fighting another navigation client. Cancel an
exploration before sending an unrelated navigation task.

SLAM freshness uses RTAB-Map's canonical `/info` topic, matching normal bringup.
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
back to mapping cannot bypass a full storage quota. The robot's normal safety
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
  "args": {"revisit_known": true, "max_duration_sec": 300.0, "max_radius_m": 2.0}
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
