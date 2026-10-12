# Robot task contract

`robot_tasks` adapts the existing Nav2 and Explore actions. It owns no motors,
perception pipeline or workflow engine. Fiber remains the workflow/policy owner;
ROS owns physical execution, collision checking and the local motion guards.

| Service | Request | Meaning of success |
| --- | --- | --- |
| `/robot/capabilities` (`std_srvs/srv/Trigger`) | empty | JSON capability catalog read |
| `/robot/tasks/submit` (`lekiwi_rmf/srv/SubmitTask`) | caller task ID, capability, JSON parameters | task durably admitted, **not completed** |
| `/robot/tasks/get` (`lekiwi_rmf/srv/GetTask`) | task ID | persisted state/result found |
| `/robot/tasks/cancel` (`lekiwi_rmf/srv/CancelTask`) | task ID; empty selects the managed active task | cancellation requested, **not a confirmed stop** |

`navigate` accepts finite map-frame `x`, `y` and optional `yaw` (radians), and
uses the normal Nav2 controller. It has a 180-second active-time budget. A collision STOP, stale collision state or
denied/stale base permission pauses the managed task without spending that budget.
The gateway cancels its current Nav2 attempt and waits for a terminal native
result before resending the same destination with a new owned goal ID. Fresh
safety inputs and one second of collision settling are required to resume.
An unconfirmed stop expires the ownership lease and blocks resubmission.
Direct legacy Nav2 clients retain their existing behavior.
`explore` accepts `revisit_known`, `max_duration_sec`, `max_radius_m` and
`resume_task_id`; the Explorer enforces its tracked bounds, quotas and recovery
policy. Exploration's recoverable pauses do not finish the task or spend its
active-time budget. Fixed-map/RMF profiles disable exploration; RMF profiles
also disable managed navigation so the fleet remains the navigation owner.

States are `starting`, `running`, `paused`, `canceling`, `succeeded`, `canceled`,
`failed` and `unknown`. Only `succeeded`, `canceled` and `failed` are terminal.
Poll after cancellation until the terminal result is recorded. `succeeded`
requires ROS SUCCEEDED and a successful domain result, not just a received reply.
An aborted or incomplete exploration is `failed`.

Repeated submission of the same ID and identical parameters returns the
existing task; it never sends a second goal. Conflicting reuse is rejected.
Only one managed task can be active, and observed native navigation/exploration
goals prevent admission and defer pending dispatch. External ROS clients still own their goals; cancellation
through this API does not cancel their unrelated actions.

The private SQLite journal defaults to `~/.ros/lekiwi_tasks.sqlite3`. Other ROS
domains and simulated-time deployments use distinct journals. A process lock
prevents competing gateways from owning one journal. A reconnect can query
historical results. `task_journal.py` owns persistence and schema migration;
`task_gateway.py` owns ROS admission, transport and observation. A dispatch
marker commits before sending an action. After restart, proven-unsent tasks
can be sent once or canceled without sending; attempted deliveries are observed
without resubmission. Existing version-1 rows migrate conservatively as attempted
deliveries; version-2 rows preserve their dispatch marker. Schema 4 persists
navigation budget, pause intent, confirmed stop and the dispatch server instance.
A restart during a pending pause restores that intent and still waits for native
terminal proof before resuming. Terminal results cannot be overwritten by late callbacks, and a
repeated user cancellation preserves an earlier navigation deadline failure. Exploration can continue in its local server; navigation's
independent owner guard cancels a lost lease, which a restarted gateway never
revives. If ROS has forgotten the goal/result, state stays `unknown` unless the gateway
can prove that the original action server instance was replaced. Reconciliation
requires the unique replacement server to return UNKNOWN for both result and
cancellation, an expired lease grace period, no observed active ROS goals and
a fresh ready owner guard. Nav2 controller/recovery action status is also
observed: a forgotten parent cannot release admission while a child executes.
The owner guard reconciles its forgotten parent only after the same replacement
server evidence and child termination. It then records `failed` with `outcome_known=false`
and `termination_confirmed=true`; it never replays that task or claims success.
Same-instance uncertainty and legacy rows without a recorded server instance
remain blocked until native terminal evidence becomes available. A canceled
request alone is never proof that motion stopped.

`/robot/tasks/events` publishes state changes as `std_msgs/msg/String` JSON.
Events accelerate observation; the journal/status service is authoritative
because a disconnected stream can miss events.

## Fiber and permissions

The Fiber rosbridge connector exposes `rosbridge.capabilities`,
`rosbridge.tasks.status`, `rosbridge.tasks.submit` and `rosbridge.tasks.cancel`.
Use these named operations for role grants instead of granting unrestricted raw
ROS frames. Read/status are read-only; cancellation is low-impact; starting a
physical task retains Fiber's high-impact policy classification. One approval
covers the admitted task, including its local exploration pauses/recovery.
The connector's exact service allow-list must include the four services above.
Rosbridge itself retains its general ROS functionality for other clients.

Enable the existing tailnet binding with
`scripts/deploy-split.sh robot-1 --rosbridge-tailnet` (or the compute installer's
matching option). Never bind the unauthenticated bridge to a public
interface. Fiber's integration URL must match the compute host's tailnet IP;
its encrypted integration configuration is maintained through Fiber's API.

This contract exposes navigation/exploration only. It does not qualify household
manipulation, docking, unrestricted unattended operation or learned skills.
