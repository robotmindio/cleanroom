# Robot task contract

`robot_tasks` adapts the existing Nav2, Explore and Search actions. It owns no motors,
perception pipeline or workflow engine. Fiber remains the workflow/policy owner;
ROS owns physical execution, collision checking and the local motion guards.

| Service | Request | Meaning of success |
| --- | --- | --- |
| `/robot/capabilities` (`std_srvs/srv/Trigger`) | empty | JSON capability catalog read |
| `/robot/tasks/submit` (`lekiwi_rmf/srv/SubmitTask`) | caller task ID, capability, JSON parameters | task durably admitted, **not completed** |
| `/robot/tasks/get` (`lekiwi_rmf/srv/GetTask`) | task ID | persisted state/result found |
| `/robot/tasks/cancel` (`lekiwi_rmf/srv/CancelTask`) | task ID; empty selects the managed active task | cancellation requested, **not a confirmed stop** |
| `/robot/tasks/v2/*` (`lekiwi_rmf/srv/TaskApi`) | schema 2 and a JSON request | see [negotiated task API](#negotiated-task-api-schema-2) |

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
`search` accepts a required `query` (a native label of the detector profile
published on `/robot/perception/profile`), an optional map-frame `region`
`{x, y, radius_m}`, and `max_duration_sec`, `max_travel_m` and
`target_coverage`; see [active search](active-search.md). It runs in the Explorer
on the same guarded Nav2 path, so it is unavailable wherever exploration is, and
`allow_search` in `config/exploration.yaml` keeps it off until the perception
operating point is qualified. A search `succeeded` only when it found the label
or reached its target coverage; budget, inaccessible space and missing
instruments are `failed` with the outcome and an honest coverage report.

States are `starting`, `running`, `paused`, `canceling`, `succeeded`, `canceled`,
`failed` and `unknown`. Only `succeeded`, `canceled` and `failed` are terminal.
Poll after cancellation until the terminal result is recorded. `succeeded`
requires ROS SUCCEEDED and a successful domain result, not just a received reply.
An aborted or incomplete exploration is `failed`.

Repeated submission of the same ID and identical parameters returns the
existing task; it never sends a second goal. Conflicting reuse is rejected.
Tasks whose resources overlap are serialized: admission is refused with the
holder while another task holds one of them. Observed native navigation, exploration and MoveGroup (arm)
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

## Negotiated task API (schema 2)

`/robot/capabilities` stays schema 1 and adds `task_api`: the supported schemas
`[1, 2]`, the `TaskApi` services, the body profile, the journal ID and the
current resource holders. Each capability declares its `resources`, SI `units`,
`frame`, `effects` (`reversible`, `compensable` or `irreversible`, with a
compensation description) and `requires_schema`. A `TaskApi` request with any
schema other than 2 is refused with `schema_incompatible`, never reinterpreted.
A capability whose effects are not reversible is refused through `SubmitTask`,
because schema 1 reports an implementation that ran to its end as `succeeded`.
Learned skills declare `effects` in `config/robot_skills.yaml`; an undeclared
skill is irreversible. The `skill` entry also lists the `implementation_kinds`
the runner executes (`lerobot_policy`), and each advertised skill carries its
`kind`, `cameras` and the `resources` it uses, derived from its registry entry:
arm joints give `arm`, base commands `base` and cameras `camera`. These are
additive schema 1 fields. Admission still holds the capability's `resources`
(the arm, and the base kept still) for every skill.

| Service | Request | Result |
| --- | --- | --- |
| `validate` | as `submit` | every admission check, without admitting |
| `submit` | `task_id`, `capability`, `parameters`, `owner`; optional `goal {id, revision}`, `body_profile`, `frame`, `units`, `resources`, `authority`, `compensates {task_id, generation}` | task view with its `generation` |
| `status` | `task_id`, optional `generation` | task view |
| `events` | `cursor`, optional `task_id`, `limit` (≤100), `generation` | journal events after the cursor |
| `cancel` | `task_id` (empty selects the active task), optional `generation`, `requester` | durable cancel receipt and stop state |
| `transfer` | `task_id`, `generation`, `owner`, `new_owner`, `checkpoint` | successor generation |
| `compensate` | `task_id`, `generation`, `owner`, `plan {intent, steps, note}` | recorded compensation plan |

Resources are `base`, `arm` and `perception_query`. Navigation and exploration
hold the base; search holds the base and the active perception query; a skill or reach
holds the arm and keeps the base still. A task holds them from admission until its
journal row is terminal, which requires native terminal evidence or the
reconciliation proof above; a timeout or an empty process never releases them. A
holder in `unknown` makes admission fail with `execution_uncertain`. The gateway
publishes holders on the latched `/robot/resources` topic; the detector refuses
`set_query` while a search holds the query and treats an unreadable state as held.

Admission and every handoff take a new, monotonic `generation`. A request that
names an older generation is refused with `stale_generation`. A cancel without a
generation is always accepted, so a stop is never fenced out. A `transfer` moves
the task to a new owner with a typed checkpoint (`summary`, `goal`,
`interpretation`, `references`, `plan`, `remote_commands`, `generations`,
`resources`, `progress`, `pending_verifications`; at most 16 KiB, no
credentials). The successor's generation starts unreconciled: it must read
`status` or `events` with that generation before it may compensate or submit a
compensating step. Handoff never dispatches anything, and the same `task_id`
returns the existing task to any client.

A repeated `task_id` with the same request digest (capability, parameters, goal
and compensation target) returns the existing task without a second command; a
different digest is `identity_conflict`. Refusal codes include
`invalid_request`, `capability_unavailable`, `resource_busy`,
`execution_uncertain`, `resource_mismatch`, `body_profile_mismatch`,
`map_unaligned`, `unit_mismatch`, `precondition_failed`, `stale_generation`,
`authority_denied`, `identity_conflict`, `irreversible_effect`, `not_found` and
`schema_incompatible`.

The task view keeps the legacy `state` and adds separate dimensions:
`execution.phase` (`admitted`, `running`, `paused`, `uncertain`,
`execution_finished`, `stopped`, `not_started`) with `outcome`; `cancel` (the
receipt); `stop` (`not_requested`, `stopping`, `uncertain`, `confirmed`);
`effect` evidence (a search's detected label, Nav2's reached pose; a skill has
none); and `goal`, which the robot never verifies (`verified: null`,
`verifier: recuriant`). It also carries owner, generation, resources, effects,
checkpoint, compensation, effective parameters, admission/dispatch/finish times
and the task's latest event cursor. After a gateway restart a dispatched task is
`uncertain` until this process observes it live or a native result arrives.

Events are journaled in the same transaction as the change they describe:
admission, dispatch, state changes (with released resources), cancel requests,
pause stops, resumes, handoffs, reconciliation, compensation and late replies
for superseded attempts, which are recorded but never applied. The newest 10,000
events are kept; `truncated` reports a cursor older than that or from another
journal (`journal_id`), and the client then reconciles from `status`.

Compensation is a recorded plan, never an automatic undo. It may be recorded only
after the task is terminal. `restore` is allowed only for reversible effects,
`repair` also for compensable ones; an irreversible effect can only be `isolate`d
or `record`ed (`irreversible_effect` otherwise). Compensating steps are ordinary
tasks submitted with `compensates`; their results update the plan's status, while
the original task keeps its own state and outcome, and the plan always reports
`counts_as_goal_success: false`.

`/robot/tasks/events` publishes state changes as `std_msgs/msg/String` JSON.
Events accelerate observation; the journal/status service is authoritative
because a disconnected stream can miss events.

## Geometric feasibility and reach

`motion_solver` (started with MoveIt) serves `/robot/motion/solve`
(`lekiwi_rmf/srv/TaskApi`, motion schema 1). It never moves anything. MoveIt owns
kinematics, collision checking and motion planning; the solver proposes joint values
and lets MoveIt judge them. `/robot/capabilities` advertises it as `motion`
(`available` while the service answers) beside a `reach` capability that executes a
solution.

| `op` | Answer |
| --- | --- |
| `solve` | `feasible` with `joints` for the arm group, the tool `pose`, and with `plan` (default true) a MoveIt `motion` (planning time, waypoints, duration); `infeasible` (`no_kinematic_solution`, `in_collision`, `no_motion_plan`, `no_verified_solution`); or `unknown` |
| `verify` | `holds`, `violated` or `uncertain` for the current tool pose from `/joint_states` fresher than 0.5 s; or `unknown` |

A request holds one to four `constraints` on `tool0` in `base_footprint` (MoveIt's
model frame), plus `max_time_sec` (at most 10):

- `position`: the tool within a `sphere` (`radius_m`) or `box` (`half_extents_m`),
  at most 0.5 m, around a `target` plus `offset_m`. The target is a TF `point`
  `{frame, xyz}` or an `observation` `{id, detection}`: a detection with a calibrated
  metric position, at most 10 s old, carried from its capture time to now through
  `odom` in TF. Its `uncertainty_m` stays visible: `verify` says `uncertain` within
  that distance of the boundary, and `solve` searches inside the region shrunk by it.
- `orientation`: one tool `axis` within `tolerance_rad` of a `direction`; rotation
  about that axis is free.

`solve` samples the arm's joint space once through MoveIt's `/compute_fk` and refines
the closest samples with bounded least squares (`scipy`). Every candidate is then
checked without the search's margin by `/compute_fk`, by `/check_state_validity`
against the current planning scene and, with `plan`, by `/plan_kinematic_path` from
the current state. The search is bounded (1500 kinematic evaluations by default) and
incomplete: `no_kinematic_solution` means none was found within the bound. Missing
inputs are `unknown`, never `infeasible`: `joint_state_stale`,
`observation_unavailable`, `position_unavailable` (no metric grounding),
`observation_stale`, `transform_unavailable`, `uncertainty_exceeds_tolerance`,
`search_timeout` (the request's `max_time_sec` ended the search early),
`service_unavailable`, `service_timeout`, `kinematics_failed`. An answer carries
`checked_at`, `valid_until` (5 s later), the resolved `targets` and its assumptions;
it describes that scene only, so a planner checks again right before it moves.

`reach` takes `joints` for exactly `arm_shoulder_pan`, `arm_shoulder_lift`,
`arm_elbow_flex`, `arm_wrist_flex` and `arm_wrist_roll` within the joint limits, and an
optional `solution_id`. The gateway sends MoveIt's `MoveGroup` action (`/move_action`)
a joint goal (0.01 rad tolerance) with `plan_only: false`: MoveIt plans again from
the current state, checks the current scene and executes through the same controller
and safety path as RViz. It holds the arm and keeps the base still, its effect is
reversible, and `succeeded` requires MoveIt's `SUCCESS` error code. The result keeps
MoveIt's code, planning time and final joints, not the trajectories. Success means
the motion finished, not that a constraint holds; check that with `verify`.
`allow_reach` is false by default, like `allow_skills`, until arm planning is
qualified on this body.

## Fiber and permissions

The Fiber rosbridge connector exposes `rosbridge.capabilities`,
`rosbridge.tasks.status`, `rosbridge.tasks.submit` and `rosbridge.tasks.cancel`.
Use these named operations for role grants instead of granting unrestricted raw
ROS frames. Read/status are read-only; cancellation is low-impact; starting a
physical task retains its high-impact metadata but the managed submission
operation is L1: authorized callers execute with an audit record and no human
approval wait. Role scopes, exact service allow-lists and local safety still apply.
The connector's exact service allow-list must include the four services above;
add the `/robot/tasks/v2/*` services for schema 2. Without them, or on a robot
that does not advertise schema 2, the connector uses schema 1.
Rosbridge itself retains its general ROS functionality for other clients.

Enable the existing tailnet binding with
`scripts/deploy-split.sh robot-1 --rosbridge-tailnet` (or the compute installer's
matching option). Never bind the unauthenticated bridge to a public
interface. Fiber's integration URL must match the compute host's tailnet IP;
its encrypted integration configuration is maintained through Fiber's API.

This contract exposes navigation, exploration, observation by base motion and, when enabled, arm reach to a solved target. It does not qualify household
manipulation, docking, unrestricted unattended operation or learned skills.


The operator robot belongs to Fiber's `default` tenant, matching the human
Fiber UI session. Its workflows are declared in the integrations plugin's
`routes.d/robot.yaml` and deployed through GitOps. For a separate DB tenant,
install the tracked single-rule `config/fiber_robot_tasks.yaml` with Fiber's
`append_workflow` operation; this does not install a default-tenant file route. Invoke `robot-task-submit` with no vars for default exploration. Optional
`task_id`, `capability` and `parameters_json` retain the legacy calling contract. Its governed execution admits the local task
automatically under L1. A tenant can explicitly require L2; that creates a durable
approval wait before admission. Approval notifications use the configured channel
(e.g. Matrix); declaring a channel does not provision its connection. A human
approver resolves the gate in Fiber's approvals UI or authenticated API. Fiber
checks approver identity and binds the single-use approval to that execution and
action. With `on_timeout: deny`, an unanswered request never admits the task.
Direct action calls can return a policy gate without creating the workflow wait;
use the named workflow for physical submissions. Poll `rosbridge.tasks.status` by the same task ID after
admission; workflow completion means admission, not physical completion.

## Agent-facing defaults

A user can ask an agent to “have the robot explore the house.” The agent reads
Fiber's action descriptions and `rosbridge.capabilities`, then submits with
`rosbridge.tasks.submit` (no arguments required). Fiber selects the authenticated
tenant's configured robot, defaults capability to `explore`, derives an omitted
task ID from its existing idempotency context and returns it for observation.
A structured `parameters` object is preferred to JSON strings in direct calls.
The tracked workflow remains the path for durable approval when policy gates.

No exploration parameters means `revisit_known=false`, empty `resume_task_id`,
and zero duration/radius overrides. Zero selects the Explorer's tracked limits,
currently 900 active seconds and 10 m in `config/exploration.yaml`; a positive
override may only reduce those limits. An agent uses `revisit_known=true` when
the user's intent includes surveying already mapped space. Navigation still
requires map-frame `x` and `y`; omitted `yaw` is zero radians.
When live Explorer defaults are known, an override above either maximum is
rejected before durable admission. A resume request cannot supply positive
duration/radius overrides; the saved task owns those bounds.

The Explorer publishes its actual configured limits on the local, latched
`/robot/explore/defaults` topic. The task gateway exposes them as
`effective_defaults` in the capability catalog alongside `defaults`,
`required_parameters`, descriptions, examples and terminal states. It snapshots
resolved parameters at admission into `result_json._task.effective_parameters`,
retained through restarts and native terminal results. New unsent exploration
tasks dispatch those captured limits, so a later default increase cannot expand
the admitted task; the Explorer still enforces its current maximum limits. `_task.parameters` records
the requested parameters. Effective exploration values are explicitly `null`
when the Explorer has not advertised them or a saved native task is resumed;
its original saved bounds remain owned by the Explorer. Older task rows remain
queryable without this additive metadata.

Retries use the returned task ID or the same Fiber idempotency key and identical
arguments. A new task uses a new key or explicit task ID. Omission uses Fiber's
normal execution/source identity or direct-call time-bucket deduplication; it
does not identify a retry across unrelated execution contexts. Starting remains
L1, so an admitted task can begin physical motion immediately.
