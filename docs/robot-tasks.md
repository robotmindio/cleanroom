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

`navigate` accepts finite map-frame `x`, `y` and optional `yaw` (radians), or one
[`target`](#navigation-targets) instead of them, and
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
`allow_search` in `config/exploration.yaml` enables attended search with native
RGB-D; unsupported sensor and fleet topologies keep it unavailable. Physical
qualification remains open. A search `succeeded` only when it found the label
or reached its target coverage; budget, inaccessible space and missing
instruments are `failed` with the outcome and an honest coverage report.
`skill`, `reach` and `gripper` command the arm and are off unless `allow_skills`,
`allow_reach` or `allow_gripper` is true in `config/robot_tasks.yaml`, the only
switch for them. All are false; turning one on for the physical robot is an
operator decision that follows the arm trials in `DEFERRED.md`.

## Navigation targets

`navigate` takes either `x`, `y` and optional `yaw`, or a `target` object holding
exactly one of the shapes below. Language, named places and which object is meant
belong to the orchestrator; the robot only turns a geometric reference, or an
entry of its own [object map](object-map.md), into a pose.

| `target` | Meaning | Resolved by |
| --- | --- | --- |
| `{pose: {x, y, yaw}}` | a map-frame pose, the same as `x`, `y`, `yaw` | nothing |
| `{anchored: {map_pose: {x, y, yaw}, site_ref, map_ref, revision}}` | a pose remembered under an earlier map revision | `/robot/spatial/query`: `current`, then `project` into the current revision |
| `{point: {x, y}, standoff_m}` | stand `standoff_m` (0.3 to 3 m, default 0.6) from a map point and face it | the latched `/map` occupancy grid and the robot's pose in `map` |
| `{object: {object_id, standoff_m}}` | stand `standoff_m` (the same range and default) from an object-map entry's estimate and face it | `/robot/objects/query` for the entry, then the point resolution at its current-revision `x`, `y` |

The target is resolved once, at `validate` and at `submit`, after the base is known
to be free. The task's `effective_parameters` hold the resolved `x`, `y`, `yaw`
and `resolved` (the target kind and how it was found: the revision, method and
`shift_m` of a projection, or the point, standoff and robot position of an
approach, with the entry it came from for an object); `parameters` keep the request. That pose is what is sent to Nav2 and
resent after a pause, through the same owner guard, limits and budgets as a raw
pose. The request digest covers the request, so a retry with the same `task_id`
returns the admitted task and its pose.

An anchored pose follows the map anchor nearest to it, like a stored observation
([spatial memory](spatial-memory.md)). It is refused with `site_mismatch` for
another site, `revision_unknown` when its revision is unknown or pruned and
`map_unaligned` when its map has no alignment with the map the robot is in;
`precondition_failed` covers a spatial memory that is not answering
(`wait_spatial_memory`), no current revision (`needs_map`) and any other
projection error. Each carries the store's own `spatial_code`.

A point target is an approach pose chosen by `lekiwi_rmf/navigation_targets.py`:
poses on the circle of radius `standoff_m` around the point, each facing it,
tried nearest the robot first. A pose is admitted only where the explorer's
tracked `clearance_m` holds on known free cells, the whole oriented Nav2 footprint
lies on known free cells (`free_threshold` of `config/exploration.yaml`) and the
straight line to the point crosses only free cells before the occupied or unknown
cells the point itself lies in, so a wall in between hides it. Without such a pose
the request is `precondition_failed`: `needs_map` when part of the circle is
unmapped, `change_standoff` when all of it is mapped and blocked,
`needs_localization` without a `map` to `base_footprint` transform younger than
the explorer's `data_timeout_sec`. Nav2 still plans and validates the route.

An object target is the point target of an entry's position in the current map
revision. The entry is what the robot last observed, not a verified place:
`resolved` repeats its `object_id`, `label`, `state`, `method`, `sigma_m`,
`qualified`, `last_seen_unix_sec`, `map_ref`, `revision` and the map's
`verification` (`last_observed`), so the caller knows it is navigating to an
estimate, unqualified for every deployed method. A `present` or `unconfirmed`
entry is accepted. The request is refused, with `field` `target.object` unless
stated:

| Refusal | `next_condition` | When |
| --- | --- | --- |
| `precondition_failed` | `wait_object_map` (retryable) | the object map is not answering |
| `not_found` | `query_objects` | no entry has this `object_id` (`target.object.object_id`) |
| `precondition_failed` | `run_search` (retryable) | the entry is `absent_at_last_place` (`object_state`), or is only a direction without a point (`method`) |
| `site_mismatch`, `revision_unknown`, `map_unaligned` or `precondition_failed` | `refresh_reference` | the entry cannot be placed in the current map (`spatial_code` is the map's `position_unavailable_reason`), or lives in a map without alignment to the one the robot is in |
| `precondition_failed` | `needs_localization` (retryable) | the robot is not localized in a known map |
| `precondition_failed` | `change_standoff` (`target.object.standoff_m`), or `run_search` (retryable) when no permitted standoff is wide enough | `sigma_m` exceeds `object_target_max_sigma_ratio` × `standoff_m`; `sigma_m` and `max_sigma_m` are stated |
| `precondition_failed` | `fix_deployment` | the answer is not `lekiwi.object_map/1` or is malformed |

`object_target_max_sigma_ratio` in `config/robot_tasks.yaml` is 0.5: an object one
`sigma_m` from its estimate is then still at least half the standoff from the
base and within 30° of its heading. The refusals of a point target apply
afterwards with the fields `target.object` and `target.object.standoff_m`.
`run_search` means a `search` for the label must see the object again first.

The `navigate` manifest entry lists which kinds can resolve now under `targets`;
`object` needs the object map's service as well as what a point needs.

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
The original server instance gives the same proof for an exploration, search,
skill or reach task once the gateway that dispatched it has restarted: that
server returns UNKNOWN for both result and cancellation, the sender is gone, so
the goal ended or never arrived. The same grace period, no observed active ROS
goals and a ready owner guard are required, and the record is the same `failed`
with `outcome_known=false`. A goal the running gateway sent itself, any
navigation task on its original server and legacy rows without a recorded
server instance remain blocked until native terminal evidence becomes
available. A canceled request alone is never proof that motion stopped.

## Negotiated task API (schema 2)

`/robot/capabilities` stays schema 1 and adds `task_api`: the supported schemas
`[1, 2]`, the `TaskApi` services, the body profile, the journal ID and the
current resource holders. Each capability declares its `resources`, SI `units`,
`frame`, `effects` (`reversible`, `compensable` or `irreversible`, with a
compensation description), `requires_schema` and its
[`schema`](#self-describing-capabilities). A `TaskApi` request with any
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

Beside the capabilities the manifest states what the gateway can observe of the
robot's memory, each with `available` (its service is discovered), `service`,
`type` and `schema`:

- `observations.grounding` is `per_observation`: each observation record states
  its own `grounding` (`image_only` or `camera_position`) and `position`.
- `objects` is the [object map](object-map.md) on `/robot/objects/query`
  (`lekiwi.object_map/1`): `estimates: local_landmark`, `verification:
  last_observed` (never currently verified), `qualified: per_object` (false
  unless a position was measured under a validated calibration) and
  `navigate_target: object`.
- `spatial` is `/robot/spatial/query` (`robotmind.spacetime/1`); `pose` holds
  the request that answers where the robot is in the current revision.

## Self-describing capabilities

`task_gateway.py` holds one descriptor per capability (`CAPABILITIES`). Request
validation, the manifest entry and execution read the same descriptor, so the
manifest states exactly what is enforced. A client needs no prior knowledge of a
capability: its entry carries

- `schema.properties`: per parameter `type` (`number`, `integer`, `boolean`,
  `string`, `object`), `unit`, `min`, `max`, `exclusive_min`, `default`,
  `required`, `enum`, `pattern` (matches the whole string), `min_length` and
  `max_length` (of the text with its whitespace collapsed) and, for objects,
  nested `properties`. `max_ahead_sec` bounds a Unix time relative to now. Only
  the keys that apply are present, and unknown parameters are refused.
- `schema.alternatives`: groups of parameters of which exactly one group is
  given, each `{required, optional}`; `schema.excludes`: parameters that cannot
  be combined with a non-default value of another.
- `schema.resources`, `schema.preconditions` (`capability_enabled` with its
  `switch`, `action_server_ready` with its `action`, each with `satisfied`;
  `available` is their conjunction) and `schema.reason_codes`, the stable reasons
  a task of this capability can end with.

Limits tracked by this deployment are part of the schema: the Explorer's
exploration and search maxima and the claw's calibrated range appear as `max` and
`min` once known. `parameters`, `required_parameters`, `defaults` and `units` are
views of the same descriptor.

A refusal answers with `ok: false`, the `error` code, a readable `detail` and a
`response_json` that always holds `retryable` (whether the unchanged request can
succeed later) and `next_condition` (what must hold first), plus `field` (the
dotted parameter path, such as `target.point.y`) when one parameter is at fault.
The manifest's `refusals` table gives each code's default pair; a refusal may
state a more specific one. Conditions in use: `fix_request`,
`use_supported_schema`, `use_new_task_id`, `enable_capability`,
`wait_capability_ready`, `acquire_skill`, `wait_resource_free`, `reconcile_task`,
`wait_task_terminal`, `wait_odometry`, `wait_spatial_memory`, `wait_object_map`,
`query_objects`, `run_search`, `needs_map`,
`needs_localization`, `change_standoff`, `refresh_reference`, `replan_for_body`,
`read_status`, `use_owner`, `isolate_or_record` and `fix_deployment`.

Every state that needs an explanation carries a stable `reason_code` beside the
readable `detail`: in the task view, in `result_json` and in the journal's state
events. A state without one clears it. The manifest lists them under
`reason_codes`:

| Kind | Codes |
| --- | --- |
| `terminal` | `succeeded`, `canceled`, `canceled_before_dispatch`, `goal_rejected`, `capability_disabled`, `executor_error` (a nonzero native error code), `executor_aborted`, `incomplete`, `outcome_unknown` (termination proven, outcome not), `budget_exhausted` (navigation deadline, active time or travel bound), and a search's own `found`, `coverage_reached`, `inaccessible`, `budget_exhausted`, `incapable`, `unmapped` |
| `paused` | `collision_stop`, `collision_settling`, `base_motion_denied`, `safety_input_stale`, `safety_inputs_unavailable`, `awaiting_navigation_stop`, `navigation_stopped`, `executor_paused` (an exploration or search pausing itself) |
| `unknown` | `result_unconfirmed`, `acceptance_unconfirmed`, `arm_stop_unconfirmed`, `pause_stop_unconfirmed` |

`skill` takes `skill_id`, an optional `parameters` object and optional `version`
and `artifact_sha256` pins. `validate` and `submit` resolve the registry entry
before anything is held: an unregistered or experimental skill, or a pin that
differs from the registered entry, is refused with `competence_insufficient`
(the reply carries the registered `version` and `artifact_sha256` for a pin
mismatch); parameters outside the entry's declared names, types and ranges are
`invalid_request`. The gateway and the skill runner read the same registry:
`config/robot_skills.yaml`, or the `registry_file` parameter when both nodes set
it. Experimental skills run only through the acquisition protocol, which calls
the runner directly.

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
hold the base; search holds the base and the active perception query; a skill, reach
or gripper task holds the arm and keeps the base still. A task holds them from admission until its
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
`invalid_request`, `capability_unavailable`, `competence_insufficient`, `resource_busy`,
`execution_uncertain`, `resource_mismatch`, `body_profile_mismatch`,
`map_unaligned`, `site_mismatch`, `revision_unknown`, `unit_mismatch`,
`precondition_failed`, `stale_generation`, `authority_denied`, `identity_conflict`,
`irreversible_effect`, `not_found` and `schema_incompatible`.

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
While a task runs, `detail` is the executor's own feedback: the `stage` of an
exploration, search or skill, MoveIt's `state` for a reach or gripper task, and
`navigating` for Nav2. A goal the action server rejected is `failed` with `goal_rejected: true`
in its result; it never executed, so `compensation_required` stays false.
A skill's `result_json` carries `arm_stopped`. When it is `false` (or the report
is unreadable) the arm controller never confirmed the last chunk terminal, so
the task is `unknown` with `stop: uncertain` and keeps the arm and base, whatever
terminal status the runner returned; a missing key means stopped. Only the
replacement-server proof above releases it.

Events are journaled in the same transaction as the change they describe:
admission, dispatch, state changes (with released resources), cancel requests,
pause stops, resumes, handoffs, reconciliation, compensation and late replies
for superseded attempts, which are recorded but never applied. The newest 10,000
events are kept; `truncated` reports a cursor older than that or from another
journal (`journal_id`), and the client then reconciles from `status`.

Compensation is a recorded plan, never an automatic undo. It may be recorded only
after the task is terminal. `restore` is allowed only for reversible effects,
`repair` also for compensable ones; an irreversible effect can only be `isolate`d
or `record`ed (`irreversible_effect` otherwise). Recording the same plan again
returns it; a different plan for the same task is `identity_conflict`.
Compensating steps are ordinary tasks submitted with `compensates`. They run in
plan order, one at a time: a submission must name the plan's next step, with that
step's capability and exactly its parameters, or it is `precondition_failed` with
the expected `step` index. A step that fails or is canceled sets the plan to
`failed` and stays the next step, so a new task (a new `task_id`) runs it again;
the plan is `succeeded` once every step has. Each attempt is journaled as a
`compensation_started` event with its `task_id` and `step`, and `tasks` maps
every attempt to its state. The original task keeps its own state and outcome,
and the plan always reports `counts_as_goal_success: false`.

`/robot/tasks/events` publishes state changes as `std_msgs/msg/String` JSON.
Events accelerate observation; the journal/status service is authoritative
because a disconnected stream can miss events.

## Geometric feasibility, reach and gripper

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

`reach` takes either `joints` for exactly `arm_shoulder_pan`, `arm_shoulder_lift`,
`arm_elbow_flex`, `arm_wrist_flex` and `arm_wrist_roll` within the joint limits, or
`named`, an arm group state of `config/lekiwi.srdf` (`home`, `travel_stow`,
`supported_rest`), and nothing else. The gateway reads the named states from the
SRDF at startup and advertises them as `named_states`; a named reach reports the
joints it stands for in `effective_parameters`. The gateway sends MoveIt's
`MoveGroup` action (`/move_action`)
a joint goal (0.01 rad tolerance) with `plan_only: false`: MoveIt plans again from
the current state, checks the current scene and executes through the same controller
and safety path as RViz. It holds the arm and keeps the base still, its effect is
reversible, and `succeeded` requires MoveIt's `SUCCESS` error code. The result keeps
MoveIt's code, planning time and final joints, not the trajectories. Success means
the motion finished, not that a constraint holds; check that with `verify`.
`allow_reach` in `config/robot_tasks.yaml` is false, like `allow_skills`, until arm
planning is qualified on this body.

`gripper` takes either `state`, a gripper group state of the SRDF (`open`, `closed`,
`travel_stow`), or `position_rad` within the claw's joint limits. It is the same
`MoveGroup` request for the `gripper` planning group, which MoveIt executes through
the arm controller, so it passes the same planning, collision check, arm permission
and safety path as a reach, holds the same resources (the arm, and the base kept
still) and reports the same result. On the physical robot the gateway applies the
arm calibration MoveIt applies (`arm_calibration_file`): `closed` is jaw contact
(0 rad), `open` and the upper limit are the calibrated opening, and the manifest's
`named_states` and `position_rad` bounds show those values. Its effects are
`compensable`, not reversible: opening releases what the claw holds and closing may
grasp, so it needs schema 2 and a failed task asks for a compensation plan. Success
means the motion finished, not that anything is held. `allow_gripper` in
`config/robot_tasks.yaml` is false. Picking, placing and handing over are learned
skills, not capabilities of this API.

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
Apply `config/fiber_perception_integration.json` to the owning tenant's
integration, preserving its full existing configuration and credentials. Agent
Organization integrations and the historical `default` integration are separate;
updating one does not update the other. Verify `rosbridge.tasks.validate`,
`rosbridge.tasks.events`, odometry and spatial reads using the agents' own tenant.
Rosbridge itself retains its general ROS functionality for other clients.

Enable the existing tailnet binding with
`scripts/deploy-split.sh robot-1 --rosbridge-tailnet` (or the compute installer's
matching option). Never bind the unauthenticated bridge to a public
interface. Fiber's integration URL must match the compute host's tailnet IP;
its encrypted integration configuration is maintained through Fiber's API.

This contract exposes navigation, exploration, observation by base motion and, when enabled, arm reach to a solved or named target and claw motion. It does not qualify household
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
the user's intent includes surveying already mapped space. Navigation
requires map-frame `x` and `y` (omitted `yaw` is zero radians) or one
[target](#navigation-targets).
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
