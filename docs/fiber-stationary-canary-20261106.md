# Stationary wrist cup canary

`config/fiber_perception_wrist_canary.json` and
`config/fiber_perception_front_canary.json` bind finite attended pilots to their
tracked cameras and declared remote proposal profiles. The wrist pilot uses the
tracked near_field profile (480-pixel shortest edge) to leave verification time
inside the unchanged2s capture-age bound; the frontal pilot uses standard. It admits only cup,
keeps score0.50, evidence age2s and confirmation window2s, has no search views,
allows 15 seconds per stationary goal, and is disabled at rest. It adds no metric or manipulation qualification.

Register this body through the owning authenticated API at
`PUT /robot/profiles/wrist-cup-canary-20261106` (or the corresponding front name). The operator authentication
procedure is `scripts/cikut-ops-jwt.sh` in the RobotMind platform repository;
retain the explicit RobotMind tenant header and never print the bearer token.
A local expired FIBER_API_KEY is not evidence that this documented administrative
path is unavailable. Use administrator authority for profile configuration;
normal goal admission and verification keep their existing policy gates.

During a finite pilot, run the installed `scripts/object-perception.sh
cameras:=wrist`, temporarily enable only this named profile, request stationary
image-only goals, and restore its disabled body on every exit. Cancel only goals
owned by this pilot and close its own perception process. Never modify a run's
sensor result, admission time or completion state. Record API responses,
fresh distinct observation IDs and separately retrieved evidence hashes. A
successful scene pilot does not qualify the independent recognition corpus.

Apply the tracked near_field profile to the primary recognizer before admitting
the wrist pilot, using /robot/perception/apply_profile with optimistic revision
checking. The profile hash in every accepted observation must match this body.
The platform caps stationary admission at15s independently of max_duration_sec.


The optional `perception_mode:=local_yolox` selects the existing pinned OpenCV
YOLOX recognizer, with the same 2 Hz inference and 2 s capture-age limit.
`config/fiber_perception_wrist_yolox_canary.json` binds a disabled, stationary,
image-only cup pilot to that operating point. Run it with
`scripts/object-perception.sh perception_mode:=local_yolox cameras:=wrist` and
register the disabled body at `/robot/profiles/wrist-yolox-canary-20261107`.
Use the same enable/owned-goal/disable cleanup protocol above; no open-vocabulary
query or near_field application is needed for this closed-vocabulary model.
A source view that works for one model does not qualify another model or the
independent scene corpus. Preserve the actual scene and every failed trial.
