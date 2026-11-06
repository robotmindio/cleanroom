# Stationary wrist cup canary

`config/fiber_perception_wrist_canary.json` binds a finite attended pilot to the
tracked wrist camera and standard remote proposal profile. It admits only cup,
keeps score0.50, evidence age2s and confirmation window2s, has no search views,
and is disabled at rest. It adds no metric or manipulation qualification.

Register this body through the owning authenticated API at
`PUT /robot/profiles/wrist-cup-canary-20261106`. The operator authentication
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
