# Simulation qualification evidence

After building the repository on the intended simulation host, run this from
the source checkout (the runner deliberately resolves its repository root from
its own path) to collect an auditable evidence bundle:

```bash
scripts/sim-qualification.py \
  --build-dir build/lekiwi_rmf \
  --output-dir "$HOME/.ros/lekiwi/qualification-evidence/final-revision"
```

The command is intentionally strict: it requires the selected CMake build and
installed package to come from this checkout, the selected test Python to
import pyzmq, every expected CTest (including the three pyzmq-dependent tests),
ShellCheck, static source/default-deny checks, all CTests, and the headless EGL
renderer probe. It writes command output, the exact revision/dirty state, and
`summary.json` to the output directory, including when a check fails.
The evidence directory must be outside the source checkout; this preserves the
revision's clean provenance rather than making the runner's own output a dirty
change.

The runner directly invokes `scripts/moveit-shutdown-probe.py` and keeps its
revision-bound JSON in the new evidence directory. It puts the selected
build's install prefix ahead of any older sourced overlay and requires schema
1, that exact package prefix, the current Git SHA, package-version metadata,
`clean_shutdown: true`, and `move_group_exit_code: 0`. The ordinary E2E CTest
deliberately does not assert `move_group`'s shutdown exit code, so a passing
test alone cannot qualify this behavior.

It never reports qualification. On a separately started managed simulation,
collect bounded ROS evidence without altering the stack:

```bash
scripts/sim-qualification.py \
  --build-dir build/lekiwi_rmf \
  --output-dir "$HOME/.ros/lekiwi/qualification-evidence/runtime-final" \
  --collect-runtime
```

The generated `runtime-checklist.md` identifies the required administrator
observations: mux/stale-stop behaviour, topology, MoveIt/RViz independently,
and native heartbeat fault injection. It also retains tails from the managed
simulation and newest ROS launch logs where present. Follow the full
acceptance criteria in [DEFERRED.md](DEFERRED.md).
