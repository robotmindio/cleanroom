# Exact navigation endpoints

- Completed: deployed clock and sub-cell fixes; six managed tasks passed; local
  perception produced 141 fresh outputs with no late or invalid ages.
- New finding: a task after the reviewed-release restart timed out and was
  canceled. General lattice paths may use the 20 cm search tolerance and return
  a discretized position/orientation. The robot was returned with guarded manual
  control to its pre-trial wheel pose, with no health faults.
- Implemented: zero general search tolerance and one collision-checked exact-goal
  connector shared by same-cell and general plans.
- Verified: six-case native regression fails before/passes after; all 12 actual
  planner route cases pass with the new plugin, including exact end poses.
- Next: review launch snapshots, checkpoint/push, deploy through the split script,
  repeat managed navigation and final health checks, update evidence, remove WIP.
