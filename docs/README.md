# Documentation

- [Robot intelligence implementation report](robot-intelligence-report.md): delivered software, scoped completion policy and remaining qualification epics.

- [Product readiness and epics](product-readiness.md): capability status,
  object recognition, semantic world state, robot goals and planning, completed
  source changes, delivery sequence and measurable completion gates.
- [Robot intelligence implementation plan](robot-intelligence-implementation-plan.md):
  delivery phases for recognition, semantic state, goals, search and verification.
- [Object observations](object-observations.md): stationary recognition, standard
  ROS detections, persistent evidence services and held-out evaluation.
- [Skill acquisition](skill-acquisition.md): train, register, evaluate and promote
  learned skills; the trainer contract, cluster training, simulation scenes, the
  independent verifier and regression suites.
- [Perception training](perception-training.md): datasets of taught and corrected
  views, recognizer fine-tuning, registration, and promotion without regression.
- [Spatial memory](spatial-memory.md): map, revision, anchor and session
  identities, capture-time map poses and reprojection after SLAM optimization.
- [Robot task contract](robot-tasks.md): managed admission, status, cancellation,
  live defaults and Fiber workflow responsibilities.
- [Launch options](launch-options.md): `bringup.launch.py` arguments, arm
  planning and calibration, arming and recovery, production safety
  prerequisites, camera sources and odometry scale.
- [Real robot](real-robot.md): bring-up, `.env` configuration, boot services,
  coordinated split deployment, the `/scan` source and the calibration and
  mapping sequence.
- [ROS and MCP exploration](exploration.md): explicit navigation tasks, progress,
  cancellation, revisits and persistent mapping quotas.
- [Active search](active-search.md): per-query coverage, negative evidence,
  information-gain viewpoints, replanning and honest termination reports.
- [Operations](operations.md): per-stage latency budgets and age gates,
  health, learned-code isolation, robot data classes, backup and restore, and
  versioned contract compatibility.
- [Safety inputs and motor health](safety.md): implementation status, current
  diagnostic behavior, and the physical validation process.
- [Network security](../README.md#network-security): exposed ports and their
  protection.
- [Scripts](../scripts/README.md): every script, one line each.
- [Deferred work](../DEFERRED.md): open acceptance and deployment work with
  completion conditions and tracking issues.
