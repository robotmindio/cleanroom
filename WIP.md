# Fiber robot tasks

Scope: close the Fiber integration and expose a small common task contract for navigation/exploration, reusing ROS actions and existing safety/ownership mechanisms. Preserve physical motion limits and arming policy.

Completed: durable ROS task gateway and native action ownership/recovery tests; no global rosbridge restrictions; repository/product audit; confirmed deployed rosbridge absent, Fiber stop example uses an unsubscribed topic, action transport success ignores ROS terminal status, and synchronous exchanges cannot cover paused exploration.

Next: finish scoped Fiber actions and role permission matrix; regenerate identity scopes; qualify both roles and deploy; verify end-to-end results and interruption behavior. Keep physical testing bounded.

Verified milestone: 106 isolated gateway/launch/release checks passed; 43 gateway/release checks passed after ownership/locking additions; Ruff and shell checks pass.

Verified baseline: PR #73 merged; reviewed revision d6f4592 deployed on compute and Pi, 73/16 release checks and CI green.

Pending: household endurance, near-field perception, semantic domestic manipulation and skill learning remain later milestones, not claimed complete by command integration.
