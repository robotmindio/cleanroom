# Runtime simplification

Completed: shared strict calibration reader/writer/CLI (including signed camera
pitch and host wheel scales), shared device ROS setup, optional Matplotlib tooling;
sim/wired/split profiles and static mapper/scan YAML; shared camera launches;
sole host-local arm executor; native qualification-script imports; driver and
launch configuration behavior tests replacing reconstructed/source-coupled checks.
Verified: isolated compute build, 138 focused behavior tests and 105 launch/service
checks, repository Python lint/compile, ShellCheck and diff checks.

Next: rebuild installation with final camera reuse, complete compute/device suites,
coverage and repository model/config checks, remove this file and submit the PR.
No live deployment or physical motion is part of this refactor.
