# Runtime simplification

Completed: shared strict calibration reader/writer/CLI (including signed camera
pitch and host wheel scales), shared device ROS setup, optional Matplotlib tooling.
Verified: 20 calibration/odometry tests, focused Ruff, ShellCheck and diff checks.

Next: consolidate launch profiles and static RTAB-Map parameters; converge on
host-local arm execution; replace reconstructed driver/source-coupled service
tests with behavioral checks; replace chained runpy script loading with imports.
Pending: complete compute/device suites, repository static checks and PR CI.
No live deployment or physical motion is part of this refactor.
