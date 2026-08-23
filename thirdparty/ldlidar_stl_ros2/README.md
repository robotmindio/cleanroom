# Vendored: ldlidar_stl_ros2

`scripts/install.sh` clones [LDROBOT's official ROS 2 driver](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
into `$LEKIWI_WS/src/ldlidar_stl_ros2` at the revision pinned by `LIDLIDAR_STL_REV`
there, then applies the patch in this directory before building.

The patch carries two build fixes, both needed on Ubuntu 24.04 / GCC 13 and
both present in upstream's unmerged pull requests (#24/#25/#28):

- `#include <pthread.h>` in `log_module.cpp`, which GCC 13 stopped providing
  transitively;
- `add_compile_definitions(LINUX)` in `CMakeLists.txt`, because the SDK's
  Windows-only paths (`comutil.h` and friends) are guarded by `#ifndef LINUX`
  and nothing upstream defines that macro.

Drop this patch once a release supersedes it.
