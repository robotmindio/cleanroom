# Vendored: ldlidar_stl_ros2

`scripts/install.sh` clones [LDROBOT's official ROS 2 driver](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
into `$LEKIWI_WS/src/ldlidar_stl_ros2` at the revision pinned by `LIDLIDAR_STL_REV`
there, then applies the patch in this directory before building.

The patch carries three fixes; the first two are needed on Ubuntu 24.04 /
GCC 13 and exist in upstream's unmerged pull requests (#24/#25/#28):

- `#include <pthread.h>` in `log_module.cpp`, which GCC 13 stopped providing
  transitively;
- `add_compile_definitions(LINUX)` in `CMakeLists.txt`, because the SDK's
  Windows-only paths (`comutil.h` and friends) are guarded by `#ifndef LINUX`
  and nothing upstream defines that macro.

The third fixes a shutdown crash we could reproduce on every Ctrl-C: the
serial port was closed *before* the RX thread joined, so the thread raced
`FD_SET()`/`pselect()` against a descriptor that had become `-1`, and glibc
aborted with "bit out of range 0 - FD_SETSIZE" (SIGABRT, core dump).
`Close()` now joins the reader first and both I/O paths snapshot the
descriptor. Until upstream merges an equivalent, re-apply on any revision
bump.

Drop this patch once a release supersedes all of it.
