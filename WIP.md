# Navigation and local perception

Completed: reproduced unsigned packet-time underflow in the real LD06 parser
with a backward clock adjustment. The native regression fails before the patch
and passes after it, including recovery after repeated clock values. Added scan
boundary validation and Nav2 process respawn. Selected local RT-DETR, no fallback,
with the existing single-front-camera deployment still disabled pending qualification.

Verified: native CMake/ctest clock regression; 108 initial focused tests; 145 broader
scan/launch/provider/profile checks and 13 regenerated launch snapshot checks;
latest scan-only tests, repository lint and diff checks. Fresh travel_stow checked.

Next: deploy this pushed revision through scripts/deploy-split.sh, measure local
front-camera latency, and run short navigation with recorded planning/control.

Pending: staged full release tests, hosted CI, deployment and live navigation.
No safety envelope, goal tolerance or freshness limit was relaxed. Images and raw
reports stay under ~/.ros/lekiwi/qualification/20261028-navigation-rtdetr.
