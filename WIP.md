# Accepted physical profile: delivery pending

Completed: 186 qualified nominal stops across all six directions and 15 moving fault stops. All accepted windows meet the 20 mm error and unchanged 50 mm total stopping budget. Final caps 0.03 m/s / 0.06 rad/s, dry concrete, 0 kg added payload, unchanged folded travel_stow, attended operator at independent power stop. The tracked acceptance validator returns true. Evidence hashes and per-case bounds are in docs/physical-acceptance-evidence-20261004.json.

Higher 0.04 m/s failed lidar-loss distance after proper floor calibration. Unqualified windows and historical failures are retained. Normal/rotation fault tests restore production; complete ROS crash killed and verified 31 owned processes. No production map or named pose was changed.

Next: native deployment, full CTest, finite production permission/MoveIt/Nav2 verification, then remove this file. Runtime before final deployment is 704b83a. No persistent test clients or motion publishers.
