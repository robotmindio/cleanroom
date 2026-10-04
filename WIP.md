# Physical acceptance in progress

44 mm floor AprilTag calibration is recorded. Nominal ground-qualified counts: forward 31, reverse 26, left 31, right 32, clockwise 30, counterclockwise 32. Six reverse windows with missing raw baselines were excluded. Worst nominal sweep is 26.632 mm. Declared measurement uncertainty is 20 mm; total clearance remains 50 mm.

Live lidar loss at 0.04 m/s exceeded the travel budget (38.088 mm + 20 mm). Limits are now 0.03 m/s and 0.06 rad/s pending repetitions. Single-frame terminal yaw noise falsely extended stop time; terminal pose now uses eight observations, with regression coverage. Runtime still deployed at 704b83a until final deployment.

Next: finish five additional reverse stops, repeat linear/angular fault stops and whole-ROS crash, qualify raw camera evidence, grant acceptance only if all budgets pass, deploy and run a finite production check. Preserve named poses and production map. No persistent test clients.
