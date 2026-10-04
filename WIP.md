# Physical acceptance: measurement qualification failed

Completed: 185 eligible nominal stops across all six directions, ten moving linear/angular fault cases, repeated sensor cases, whole-ROS crash, sixteen Nav2 goals and fresh-map spatial closure. All applicable functional fault flags are now true; counters, distances and declared limits are populated. Physical acceptance remains false.

Additional live checks: real bottle blocks an otherwise authorized command; temporary MoveIt wrist collision rejects an arm goal with zero joint change; authenticated motor-error and exact-duplicate telemetry injections withdraw permission at rest. The motor-error test found normal mode overriding arm health. That override is removed, deployed and retested; driver feedback-gap holding remains intact. No named pose or production map was changed.

Verified implementation: 3ecbba672ad9, 57/57 CTests including nine launch tests (163.52 s); 72 targeted measurement/torque checks. The additional motor-diagnostic and duplicate-telemetry checks passed while translating and rotating. Test host/injections are finite and restore production.

The bottle was removed and the ruler is now visible; its confirmed width is 7 mm. Those prerequisites are complete. A sensitivity check of the narrow, blurred floor reference cannot establish the declared 10 mm measurement error: one-pixel corner changes produce 18.27–46.70 mm floor estimates for a 35 mm marker-plane displacement. This is a calibration failure, not another unperformed fault test.

Update: the operator supplied a 44 mm black-square floor AprilTag (ID 33). Floor rectification and raised-marker correction are implemented and checked against synthetic physical geometry; three chassis headings are combined. A stationary capture bounds observed swept jitter at 4.53 mm. The new forward/reverse run stopped during calibration because real lidar/depth/telemetry data became stale; it contributed no stopping trials.

Network evidence: tailnet traffic uses a Singapore relay; direct LAN ICMP is fast, but TCP probes never arrive at the Pi. Pi origin fetch also fails DNS. JPEG quality is explicitly bounded to 70 (depth remains lossless), and deployment can transfer the already pushed revision by native Git bundle if the device origin fetch fails. These changes still need deployment and live verification.

Next: deploy, qualify floor-reference error and recorded camera geometries, complete eligible forward/reverse floor measurements and check fault-stop bounds before validated:true. Retain 0.04 m/s / 0.08 rad/s and 5 cm clearance. Production map and named poses are unchanged. No monitoring client is left running.
