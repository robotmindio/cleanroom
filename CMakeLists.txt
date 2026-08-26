cmake_minimum_required(VERSION 3.16)
project(lekiwi_rmf)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS) accepts a single RENAME value, so each renamed script gets its own call.
install(PROGRAMS lekiwi_rmf/driver.py DESTINATION lib/${PROJECT_NAME} RENAME lekiwi_driver)
install(PROGRAMS lekiwi_rmf/camera_relay.py DESTINATION lib/${PROJECT_NAME} RENAME camera_relay)
install(PROGRAMS lekiwi_rmf/cmd_vel_mux.py DESTINATION lib/${PROJECT_NAME} RENAME cmd_vel_mux)
install(PROGRAMS lekiwi_rmf/readiness_gate.py DESTINATION lib/${PROJECT_NAME} RENAME readiness_gate)
install(PROGRAMS scripts/arm_calibration.py scripts/arm_jog.py scripts/odom_scale.py scripts/teleop.py scripts/free_space.py scripts/rtabmap-db-maintenance.py scripts/sim-renderer-check.py scripts/sim-scan-check.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/camera-supervisor.sh DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES scripts/setup-pi.bash DESTINATION share/${PROJECT_NAME}/scripts)
# MoveItConfigsBuilder reads this from the package share root to infer the
# description files silently; see launch/moveit.launch.py.
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch maps urdf worlds DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.pyo" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Test the package under construction, not a similarly named package sourced
  # from another overlay workspace.
  set(test_pythonpath "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")
  # Never let a test discover a running robot or a parallel test. These are
  # distinct valid DDS domains, intentionally away from the live default (0).
  ament_add_pytest_test(test_odometry test/test_odometry.py ENV "${test_pythonpath};ROS_DOMAIN_ID=180")
  ament_add_pytest_test(test_odom_scale test/test_odom_scale.py ENV "${test_pythonpath};ROS_DOMAIN_ID=181")
  ament_add_pytest_test(test_free_space test/test_free_space.py ENV "${test_pythonpath};ROS_DOMAIN_ID=182")
  ament_add_pytest_test(test_camera_relay test/test_camera_relay.py ENV "${test_pythonpath};ROS_DOMAIN_ID=183")
  ament_add_pytest_test(test_arm_trajectory test/test_arm_trajectory.py ENV "${test_pythonpath};ROS_DOMAIN_ID=184")
  ament_add_pytest_test(test_collision_model test/test_collision_model.py ENV "${test_pythonpath};ROS_DOMAIN_ID=185")
  ament_add_pytest_test(test_driver_link test/test_driver_link.py ENV "${test_pythonpath};ROS_DOMAIN_ID=186")
  ament_add_pytest_test(test_cmd_vel_mux test/test_cmd_vel_mux.py ENV "${test_pythonpath};ROS_DOMAIN_ID=187")
  ament_add_pytest_test(test_teleop test/test_teleop.py ENV "${test_pythonpath};ROS_DOMAIN_ID=188")
  ament_add_pytest_test(test_readiness_gate test/test_readiness_gate.py ENV "${test_pythonpath};ROS_DOMAIN_ID=189")
  ament_add_pytest_test(test_sim_host_tools test/test_sim_host_tools.py ENV "${test_pythonpath};ROS_DOMAIN_ID=190")
  ament_add_pytest_test(test_rtabmap_db_maintenance test/test_rtabmap_db_maintenance.py ENV "${test_pythonpath};ROS_DOMAIN_ID=191")
  ament_add_pytest_test(test_lidar_detection test/test_lidar_detection.py ENV "${test_pythonpath};ROS_DOMAIN_ID=193")
endif()

ament_package()
