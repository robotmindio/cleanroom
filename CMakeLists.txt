cmake_minimum_required(VERSION 3.16)
project(lekiwi_rmf)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(lekiwi_sim_native_failsafe SHARED src/sim_native_failsafe.cpp)
target_compile_features(lekiwi_sim_native_failsafe PUBLIC cxx_std_17)
target_link_libraries(lekiwi_sim_native_failsafe gz-sim8::gz-sim8)
install(TARGETS lekiwi_sim_native_failsafe LIBRARY DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS) accepts a single RENAME value, so each renamed script gets its own call.
install(PROGRAMS lekiwi_rmf/driver.py DESTINATION lib/${PROJECT_NAME} RENAME lekiwi_driver)
install(PROGRAMS lekiwi_rmf/astra_cloud_filter.py DESTINATION lib/${PROJECT_NAME} RENAME astra_cloud_filter)
install(PROGRAMS lekiwi_rmf/camera_relay.py DESTINATION lib/${PROJECT_NAME} RENAME camera_relay)
install(PROGRAMS lekiwi_rmf/cmd_vel_mux.py DESTINATION lib/${PROJECT_NAME} RENAME cmd_vel_mux)
install(PROGRAMS lekiwi_rmf/readiness_gate.py DESTINATION lib/${PROJECT_NAME} RENAME readiness_gate)
install(PROGRAMS lekiwi_rmf/safety_supervisor.py DESTINATION lib/${PROJECT_NAME} RENAME safety_supervisor)
install(PROGRAMS lekiwi_rmf/arm_workspace_monitor.py DESTINATION lib/${PROJECT_NAME} RENAME arm_workspace_monitor)
install(PROGRAMS lekiwi_rmf/rmf_owner_guard.py DESTINATION lib/${PROJECT_NAME} RENAME rmf_owner_guard)
install(PROGRAMS
  scripts/arm_calibration.py
  scripts/arm_jog.py
  scripts/odom_scale.py
  scripts/teleop.py
  scripts/free_space.py
  scripts/gripper-diagnose.py
  scripts/gripper-calibrate.py
  scripts/host-health-check.py
  scripts/rtabmap-db-maintenance.py
  scripts/rtabmap-session-guard.py
  scripts/torque-host.py
  scripts/validate-map-bundle.py
  scripts/sim-renderer-check.py
  scripts/sim-scan-check.py
  scripts/generate-zmq-keys.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/camera-supervisor.sh scripts/ros-astra.sh scripts/ros-lidar.sh scripts/install-lidar-service.sh
  DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES scripts/lib/service-install-common.sh scripts/lib/runtime-common.sh
  DESTINATION share/${PROJECT_NAME}/scripts/lib)
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
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  # Test the package under construction, not a similarly named package sourced
  # from another overlay workspace.
  set(test_pythonpath "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")
  set(test_ament_prefix_path
    "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
  set(test_launch_environment "${test_pythonpath};${test_ament_prefix_path}")
  # pyzmq is an intentional runtime dependency on the robot, but some minimal
  # ROS development hosts omit it.  Do not register tests that cannot collect:
  # pytest's module-level skip otherwise becomes a misleading CTest failure.
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import zmq"
    RESULT_VARIABLE test_python_has_zmq
    OUTPUT_QUIET ERROR_QUIET)
  # Never let a test discover a running robot or a parallel test. These are
  # distinct valid DDS domains, intentionally away from the live default (0).
  ament_add_pytest_test(test_odometry test/test_odometry.py ENV "${test_pythonpath};ROS_DOMAIN_ID=180")
  ament_add_pytest_test(test_motor_health test/test_motor_health.py ENV "${test_pythonpath};ROS_DOMAIN_ID=216")
  ament_add_pytest_test(test_odom_scale test/test_odom_scale.py ENV "${test_pythonpath};ROS_DOMAIN_ID=181")
  ament_add_pytest_test(test_free_space test/test_free_space.py ENV "${test_pythonpath};ROS_DOMAIN_ID=182")
  ament_add_pytest_test(test_camera_relay test/test_camera_relay.py ENV "${test_pythonpath};ROS_DOMAIN_ID=183")
  ament_add_pytest_test(test_astra_pro_configuration test/test_astra_pro_configuration.py ENV "${test_pythonpath};ROS_DOMAIN_ID=215")
  ament_add_pytest_test(test_astra_cloud_filter test/test_astra_cloud_filter.py ENV "${test_pythonpath};ROS_DOMAIN_ID=218")
  ament_add_pytest_test(test_remote_astra_service test/test_remote_astra_service.py ENV "${test_pythonpath};ROS_DOMAIN_ID=217")
  ament_add_pytest_test(test_arm_trajectory test/test_arm_trajectory.py ENV "${test_pythonpath};ROS_DOMAIN_ID=184")
  ament_add_pytest_test(test_collision_model test/test_collision_model.py ENV "${test_pythonpath};ROS_DOMAIN_ID=185")
  ament_add_pytest_test(test_driver_link test/test_driver_link.py ENV "${test_pythonpath};ROS_DOMAIN_ID=186")
  ament_add_pytest_test(test_cmd_vel_mux test/test_cmd_vel_mux.py ENV "${test_pythonpath};ROS_DOMAIN_ID=187")
  ament_add_pytest_test(test_teleop test/test_teleop.py ENV "${test_pythonpath};ROS_DOMAIN_ID=188")
  ament_add_pytest_test(test_readiness_gate test/test_readiness_gate.py ENV "${test_pythonpath};ROS_DOMAIN_ID=189")
  ament_add_pytest_test(test_sim_host_tools test/test_sim_host_tools.py ENV "${test_pythonpath};ROS_DOMAIN_ID=190")
  ament_add_pytest_test(test_rtabmap_db_maintenance test/test_rtabmap_db_maintenance.py ENV "${test_pythonpath};ROS_DOMAIN_ID=191")
  ament_add_pytest_test(test_torque_control test/test_torque_control.py ENV "${test_pythonpath};ROS_DOMAIN_ID=192")
  ament_add_pytest_test(test_lidar_detection test/test_lidar_detection.py ENV "${test_pythonpath};ROS_DOMAIN_ID=193")
  ament_add_pytest_test(test_safety_supervisor test/test_safety_supervisor.py ENV "${test_pythonpath};ROS_DOMAIN_ID=194")
  ament_add_pytest_test(test_arm_workspace_monitor test/test_arm_workspace_monitor.py ENV "${test_pythonpath};ROS_DOMAIN_ID=209")
  ament_add_pytest_test(test_map_bundle test/test_map_bundle.py ENV "${test_pythonpath};ROS_DOMAIN_ID=195")
  ament_add_pytest_test(test_launch_validation test/test_launch_validation.py ENV "${test_pythonpath};ROS_DOMAIN_ID=196")
  ament_add_pytest_test(test_rtabmap_session_guard test/test_rtabmap_session_guard.py ENV "${test_pythonpath};ROS_DOMAIN_ID=197")
  ament_add_pytest_test(test_rmf_owner_guard test/test_rmf_owner_guard.py
    ENV "${test_pythonpath};ROS_DOMAIN_ID=211")
  ament_add_pytest_test(test_moveit_shutdown_probe test/test_moveit_shutdown_probe.py
    ENV "${test_pythonpath};ROS_DOMAIN_ID=213")
  ament_add_pytest_test(test_sim_qualification test/test_sim_qualification.py
    ENV "${test_pythonpath};ROS_DOMAIN_ID=214")
  if(test_python_has_zmq EQUAL 0)
    ament_add_pytest_test(test_fake_host test/test_fake_host.py
      ENV "${test_pythonpath};ROS_DOMAIN_ID=198")
  endif()
  ament_add_pytest_test(test_zmq_security test/test_zmq_security.py ENV "${test_pythonpath};ROS_DOMAIN_ID=201")
  ament_add_pytest_test(test_simulation_model test/test_simulation_model.py
    ENV "${test_pythonpath};ROS_DOMAIN_ID=203")
  ament_add_pytest_test(test_service_installation test/test_service_installation.py
    ENV "${test_pythonpath};ROS_DOMAIN_ID=200")
  add_launch_test(test/test_cmd_vel_mux_launch.py
    TIMEOUT 30
    ENV "${test_launch_environment};ROS_DOMAIN_ID=199")
  # A module-level pytest skip is not represented correctly by launch_testing:
  # without pyzmq it reports a misleading missing-result error before launch.
  # CI installs python3-zmq; minimal ROS hosts may intentionally omit it.
  if(test_python_has_zmq EQUAL 0)
    add_launch_test(test/test_driver_fake_host_launch.py
      TIMEOUT 45
      ENV "${test_launch_environment};ROS_DOMAIN_ID=202")
  else()
    message(WARNING
      "pyzmq is unavailable to ${Python3_EXECUTABLE}; not registering ZMQ launch tests")
  endif()
  add_launch_test(test/test_simulation_physics_launch.py
    TIMEOUT 60
    ENV "${test_launch_environment};ROS_DOMAIN_ID=204")
  add_launch_test(test/test_sim_native_failsafe_launch.py
    TIMEOUT 45
    ENV "${test_launch_environment};ROS_DOMAIN_ID=206")
  # Both tests own a Gazebo server and make assertions against simulated-time
  # progress. Running the two servers concurrently on a small host can starve
  # physics updates and produce false motion/failsafe failures even though DDS
  # is isolated. Keep the Gazebo resource exclusive while allowing all other
  # tests to run in parallel.
  set_tests_properties(
    test_test_simulation_physics_launch.py
    test_test_sim_native_failsafe_launch.py
    PROPERTIES RESOURCE_LOCK lekiwi_gazebo_server)
  add_launch_test(test/test_safety_supervisor_launch.py
    TIMEOUT 30
    ENV "${test_launch_environment};ROS_DOMAIN_ID=205")
  add_launch_test(test/test_arm_workspace_monitor_launch.py
    TIMEOUT 30
    ENV "${test_launch_environment};ROS_DOMAIN_ID=210")
  add_launch_test(test/test_rmf_owner_guard_launch.py
    TIMEOUT 20
    ENV "${test_launch_environment};ROS_DOMAIN_ID=212")
  if(test_python_has_zmq EQUAL 0)
    # This is intentionally its own DDS graph: it launches MoveIt and the real
    # driver against the loopback-only fake motor host.
    add_launch_test(test/test_moveit_driver_e2e_launch.py
      TIMEOUT 90
      ENV "${test_launch_environment};ROS_DOMAIN_ID=208")
  endif()
endif()

ament_package()
