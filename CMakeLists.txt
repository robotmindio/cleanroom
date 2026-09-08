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
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c
      "import numpy; raise SystemExit(int(numpy.__version__.split('.')[0]) >= 2)"
    RESULT_VARIABLE test_python_numpy_incompatible)
  if(NOT test_python_numpy_incompatible EQUAL 0)
    message(FATAL_ERROR
      "${Python3_EXECUTABLE} must use NumPy 1.x with ROS Jazzy compiled extensions; rerun scripts/install.sh")
  endif()
  # Test the package under construction, not a similarly named package sourced
  # from another overlay workspace.
  set(test_python_environment
    "PYTHONNOUSERSITE=1;PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")
  set(test_ament_prefix_path
    "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
  set(test_launch_environment "${test_python_environment};${test_ament_prefix_path}")
  # pyzmq is an intentional runtime dependency on the robot, but some minimal
  # ROS development hosts omit it.  Do not register tests that cannot collect:
  # pytest's module-level skip otherwise becomes a misleading CTest failure.
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env PYTHONNOUSERSITE=1
      "${Python3_EXECUTABLE}" -c "import zmq"
    RESULT_VARIABLE test_python_has_zmq
    OUTPUT_QUIET ERROR_QUIET)
  function(add_lekiwi_pytest name)
    set(test_environment "${test_launch_environment}")
    if(ARGC GREATER 1)
      list(APPEND test_environment "ROS_DOMAIN_ID=${ARGV1}")
    endif()
    ament_add_pytest_test(${name} test/${name}.py ENV "${test_environment}")
  endfunction()
  # Never let a test discover a running robot or a parallel test. These are
  # distinct valid DDS domains, intentionally away from the live default (0).
  add_lekiwi_pytest(test_odometry 180)
  add_lekiwi_pytest(test_motor_health 216)
  add_lekiwi_pytest(test_odom_scale 181)
  add_lekiwi_pytest(test_free_space 182)
  add_lekiwi_pytest(test_camera_relay 183)
  add_lekiwi_pytest(test_astra_pro_configuration 215)
  add_lekiwi_pytest(test_astra_cloud_filter 218)
  add_lekiwi_pytest(test_remote_astra_service 217)
  add_lekiwi_pytest(test_arm_trajectory 184)
  add_lekiwi_pytest(test_arm_calibration)
  add_lekiwi_pytest(test_collision_model 185)
  add_lekiwi_pytest(test_vendor_lekiwi_model)
  add_lekiwi_pytest(test_driver_link 186)
  add_lekiwi_pytest(test_cmd_vel_mux 187)
  add_lekiwi_pytest(test_teleop 188)
  add_lekiwi_pytest(test_readiness_gate 189)
  add_lekiwi_pytest(test_sim_host_tools 190)
  add_lekiwi_pytest(test_rtabmap_db_maintenance 191)
  add_lekiwi_pytest(test_torque_control 192)
  add_lekiwi_pytest(test_lidar_detection 193)
  add_lekiwi_pytest(test_safety_supervisor 194)
  add_lekiwi_pytest(test_arm_workspace_monitor 209)
  add_lekiwi_pytest(test_map_bundle 195)
  add_lekiwi_pytest(test_launch_validation 196)
  add_lekiwi_pytest(test_rtabmap_session_guard 197)
  add_lekiwi_pytest(test_rmf_owner_guard 211)
  add_lekiwi_pytest(test_moveit_shutdown_probe 213)
  add_lekiwi_pytest(test_sim_qualification 214)
  if(test_python_has_zmq EQUAL 0)
    add_lekiwi_pytest(test_fake_host 198)
  endif()
  add_lekiwi_pytest(test_zmq_security 201)
  add_lekiwi_pytest(test_simulation_model 203)
  add_lekiwi_pytest(test_service_installation 200)
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
  add_launch_test(test/test_sim_sensor_frames_launch.py
    TIMEOUT 35
    ENV "${test_launch_environment};ROS_DOMAIN_ID=219")
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
