cmake_minimum_required(VERSION 3.16)
project(lekiwi_rmf)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "action/Explore.action" DEPENDENCIES geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
option(LEKIWI_BUILD_COMPUTE "Build compute-only Gazebo and MoveIt plugins" ON)
if(LEKIWI_BUILD_COMPUTE)
  find_package(gz-sim8 REQUIRED)
  find_package(moveit_core REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(moveit_ros_visualization REQUIRED)
  find_package(rviz_common REQUIRED)

  add_library(lekiwi_rest_collision SHARED src/rest_collision_plugin.cpp)
  target_compile_features(lekiwi_rest_collision PUBLIC cxx_std_17)
  ament_target_dependencies(lekiwi_rest_collision moveit_core pluginlib)
  pluginlib_export_plugin_description_file(moveit_core config/rest_collision_plugin.xml)
  install(TARGETS lekiwi_rest_collision LIBRARY DESTINATION lib)

  add_library(lekiwi_rest_motion_planning SHARED src/rest_motion_planning_display.cpp)
  target_compile_features(lekiwi_rest_motion_planning PUBLIC cxx_std_17)
  ament_target_dependencies(lekiwi_rest_motion_planning moveit_ros_visualization rviz_common pluginlib)
  target_link_libraries(lekiwi_rest_motion_planning moveit_ros_visualization::moveit_motion_planning_rviz_plugin_core)
  pluginlib_export_plugin_description_file(rviz_common config/rest_motion_planning_plugin.xml)
  install(TARGETS lekiwi_rest_motion_planning LIBRARY DESTINATION lib)

  add_library(lekiwi_sim_native_failsafe SHARED src/sim_native_failsafe.cpp)
  target_compile_features(lekiwi_sim_native_failsafe PUBLIC cxx_std_17)
  target_link_libraries(lekiwi_sim_native_failsafe gz-sim8::gz-sim8)
  install(TARGETS lekiwi_sim_native_failsafe LIBRARY DESTINATION lib/${PROJECT_NAME})
endif()

# rosidl owns the Python package installation; install application modules into
# it without a second ament_python_install_package target of the same name.
install(DIRECTORY lekiwi_rmf/ DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.py" PATTERN "__init__.py" EXCLUDE PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS lekiwi_rmf/exploration_node.py DESTINATION lib/${PROJECT_NAME} RENAME robot_explorer)
install(PROGRAMS lekiwi_rmf/exploration_owner_guard.py DESTINATION lib/${PROJECT_NAME} RENAME exploration_owner_guard)
# install(PROGRAMS) accepts a single RENAME value, so each renamed script gets its own call.
install(PROGRAMS lekiwi_rmf/driver.py DESTINATION lib/${PROJECT_NAME} RENAME lekiwi_driver)
install(PROGRAMS lekiwi_rmf/astra_cloud_filter.py DESTINATION lib/${PROJECT_NAME} RENAME astra_cloud_filter)
install(PROGRAMS lekiwi_rmf/camera_relay.py DESTINATION lib/${PROJECT_NAME} RENAME camera_relay)
install(PROGRAMS lekiwi_rmf/cmd_vel_mux.py DESTINATION lib/${PROJECT_NAME} RENAME cmd_vel_mux)
install(PROGRAMS lekiwi_rmf/readiness_gate.py DESTINATION lib/${PROJECT_NAME} RENAME readiness_gate)
install(PROGRAMS lekiwi_rmf/slam_cloud.py DESTINATION lib/${PROJECT_NAME} RENAME slam_cloud)
install(PROGRAMS lekiwi_rmf/scan_self_filter.py DESTINATION lib/${PROJECT_NAME} RENAME scan_self_filter)
install(PROGRAMS lekiwi_rmf/moveit_cloud_gate.py DESTINATION lib/${PROJECT_NAME} RENAME moveit_cloud_gate)
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
  scripts/reload-moveit.py
  scripts/rtabmap-db-maintenance.py
  scripts/rtabmap-session-guard.py
  scripts/torque-host.py
  scripts/validate-map-bundle.py
  scripts/sim-renderer-check.py
  scripts/sim-scan-check.py
  scripts/generate-zmq-keys.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/camera-supervisor.sh scripts/ros-astra.sh scripts/ros-lidar.sh
  DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES scripts/lib/runtime-common.sh scripts/lib/self-heal.sh
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
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  if(LEKIWI_BUILD_COMPUTE)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_action REQUIRED)
    find_package(control_msgs REQUIRED)
    add_executable(test_native_waitable test/test_native_waitable.cpp)
    ament_target_dependencies(test_native_waitable rclcpp rclcpp_action control_msgs)
    add_executable(test_rest_contacts test/test_rest_contacts.cpp)
    target_compile_features(test_rest_contacts PUBLIC cxx_std_17)
    ament_target_dependencies(test_rest_contacts moveit_core)
    find_package(launch_testing_ament_cmake REQUIRED)
    execute_process(COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/urdf/lekiwi.urdf.xacro sim:=false
      OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/rest-test.urdf COMMAND_ERROR_IS_FATAL ANY)
    add_test(NAME test_rest_contacts COMMAND test_rest_contacts
      ${CMAKE_CURRENT_BINARY_DIR}/rest-test.urdf ${CMAKE_CURRENT_SOURCE_DIR}/config/lekiwi.srdf)
  endif()
  find_package(ament_cmake_pytest REQUIRED)
  # Every test that may open a DDS participant runs through ament_cmake_ros'
  # isolated runner, which reserves a free ROS_DOMAIN_ID (1-100) for the test's
  # duration, so parallel tests and concurrent ctest runs never share a graph.
  # Domains up to 100 keep Cyclone's discovery ports below Linux's ephemeral
  # range (32768+). An inherited ROS_DOMAIN_ID -- a shell that talks to the
  # live robot on domain 0 -- or DISABLE_ROS_ISOLATION must never reach a
  # test, so both are removed before the runner chooses.
  find_package(ament_cmake_ros REQUIRED)
  set(test_isolated_runner "${ament_cmake_ros_DIR}/run_test_isolated.py")
  function(isolate_lekiwi_test name)
    set_tests_properties(${name} PROPERTIES
      ENVIRONMENT_MODIFICATION "ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:")
  endfunction()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env PYTHONNOUSERSITE=1
      "${Python3_EXECUTABLE}" -c
      "import numpy; raise SystemExit(int(numpy.__version__.split('.')[0]) >= 2)"
    RESULT_VARIABLE test_python_numpy_incompatible)
  if(NOT test_python_numpy_incompatible EQUAL 0)
    message(FATAL_ERROR
      "${Python3_EXECUTABLE} must use NumPy 1.x with ROS Jazzy compiled extensions; rerun scripts/install.sh")
  endif()
  # Test the package under construction, not a similarly named package sourced
  # from another overlay workspace.
  set(test_python_environment
    "PYTHONNOUSERSITE=1;PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test:${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}:$ENV{PYTHONPATH};LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
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
    ament_add_ros_isolated_pytest_test(${name} test/${name}.py ENV "${test_launch_environment}")
    isolate_lekiwi_test(${name})
  endfunction()
  function(add_lekiwi_launch_test name timeout)
    add_launch_test(test/${name}.py TIMEOUT ${timeout} RUNNER "${test_isolated_runner}"
      ENV "${test_launch_environment}")
    isolate_lekiwi_test(test_${name}.py)
  endfunction()
  if(LEKIWI_BUILD_COMPUTE)
    ament_add_test(test_native_waitable
      COMMAND "$<TARGET_FILE:test_native_waitable>"
      RUNNER "${test_isolated_runner}"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
    isolate_lekiwi_test(test_native_waitable)
  endif()
  if(NOT LEKIWI_BUILD_COMPUTE)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/config/device_tests.txt")
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/config/device_tests.txt" device_tests)
    foreach(name IN LISTS device_tests)
      add_lekiwi_pytest(${name})
    endforeach()
  else()
    add_lekiwi_pytest(test_odometry)
    add_lekiwi_pytest(test_launch_calibration)
    add_lekiwi_pytest(test_motor_health)
    add_lekiwi_pytest(test_odom_scale)
    add_lekiwi_pytest(test_free_space)
    add_lekiwi_pytest(test_camera_relay)
    add_lekiwi_pytest(test_astra_pro_configuration)
    add_lekiwi_pytest(test_astra_cloud_filter)
    add_lekiwi_pytest(test_slam_cloud)
    add_lekiwi_pytest(test_remote_astra_service)
    add_lekiwi_pytest(test_arm_trajectory)
    add_lekiwi_pytest(test_local_arm_executor)
    add_lekiwi_pytest(test_arm_calibration)
    add_lekiwi_pytest(test_arm_jog)
    add_lekiwi_pytest(test_capture_stow)
    add_lekiwi_pytest(test_collision_model)
    add_lekiwi_pytest(test_vendor_lekiwi_model)
    add_lekiwi_pytest(test_driver_link)
    add_lekiwi_pytest(test_cmd_vel_mux)
    add_lekiwi_pytest(test_motion_guards)
    add_lekiwi_pytest(test_collision_monitor)
    add_lekiwi_pytest(test_onboard_navigation)
    add_lekiwi_pytest(test_onboard_braking)
    add_lekiwi_pytest(test_teleop)
    add_lekiwi_pytest(test_readiness_gate)
    add_lekiwi_pytest(test_sim_host_tools)
    add_lekiwi_pytest(test_rtabmap_db_maintenance)
    add_lekiwi_pytest(test_torque_control)
    add_lekiwi_pytest(test_gripper_calibrate)
    add_lekiwi_pytest(test_lidar_detection)
    add_lekiwi_pytest(test_safety_supervisor)
    add_lekiwi_pytest(test_arm_workspace_monitor)
    add_lekiwi_pytest(test_map_bundle)
    add_lekiwi_pytest(test_launch_validation)
    add_lekiwi_pytest(test_bringup_snapshot)
    add_lekiwi_pytest(test_safety_profiles)
    add_lekiwi_pytest(test_rtabmap_session_guard)
    add_lekiwi_pytest(test_exploration)
    add_lekiwi_pytest(test_explore_launcher)
    add_lekiwi_pytest(test_exploration_recovery)
    add_lekiwi_pytest(test_exploration_owner_guard)
    add_lekiwi_pytest(test_exploration_ros)
    add_lekiwi_pytest(test_exploration_nav2)
    add_lekiwi_pytest(test_exploration_planner)
    add_lekiwi_pytest(test_rmf_owner_guard)
    add_lekiwi_pytest(test_moveit_shutdown_probe)
    add_lekiwi_pytest(test_moveit_reload)
    add_lekiwi_pytest(test_native_lifecycle)
    add_lekiwi_pytest(test_sim_qualification)
    if(test_python_has_zmq EQUAL 0)
      add_lekiwi_pytest(test_fake_host)
    endif()
    add_lekiwi_pytest(test_zmq_security)
    add_lekiwi_pytest(test_simulation_model)
    add_lekiwi_pytest(test_service_installation)
    add_lekiwi_pytest(test_release)
    add_lekiwi_pytest(test_scan_self_filter)
    add_lekiwi_pytest(test_camera_supervisor)
    add_lekiwi_pytest(test_moveit_cloud_gate)
    add_lekiwi_pytest(test_moveit_gripper_calibration)
    add_lekiwi_pytest(test_self_heal)
    add_lekiwi_pytest(test_zenoh_tls)
    add_lekiwi_pytest(test_foxglove_configuration)
    add_lekiwi_launch_test(test_cmd_vel_mux_launch 30)
    # A module-level pytest skip is not represented correctly by launch_testing:
    # without pyzmq it reports a misleading missing-result error before launch.
    # CI installs python3-zmq; minimal ROS hosts may intentionally omit it.
    if(test_python_has_zmq EQUAL 0)
      add_lekiwi_launch_test(test_driver_fake_host_launch 45)
    else()
      message(WARNING
        "pyzmq is unavailable to ${Python3_EXECUTABLE}; not registering ZMQ launch tests")
    endif()
    add_lekiwi_launch_test(test_simulation_physics_launch 60)
    add_lekiwi_launch_test(test_sim_native_failsafe_launch 45)
    add_lekiwi_launch_test(test_sim_sensor_frames_launch 35)
    # Both tests own a Gazebo server and make assertions against simulated-time
    # progress. Running the two servers concurrently on a small host can starve
    # physics updates and produce false motion/failsafe failures even though DDS
    # is isolated. Keep the Gazebo resource exclusive while allowing all other
    # tests to run in parallel.
    set_tests_properties(
      test_test_simulation_physics_launch.py
      test_test_sim_native_failsafe_launch.py
      PROPERTIES RESOURCE_LOCK lekiwi_gazebo_server)
    add_lekiwi_launch_test(test_safety_supervisor_launch 30)
    add_lekiwi_launch_test(test_arm_workspace_monitor_launch 30)
    add_lekiwi_launch_test(test_rmf_owner_guard_launch 20)
    if(test_python_has_zmq EQUAL 0)
      # This is intentionally its own DDS graph: it launches MoveIt and the real
      # driver against the loopback-only fake motor host.
      add_lekiwi_launch_test(test_moveit_driver_e2e_launch 90)
    endif()
  endif()
endif()

ament_package()
