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
    add_test(NAME test_native_waitable COMMAND test_native_waitable)
    set_tests_properties(test_native_waitable PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=83")
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
    set(test_environment "${test_launch_environment}")
    if(ARGC GREATER 1)
      list(APPEND test_environment "ROS_DOMAIN_ID=${ARGV1}")
    endif()
    ament_add_pytest_test(${name} test/${name}.py ENV "${test_environment}")
  endfunction()
  if(NOT LEKIWI_BUILD_COMPUTE)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/config/device_tests.txt")
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/config/device_tests.txt" device_tests)
    foreach(name IN LISTS device_tests)
      add_lekiwi_pytest(${name} 78)
    endforeach()
  else()
    # Never let a test discover a running robot or a parallel test. These are
    # distinct DDS domains (40-86), away from the live default (0) and
    # Linux ephemeral UDP ports (32768-60999). Higher domains collided with a
    # live node's ephemeral socket during discovery. Tests
    # without one never open a DDS participant.
    add_lekiwi_pytest(test_odometry 40)
    add_lekiwi_pytest(test_launch_calibration)
    add_lekiwi_pytest(test_motor_health 76)
    add_lekiwi_pytest(test_odom_scale 41)
    add_lekiwi_pytest(test_free_space 42)
    add_lekiwi_pytest(test_camera_relay 43)
    add_lekiwi_pytest(test_astra_pro_configuration 75)
    add_lekiwi_pytest(test_astra_cloud_filter 78)
    add_lekiwi_pytest(test_slam_cloud 79)
    add_lekiwi_pytest(test_remote_astra_service 77)
    add_lekiwi_pytest(test_arm_trajectory 44)
    add_lekiwi_pytest(test_local_arm_executor)
    add_lekiwi_pytest(test_arm_calibration)
    add_lekiwi_pytest(test_arm_jog 84)
    add_lekiwi_pytest(test_capture_stow)
    add_lekiwi_pytest(test_collision_model 45)
    add_lekiwi_pytest(test_vendor_lekiwi_model)
    add_lekiwi_pytest(test_driver_link 46)
    add_lekiwi_pytest(test_cmd_vel_mux 47)
    add_lekiwi_pytest(test_motion_guards)
    add_lekiwi_pytest(test_onboard_navigation)
    add_lekiwi_pytest(test_onboard_braking)
    add_lekiwi_pytest(test_teleop 48)
    add_lekiwi_pytest(test_readiness_gate 49)
    add_lekiwi_pytest(test_sim_host_tools 50)
    add_lekiwi_pytest(test_rtabmap_db_maintenance 51)
    add_lekiwi_pytest(test_torque_control 52)
    add_lekiwi_pytest(test_gripper_calibrate)
    add_lekiwi_pytest(test_lidar_detection 53)
    add_lekiwi_pytest(test_safety_supervisor 54)
    add_lekiwi_pytest(test_arm_workspace_monitor 69)
    add_lekiwi_pytest(test_map_bundle 55)
    add_lekiwi_pytest(test_launch_validation 56)
    add_lekiwi_pytest(test_rtabmap_session_guard 57)
    add_lekiwi_pytest(test_exploration)
    add_lekiwi_pytest(test_exploration_ros 85)
    add_lekiwi_pytest(test_exploration_nav2 86)
    add_lekiwi_pytest(test_rmf_owner_guard 71)
    add_lekiwi_pytest(test_moveit_shutdown_probe 73)
    add_lekiwi_pytest(test_moveit_reload)
    add_lekiwi_pytest(test_native_lifecycle 82)
    add_lekiwi_pytest(test_sim_qualification 74)
    if(test_python_has_zmq EQUAL 0)
      add_lekiwi_pytest(test_fake_host 58)
    endif()
    add_lekiwi_pytest(test_zmq_security 61)
    add_lekiwi_pytest(test_simulation_model 63)
    add_lekiwi_pytest(test_service_installation 60)
    add_lekiwi_pytest(test_release)
    add_lekiwi_pytest(test_scan_self_filter 80)
    add_lekiwi_pytest(test_camera_supervisor)
    add_lekiwi_pytest(test_moveit_cloud_gate)
    add_lekiwi_pytest(test_moveit_gripper_calibration)
    add_lekiwi_pytest(test_self_heal)
    add_lekiwi_pytest(test_zenoh_tls)
    add_lekiwi_pytest(test_foxglove_configuration)
    add_launch_test(test/test_cmd_vel_mux_launch.py
      TIMEOUT 30
      ENV "${test_launch_environment};ROS_DOMAIN_ID=59")
    # A module-level pytest skip is not represented correctly by launch_testing:
    # without pyzmq it reports a misleading missing-result error before launch.
    # CI installs python3-zmq; minimal ROS hosts may intentionally omit it.
    if(test_python_has_zmq EQUAL 0)
      add_launch_test(test/test_driver_fake_host_launch.py
        TIMEOUT 45
        ENV "${test_launch_environment};ROS_DOMAIN_ID=62")
    else()
      message(WARNING
        "pyzmq is unavailable to ${Python3_EXECUTABLE}; not registering ZMQ launch tests")
    endif()
    add_launch_test(test/test_simulation_physics_launch.py
      TIMEOUT 60
      ENV "${test_launch_environment};ROS_DOMAIN_ID=64")
    add_launch_test(test/test_sim_native_failsafe_launch.py
      TIMEOUT 45
      ENV "${test_launch_environment};ROS_DOMAIN_ID=66")
    add_launch_test(test/test_sim_sensor_frames_launch.py
      TIMEOUT 35
      ENV "${test_launch_environment};ROS_DOMAIN_ID=67")
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
      ENV "${test_launch_environment};ROS_DOMAIN_ID=65")
    add_launch_test(test/test_arm_workspace_monitor_launch.py
      TIMEOUT 30
      ENV "${test_launch_environment};ROS_DOMAIN_ID=70")
    add_launch_test(test/test_rmf_owner_guard_launch.py
      TIMEOUT 20
      ENV "${test_launch_environment};ROS_DOMAIN_ID=72")
    if(test_python_has_zmq EQUAL 0)
      # This is intentionally its own DDS graph: it launches MoveIt and the real
      # driver against the loopback-only fake motor host.
      add_launch_test(test/test_moveit_driver_e2e_launch.py
        TIMEOUT 90
        ENV "${test_launch_environment};ROS_DOMAIN_ID=68")
    endif()
  endif()
endif()

ament_package()
