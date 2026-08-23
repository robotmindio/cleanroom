cmake_minimum_required(VERSION 3.16)
project(lekiwi_rmf)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS) accepts a single RENAME value, so each renamed script gets its own call.
install(PROGRAMS lekiwi_rmf/driver.py DESTINATION lib/${PROJECT_NAME} RENAME lekiwi_driver)
install(PROGRAMS lekiwi_rmf/camera_relay.py DESTINATION lib/${PROJECT_NAME} RENAME camera_relay)
install(PROGRAMS scripts/arm_calibration.py scripts/odom_scale.py scripts/teleop.py scripts/free_space.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/setup-pi.bash DESTINATION share/${PROJECT_NAME}/scripts)
install(DIRECTORY config launch maps urdf worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Test the package under construction, not a similarly named package sourced
  # from another overlay workspace.
  set(test_pythonpath "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")
  ament_add_pytest_test(test_odometry test/test_odometry.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_odom_scale test/test_odom_scale.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_free_space test/test_free_space.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_arm_trajectory test/test_arm_trajectory.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_driver_link test/test_driver_link.py ENV "${test_pythonpath}")
  ament_add_pytest_test(test_teleop test/test_teleop.py ENV "${test_pythonpath}")
endif()

ament_package()
