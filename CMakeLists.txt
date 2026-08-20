cmake_minimum_required(VERSION 3.16)
project(lekiwi_rmf)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS lekiwi_rmf/driver.py DESTINATION lib/${PROJECT_NAME} RENAME lekiwi_driver)
install(PROGRAMS scripts/arm_calibration.py scripts/odom_scale.py scripts/teleop.py scripts/free_space.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/setup-pi.bash DESTINATION share/${PROJECT_NAME}/scripts)
install(DIRECTORY config launch maps urdf worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_odometry test/test_odometry.py)
  ament_add_pytest_test(test_free_space test/test_free_space.py)
  ament_add_pytest_test(test_arm_trajectory test/test_arm_trajectory.py)
  ament_add_pytest_test(test_teleop test/test_teleop.py)
endif()

ament_package()
