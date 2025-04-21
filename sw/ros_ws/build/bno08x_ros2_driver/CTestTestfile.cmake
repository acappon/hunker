# CMake generated Testfile for 
# Source directory: /home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver
# Build directory: /home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_i2c_mock "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver/test_results/bno08x_ros2_driver/test_i2c_mock.gtest.xml" "--package-name" "bno08x_ros2_driver" "--output-file" "/home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver/ament_cmake_gmock/test_i2c_mock.txt" "--command" "/home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver/test_i2c_mock" "--gtest_output=xml:/home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver/test_results/bno08x_ros2_driver/test_i2c_mock.gtest.xml")
set_tests_properties(test_i2c_mock PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver/test_i2c_mock" TIMEOUT "60" WORKING_DIRECTORY "/home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gmock/cmake/ament_add_gmock_test.cmake;98;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;90;ament_add_gmock_test;/home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver/CMakeLists.txt;76;ament_add_gmock;/home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver/CMakeLists.txt;0;")
subdirs("include/sh2")
subdirs("gmock")
subdirs("gtest")
