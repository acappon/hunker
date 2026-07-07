# Add this line to silence the ABI notes
add_compile_options("-Wno-psabi")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

cmake_minimum_required(VERSION 3.5)
project(robot)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Set the directory containing ament_cmakeConfig.cmake
set(ament_cmake_DIR "/opt/ros/jazzy/share/ament_cmake/cmake")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})

# Find the lgpio library
find_package(lgpio REQUIRED)
add_executable(RobotNode src/main.cpp src/RobotNode.cpp src/FaultIndicator.cpp src/MyGpio.cpp src/BalanceDrive.cpp src/Robot.cpp)
ament_target_dependencies(RobotNode rclcpp std_msgs sensor_msgs)

# Link against the lgpio library
target_include_directories(RobotNode PRIVATE ${LGPIO_INCLUDE_DIRS})
target_link_libraries(RobotNode ${LGPIO_LIBRARIES})

# Link against the sparkcan library
target_include_directories(RobotNode PRIVATE /usr/local/include)
target_link_libraries(RobotNode sparkcan)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

install(TARGETS
  RobotNode
  DESTINATION lib/${PROJECT_NAME})

ament_package()