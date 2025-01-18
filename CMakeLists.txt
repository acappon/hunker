cmake_minimum_required(VERSION 3.5)
project(hunker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the custom cmake directory to the module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find the lgpio library
find_package(lgpio REQUIRED)

add_executable(hunker_node src/RobotNode.cpp src/Robot.cpp src/Motor.cpp src/IMU_bno055.cpp)

ament_target_dependencies(hunker_node rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs)

# Link against the lgpio library
target_include_directories(hunker_node PRIVATE ${LGPIO_INCLUDE_DIRS})
target_link_libraries(hunker_node ${LGPIO_LIBRARIES})

install(TARGETS
  hunker_node
  DESTINATION lib/${PROJECT_NAME})

message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Install destination: ${CMAKE_INSTALL_PREFIX}/lib")

ament_package()