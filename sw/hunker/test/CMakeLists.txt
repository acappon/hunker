cmake_minimum_required(VERSION 3.14)
project(hunker_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-Wno-psabi")

# Google Test
find_package(GTest REQUIRED)

# Stubs replace all hardware / ROS dependencies so the logic-only
# source files can compile and link in a plain Linux environment.
add_executable(hunker_unit_tests
    stubs.cpp
    test_fault_indicator.cpp
    test_robot.cpp
    test_balance_drive.cpp
    test_mygpio.cpp
    # Production sources under test
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/FaultIndicator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/Robot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/BalanceDrive.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/MyGpio.cpp
)

target_include_directories(hunker_unit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}          # for stub headers
)

target_link_libraries(hunker_unit_tests GTest::GTest GTest::Main pthread)
