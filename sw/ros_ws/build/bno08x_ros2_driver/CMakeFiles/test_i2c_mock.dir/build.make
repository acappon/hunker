# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver

# Include any dependencies generated for this target.
include CMakeFiles/test_i2c_mock.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test_i2c_mock.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test_i2c_mock.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_i2c_mock.dir/flags.make

CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.o: CMakeFiles/test_i2c_mock.dir/flags.make
CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.o: /home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver/test/i2c_interface_mock_test.cpp
CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.o: CMakeFiles/test_i2c_mock.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.o -MF CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.o.d -o CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.o -c /home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver/test/i2c_interface_mock_test.cpp

CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver/test/i2c_interface_mock_test.cpp > CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.i

CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver/test/i2c_interface_mock_test.cpp -o CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.s

# Object files for target test_i2c_mock
test_i2c_mock_OBJECTS = \
"CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.o"

# External object files for target test_i2c_mock
test_i2c_mock_EXTERNAL_OBJECTS =

test_i2c_mock: CMakeFiles/test_i2c_mock.dir/test/i2c_interface_mock_test.cpp.o
test_i2c_mock: CMakeFiles/test_i2c_mock.dir/build.make
test_i2c_mock: gmock/libgmock_main.a
test_i2c_mock: gmock/libgmock.a
test_i2c_mock: /opt/ros/jazzy/lib/librclcpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_generator_py.so
test_i2c_mock: /opt/ros/jazzy/lib/liblibstatistics_collector.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl.so
test_i2c_mock: /opt/ros/jazzy/lib/librmw_implementation.so
test_i2c_mock: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_generator_py.so
test_i2c_mock: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_generator_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_generator_py.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_generator_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_yaml_param_parser.so
test_i2c_mock: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_generator_py.so
test_i2c_mock: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_generator_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_generator_py.so
test_i2c_mock: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_generator_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libtracetools.so
test_i2c_mock: /opt/ros/jazzy/lib/librcl_logging_interface.so
test_i2c_mock: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libstd_msgs__rosidl_generator_py.so
test_i2c_mock: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librosidl_typesupport_fastrtps_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librosidl_typesupport_fastrtps_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librmw.so
test_i2c_mock: /opt/ros/jazzy/lib/librosidl_dynamic_typesupport.so
test_i2c_mock: /opt/ros/jazzy/lib/libfastcdr.so.2.2.5
test_i2c_mock: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librosidl_typesupport_introspection_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librosidl_typesupport_introspection_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_generator_py.so
test_i2c_mock: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_generator_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_generator_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libstd_msgs__rosidl_generator_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/librosidl_typesupport_cpp.so
test_i2c_mock: /opt/ros/jazzy/lib/libservice_msgs__rosidl_generator_c.so
test_i2c_mock: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_generator_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librosidl_typesupport_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librcpputils.so
test_i2c_mock: /opt/ros/jazzy/lib/librosidl_runtime_c.so
test_i2c_mock: /opt/ros/jazzy/lib/librcutils.so
test_i2c_mock: CMakeFiles/test_i2c_mock.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_i2c_mock"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_i2c_mock.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_i2c_mock.dir/build: test_i2c_mock
.PHONY : CMakeFiles/test_i2c_mock.dir/build

CMakeFiles/test_i2c_mock.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_i2c_mock.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_i2c_mock.dir/clean

CMakeFiles/test_i2c_mock.dir/depend:
	cd /home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver /home/viking/hunker/sw/ros_ws/src/bno08x_ros2_driver /home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver /home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver /home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver/CMakeFiles/test_i2c_mock.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/test_i2c_mock.dir/depend

