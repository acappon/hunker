#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/viking/hunker/sw/ros_ws/install/setup.bash
ros2 launch bno08x_ros2_driver bno085_i2c.launch.py
