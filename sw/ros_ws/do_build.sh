#!/bin/bash
cd /home/viking/hunker/sw/ros_ws
source install/setup.bash
colcon build --packages-select bno08x_ros2_driver
cd /home/viking/hunker/sw/ros_ws/build/bno08x_ros2_driver
sudo make install

